/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci_types.h>

#if IS_ENABLED(CONFIG_SETTINGS)

#include <zephyr/settings/settings.h>

#endif

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/ble.h>
#include <zmk/keys.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/activity_state_changed.h>

#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)

#define PASSKEY_DIGITS 6

static struct bt_conn *auth_passkey_entry_conn;
RING_BUF_DECLARE(passkey_entries, PASSKEY_DIGITS);

#endif /* IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY) */

enum advertising_type advertising_status;

#define CURR_ADV(adv) (adv << 4)

#define ZMK_ADV_CONN_NAME                                                                          \
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)

/* Reconnect ("stealth") advertising for a bonded but disconnected profile:
 * connectable undirected ADV_IND, which every host type answers (Apple's
 * accessory guidelines forbid directed advertising to them), carrying the
 * non-discoverable zmk_ble_stealth_ad payload below so the keyboard stays out
 * of everyone else's pairing UIs. Slow cadence matches the open advertisement;
 * the burst variant runs at the LL minimum interval (0x20 * 0.625 ms = 20 ms,
 * plus the controller's 0-10 ms advDelay) so a scanning host hears the very
 * first packets and reconnection completes in tens of milliseconds.
 */
#define ZMK_ADV_CONN_STEALTH_SLOW                                                                  \
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)

#define ZMK_ADV_CONN_STEALTH_BURST BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, 0x0020, 0x0020, NULL)

static struct zmk_ble_profile profiles[ZMK_BLE_PROFILE_COUNT];
static uint8_t active_profile;

static bool permit_adv = true;

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

BUILD_ASSERT(
    DEVICE_NAME_LEN <= CONFIG_BT_DEVICE_NAME_MAX,
    "ERROR: BLE device name is too long. Max length: " STRINGIFY(CONFIG_BT_DEVICE_NAME_MAX));

static struct bt_data zmk_ble_ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, BT_BYTES_LIST_LE16(CONFIG_BT_DEVICE_APPEARANCE)),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_SOME, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL), /* HID Service */
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)                        /* Battery Service */
                  ),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* AD payload for the reconnect advertisement. GAP non-discoverable mode is the
 * load-bearing part: with both discoverable-mode bits of the Flags AD clear,
 * hosts' discovery procedures (Core v6.0 Vol 3 Part C 9.2) are required to
 * hide us from pairing UIs. Everything identifying is stripped on top of that:
 * the bonded host reconnects purely by (IRK-resolved) address and never parses
 * this payload, so it needs no name, appearance or service UUIDs. Developer
 * scanners see the packets regardless; the goal is pairing-UI invisibility,
 * not RF invisibility. Design notes: ~/clone/aster/plans/reconnectable.md. */
static const struct bt_data zmk_ble_stealth_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
#if IS_ENABLED(CONFIG_ZMK_BLE_STEALTH_ADV_NAME)
    /* Compat hedge for hosts that mishandle nameless adverts; see Kconfig. */
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
#endif
};

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

static bt_addr_le_t peripheral_addrs[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) */

static void apply_link_params(void);

static void raise_profile_changed_event(void) {
    /* The active profile decides which link runs at HID speed and which idle
     * at the long interval (see apply_link_params); every path that changes
     * it ends up here. */
    apply_link_params();
    raise_zmk_ble_active_profile_changed((struct zmk_ble_active_profile_changed){
        .index = active_profile, .profile = &profiles[active_profile]});
}

static void raise_profile_changed_event_callback(struct k_work *work) {
    raise_profile_changed_event();
}

K_WORK_DEFINE(raise_profile_changed_event_work, raise_profile_changed_event_callback);

bool zmk_ble_active_profile_is_open(void) { return zmk_ble_profile_is_open(active_profile); }

bool zmk_ble_profile_is_open(uint8_t index) {
    if (index >= ZMK_BLE_PROFILE_COUNT) {
        return false;
    }
    return !bt_addr_le_cmp(&profiles[index].peer, BT_ADDR_LE_ANY);
}

void set_profile_address(uint8_t index, const bt_addr_le_t *addr) {
    char setting_name[17];
    char addr_str[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    memcpy(&profiles[index].peer, addr, sizeof(bt_addr_le_t));
    sprintf(setting_name, "ble/profiles/%d", index);
    LOG_DBG("Setting profile addr for %s to %s", setting_name, addr_str);
#if IS_ENABLED(CONFIG_SETTINGS)
    settings_save_one(setting_name, &profiles[index], sizeof(struct zmk_ble_profile));
#endif
    k_work_submit(&raise_profile_changed_event_work);
}

/* Look up a peer's connection, but only once it is actually established.
 *
 * Between the controller accepting a CONNECT_IND and the connection completing
 * (or failing) the host already holds a conn object for the peer, and
 * bt_conn_lookup_addr_le() happily returns it. Callers that mean "is this host
 * reachable" must not mistake that for a live link. Returns a referenced conn
 * (caller unrefs) or NULL.
 */
static struct bt_conn *lookup_connected_addr_le(const bt_addr_le_t *addr) {
    struct bt_conn *conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);
    struct bt_conn_info info;

    if (conn == NULL) {
        return NULL;
    }

    bt_conn_get_info(conn, &info);
    if (info.state != BT_CONN_STATE_CONNECTED) {
        bt_conn_unref(conn);
        return NULL;
    }

    return conn;
}

bool zmk_ble_active_profile_is_connected(void) {
    return zmk_ble_profile_is_connected(active_profile);
}

bool zmk_ble_profile_is_connected(uint8_t index) {
    if (index >= ZMK_BLE_PROFILE_COUNT) {
        return false;
    }
    struct bt_conn *conn;
    bt_addr_le_t *addr = &profiles[index].peer;
    if (!bt_addr_le_cmp(addr, BT_ADDR_LE_ANY)) {
        return false;
    } else if ((conn = lookup_connected_addr_le(addr)) == NULL) {
        return false;
    }

    bt_conn_unref(conn);

    return true;
}

/* The stealth advertisement on the air is the fast burst variant. Only
 * meaningful while advertising_status == ZMK_ADV_STEALTH. */
static bool stealth_adv_bursting;

int update_advertising(void);

#if IS_ENABLED(CONFIG_ZMK_BLE_STEALTH_ADV_BURST)

/* Every fresh reconnect opportunity (boot with a bond, disconnect, profile
 * switch, adv re-enable, key activity) arms a burst window: stealth
 * advertising started while it is open runs at the 20 ms burst interval, and
 * the slowdown work drops the advert to the slow cadence when it closes. */
static int64_t adv_burst_until;
/* Uptime of the last window arm, for the activity-kick cooldown. */
static int64_t adv_burst_armed_at;

static void adv_burst_arm(void) {
    adv_burst_armed_at = k_uptime_get();
    adv_burst_until = adv_burst_armed_at + CONFIG_ZMK_BLE_STEALTH_ADV_BURST_MS;
}

static bool adv_burst_window_open(void) { return k_uptime_get() < adv_burst_until; }

/* Undirected advertising has no controller timeout, so ending the burst is on
 * us: re-evaluate once the window closes, and update_advertising() restarts
 * the advert at the slow cadence (the burst-state mismatch in ZMK_ADV_STEALTH
 * + CURR_ADV(ZMK_ADV_STEALTH)). */
static void adv_slowdown_callback(struct k_work *work) { update_advertising(); }
static K_WORK_DELAYABLE_DEFINE(adv_slowdown_work, adv_slowdown_callback);

static void adv_slowdown_schedule(void) {
    k_work_reschedule(&adv_slowdown_work, K_MSEC(MAX(adv_burst_until - k_uptime_get(), 0)));
}

#else

static void adv_burst_arm(void) {}
static bool adv_burst_window_open(void) { return false; }
static void adv_slowdown_schedule(void) {}

#endif /* IS_ENABLED(CONFIG_ZMK_BLE_STEALTH_ADV_BURST) */

#define CHECKED_ADV_STOP()                                                                         \
    err = bt_le_adv_stop();                                                                        \
    advertising_status = ZMK_ADV_NONE;                                                             \
    stealth_adv_bursting = false;                                                                  \
    if (err) {                                                                                     \
        LOG_ERR("Failed to stop advertising (err %d)", err);                                       \
        return err;                                                                                \
    }

#define CHECKED_OPEN_ADV()                                                                         \
    err = bt_le_adv_start(ZMK_ADV_CONN_NAME, zmk_ble_ad, ARRAY_SIZE(zmk_ble_ad), NULL, 0);         \
    if (err) {                                                                                     \
        LOG_ERR("Advertising failed to start (err %d)", err);                                      \
        return err;                                                                                \
    }                                                                                              \
    advertising_status = ZMK_ADV_CONN;

/* The reconnect advertisement is peer-agnostic (undirected, nothing
 * peer-specific in the payload), so nothing here depends on which profile is
 * active; the lookup guard only keeps us from tearing advertising down
 * underneath a connection that is mid-establishment. No fallback on failure:
 * unlike directed advertising there is no exotic feature to be missing, so
 * anything that fails this start would fail an open start the same way.
 */
#define CHECKED_STEALTH_ADV()                                                                      \
    addr = zmk_ble_active_profile_addr();                                                          \
    conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);                                            \
    if (conn != NULL) { /* connected, or a connection is being established */                      \
        LOG_DBG("Skipping advertising, profile host already has a conn object");                   \
        bt_conn_unref(conn);                                                                       \
        return 0;                                                                                  \
    }                                                                                              \
    burst = adv_burst_window_open();                                                               \
    err = bt_le_adv_start(burst ? ZMK_ADV_CONN_STEALTH_BURST : ZMK_ADV_CONN_STEALTH_SLOW,          \
                          zmk_ble_stealth_ad, ARRAY_SIZE(zmk_ble_stealth_ad), NULL, 0);            \
    if (err) {                                                                                     \
        LOG_ERR("Stealth advertising failed to start (err %d)", err);                              \
        return err;                                                                                \
    }                                                                                              \
    advertising_status = ZMK_ADV_STEALTH;                                                          \
    stealth_adv_bursting = burst;                                                                  \
    if (burst) {                                                                                   \
        adv_slowdown_schedule();                                                                   \
    }

int update_advertising(void) {
    int err = 0;
    bt_addr_le_t *addr;
    struct bt_conn *conn;
    bool burst;
    enum advertising_type desired_adv = ZMK_ADV_NONE;

    if (permit_adv && zmk_ble_active_profile_is_open()) {
        desired_adv = ZMK_ADV_CONN;
    } else if (permit_adv && !zmk_ble_active_profile_is_connected()) {
        // The profile is taken, so only its host has any business connecting.
        // Non-discoverable advertising keeps us out of everyone else's pairing
        // UIs while staying connectable for the bonded host.
        desired_adv = IS_ENABLED(CONFIG_ZMK_BLE_STEALTH_ADV) ? ZMK_ADV_STEALTH : ZMK_ADV_CONN;
    }
    LOG_DBG("advertising from %d to %d", advertising_status, desired_adv);

    switch (desired_adv + CURR_ADV(advertising_status)) {
    case ZMK_ADV_NONE + CURR_ADV(ZMK_ADV_STEALTH):
    case ZMK_ADV_NONE + CURR_ADV(ZMK_ADV_CONN):
        CHECKED_ADV_STOP();
        break;
    case ZMK_ADV_STEALTH + CURR_ADV(ZMK_ADV_STEALTH):
        /* Already advertising stealth. The advert is peer-agnostic, so even a
         * profile switch needs no restart for correctness -- only a cadence
         * change does: the burst window closed under a bursting advert (drop
         * to slow; the slowdown work lands here) or a fresh window was armed
         * under a slow one (an activity kick; speed back up). With nothing to
         * change, leave the advert strictly alone: a stop/start drops
         * whatever CONNECT_IND is in flight and sends the host back to its
         * (slow, duty-cycled) reconnect scan, and daisy re-affirms
         * advertising on every USB event -- that used to tear the advert down
         * repeatedly at exactly the moment a dongle was answering it. */
        if (stealth_adv_bursting == adv_burst_window_open()) {
            LOG_DBG("Already advertising stealth at the right cadence");
            break;
        }
        CHECKED_ADV_STOP();
        CHECKED_STEALTH_ADV();
        break;
    case ZMK_ADV_STEALTH + CURR_ADV(ZMK_ADV_CONN):
        CHECKED_ADV_STOP();
        CHECKED_STEALTH_ADV();
        break;
    case ZMK_ADV_STEALTH + CURR_ADV(ZMK_ADV_NONE):
        CHECKED_STEALTH_ADV();
        break;
    case ZMK_ADV_CONN + CURR_ADV(ZMK_ADV_STEALTH):
        CHECKED_ADV_STOP();
        CHECKED_OPEN_ADV();
        break;
    case ZMK_ADV_CONN + CURR_ADV(ZMK_ADV_NONE):
        CHECKED_OPEN_ADV();
        break;
    }

    return 0;
};

static void update_advertising_callback(struct k_work *work) { update_advertising(); }

K_WORK_DEFINE(update_advertising_work, update_advertising_callback);

void zmk_ble_adv_enabled_set(bool adv_enabled) {
    if (adv_enabled) {
        if (!permit_adv) {
            permit_adv = true;
            /* A genuine off->on transition (wired mode released BLE, factory
             * mode exited) is a fresh reconnect opportunity; the re-affirming
             * calls that arrive while already enabled are not. */
            adv_burst_arm();
            LOG_DBG("Enabling adv");
        }
        update_advertising();
    } else {
        permit_adv = false;
        LOG_DBG("Disabling adv and disconnecting");
        for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
            int err = zmk_ble_prof_disconnect(i);
            if (err) {
                LOG_DBG("Failed to disconnect profile %d : %d", i, err);
            }
        }
        update_advertising();
    }
}

bool zmk_ble_adv_enabled_get(void) { return permit_adv; }

static void clear_profile_bond(uint8_t profile) {
    if (bt_addr_le_cmp(&profiles[profile].peer, BT_ADDR_LE_ANY)) {
        bt_unpair(BT_ID_DEFAULT, &profiles[profile].peer);
        set_profile_address(profile, BT_ADDR_LE_ANY);
    }
}

void zmk_ble_clear_bonds(void) {
    LOG_DBG("zmk_ble_clear_bonds()");

    clear_profile_bond(active_profile);
    update_advertising();
};

void zmk_ble_clear_all_bonds(void) {
    LOG_DBG("zmk_ble_clear_all_bonds()");

    // Unpair all profiles
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        clear_profile_bond(i);
    }

    // Automatically switch to profile 0
    zmk_ble_prof_select(0);
    update_advertising();
};

int zmk_ble_active_profile_index(void) { return active_profile; }

int zmk_ble_profile_index(const bt_addr_le_t *addr) {
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        if (bt_addr_le_cmp(addr, &profiles[i].peer) == 0) {
            return i;
        }
    }
    return -ENODEV;
}

bt_addr_le_t *zmk_ble_profile_address(uint8_t index) {
    if (index >= ZMK_BLE_PROFILE_COUNT) {
        return (bt_addr_le_t *)(BT_ADDR_LE_NONE);
    }
    return &profiles[index].peer;
}

#if IS_ENABLED(CONFIG_SETTINGS)
static void ble_save_profile_work(struct k_work *work) {
    settings_save_one("ble/active_profile", &active_profile, sizeof(active_profile));
}

static struct k_work_delayable ble_save_work;
#endif

static int ble_save_profile(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    return k_work_reschedule(&ble_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
#else
    return 0;
#endif
}

int zmk_ble_prof_select(uint8_t index) {
    if (index >= ZMK_BLE_PROFILE_COUNT) {
        return -ERANGE;
    }

    LOG_DBG("profile %d", index);
    if (active_profile == index) {
        return 0;
    }

    active_profile = index;
    ble_save_profile();

    adv_burst_arm();
    update_advertising();

    raise_profile_changed_event();

    return 0;
};

int zmk_ble_save_profile_immediate(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    // Cancel any pending debounced save (its value would match) and persist the
    // active profile now, so a reboot immediately after does not lose it.
    k_work_cancel_delayable(&ble_save_work);
    return settings_save_one("ble/active_profile", &active_profile, sizeof(active_profile));
#else
    return 0;
#endif
}

int zmk_ble_prof_next(void) {
    LOG_DBG("");
    return zmk_ble_prof_select((active_profile + 1) % ZMK_BLE_PROFILE_COUNT);
};

int zmk_ble_prof_prev(void) {
    LOG_DBG("");
    return zmk_ble_prof_select((active_profile + ZMK_BLE_PROFILE_COUNT - 1) %
                               ZMK_BLE_PROFILE_COUNT);
};

int zmk_ble_prof_disconnect(uint8_t index) {
    if (index >= ZMK_BLE_PROFILE_COUNT)
        return -ERANGE;

    bt_addr_le_t *addr = &profiles[index].peer;
    struct bt_conn *conn;
    int result;

    if (!bt_addr_le_cmp(addr, BT_ADDR_LE_ANY)) {
        return -ENODEV;
    } else if ((conn = lookup_connected_addr_le(addr)) == NULL) {
        return -ENODEV;
    }

    result = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    LOG_DBG("Disconnected from profile %d: %d", index, result);

    bt_conn_unref(conn);
    return result;
}

bt_addr_le_t *zmk_ble_active_profile_addr(void) { return &profiles[active_profile].peer; }

struct bt_conn *zmk_ble_active_profile_conn(void) {
    struct bt_conn *conn;
    bt_addr_le_t *addr = zmk_ble_active_profile_addr();

    if (!bt_addr_le_cmp(addr, BT_ADDR_LE_ANY)) {
        LOG_WRN("Not sending, no active address for current profile");
        return NULL;
    } else if ((conn = lookup_connected_addr_le(addr)) == NULL) {
        LOG_WRN("Not sending, not connected to active profile");
        return NULL;
    }

    return conn;
}

char *zmk_ble_active_profile_name(void) { return profiles[active_profile].name; }

int zmk_ble_set_device_name(char *name) {
    // Copy new name to advertising parameters
    int err = bt_set_name(name);
    LOG_DBG("New device name: %s", name);
    if (err) {
        LOG_ERR("Failed to set new device name (err %d)", err);
        return err;
    }
    if (advertising_status == ZMK_ADV_CONN) {
        // Stop current advertising so it can restart with new name
        err = bt_le_adv_stop();
        advertising_status = ZMK_ADV_NONE;
        if (err) {
            LOG_ERR("Failed to stop advertising (err %d)", err);
            return err;
        }
    }
    return update_advertising();
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

int zmk_ble_put_peripheral_addr(const bt_addr_le_t *addr) {
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        // If the address is recognized and already stored in settings, return
        // index and no additional action is necessary.
        if (bt_addr_le_cmp(&peripheral_addrs[i], addr) == 0) {
            LOG_DBG("Found existing peripheral address in slot %d", i);
            return i;
        } else {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(&peripheral_addrs[i], addr_str, sizeof(addr_str));
            LOG_DBG("peripheral slot %d occupied by %s", i, addr_str);
        }

        // If the peripheral address slot is open, store new peripheral in the
        // slot and return index. This compares against BT_ADDR_LE_ANY as that
        // is the zero value.
        if (bt_addr_le_cmp(&peripheral_addrs[i], BT_ADDR_LE_ANY) == 0) {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
            LOG_DBG("Storing peripheral %s in slot %d", addr_str, i);
            bt_addr_le_copy(&peripheral_addrs[i], addr);

#if IS_ENABLED(CONFIG_SETTINGS)
            char setting_name[32];
            sprintf(setting_name, "ble/peripheral_addresses/%d", i);
            settings_save_one(setting_name, addr, sizeof(bt_addr_le_t));
#endif // IS_ENABLED(CONFIG_SETTINGS)
            return i;
        }
    }

    // The peripheral does not match a known peripheral and there is no
    // available slot.
    return -ENOMEM;
}

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) */

#if IS_ENABLED(CONFIG_SETTINGS)

static int ble_profiles_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg) {
    const char *next;

    LOG_DBG("Setting BLE value %s", name);

    if (settings_name_steq(name, "profiles", &next) && next) {
        char *endptr;
        uint8_t idx = strtoul(next, &endptr, 10);
        if (*endptr != '\0') {
            LOG_WRN("Invalid profile index: %s", next);
            return -EINVAL;
        }

        if (len != sizeof(struct zmk_ble_profile)) {
            LOG_ERR("Invalid profile size (got %d expected %d)", len,
                    sizeof(struct zmk_ble_profile));
            return -EINVAL;
        }

        if (idx >= ZMK_BLE_PROFILE_COUNT) {
            LOG_WRN("Profile address for index %d is larger than max of %d", idx,
                    ZMK_BLE_PROFILE_COUNT);
            return -EINVAL;
        }

        int err = read_cb(cb_arg, &profiles[idx], sizeof(struct zmk_ble_profile));
        if (err <= 0) {
            LOG_ERR("Failed to handle profile address from settings (err %d)", err);
            return err;
        }

        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&profiles[idx].peer, addr_str, sizeof(addr_str));

        LOG_DBG("Loaded %s address for profile %d", addr_str, idx);
    } else if (settings_name_steq(name, "active_profile", &next) && !next) {
        if (len != sizeof(active_profile)) {
            return -EINVAL;
        }

        int err = read_cb(cb_arg, &active_profile, sizeof(active_profile));
        if (err <= 0) {
            LOG_ERR("Failed to handle active profile from settings (err %d)", err);
            return err;
        }
    }
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    else if (settings_name_steq(name, "peripheral_addresses", &next) && next) {
        if (len != sizeof(bt_addr_le_t)) {
            return -EINVAL;
        }

        int i = atoi(next);
        if (i < 0 || i >= ZMK_SPLIT_BLE_PERIPHERAL_COUNT) {
            LOG_ERR("Failed to store peripheral address in memory");
        } else {
            int err = read_cb(cb_arg, &peripheral_addrs[i], sizeof(bt_addr_le_t));
            if (err <= 0) {
                LOG_ERR("Failed to handle peripheral address from settings (err %d)", err);
                return err;
            }
        }
    }
#endif

    return 0;
};

static int zmk_ble_complete_startup(void);

static struct settings_handler profiles_handler = {
    .name = "ble", .h_set = ble_profiles_handle_set, .h_commit = zmk_ble_complete_startup};

#endif /* IS_ENABLED(CONFIG_SETTINGS) */

static bool is_conn_active_profile(const struct bt_conn *conn) {
    return bt_addr_le_cmp(bt_conn_get_dst(conn), &profiles[active_profile].peer) == 0;
}

/* Connection parameters by profile role.
 *
 * Every bonded profile stays connected (BT_MAX_CONN > 1) but only the active
 * one carries HID traffic. An inactive link that keeps the same 7.5 ms
 * interval / latency 30 as the active one still wakes the radio every
 * (latency + 1) intervals ~= 232 ms, and the Zephyr link layer gives the
 * active link no precedence when the two events overlap: the ticker's age
 * comparison discounts planned latency skips, and a running peripheral event
 * is aborted by any other event's prepare unless it is near supervision
 * timeout (lll_conn_peripheral_is_abort_cb). Each such collision costs the
 * active link one connection event; on the dongle path that surfaces as a
 * touchpad frame arriving 7.5 ms late paired with the next one, which
 * libinput turns into a cursor jump (aster plans/touchpad-passthrough-fixes.md,
 * Phase G, cause 2 -- measured 2026-09-06: disconnecting the idle laptop link
 * took the dongle from ~9 % paired frames to the direct-BLE baseline).
 *
 * So inactive links are renegotiated to a long interval with latency: one
 * radio wake every 100 ms x (19 + 1) = 2 s instead of every 232 ms, i.e.
 * roughly a tenth of the collisions, and a latency link breaks latency on its
 * own as soon as it has data, so switching back to that profile costs one
 * 100 ms interval for the request plus ~6 intervals to the update instant.
 * Supervision timeout must exceed 2 x (1 + latency) x interval = 4 s, and
 * Linux additionally requires latency <= timeout x 4 / interval - 1 = 29
 * (hci_check_conn_params). The active link keeps the BT_PERIPHERAL_PREF_*
 * values (see daisy.conf for why those are 7.5-10 ms). */
static const struct bt_le_conn_param conn_param_active = BT_LE_CONN_PARAM_INIT(
    CONFIG_BT_PERIPHERAL_PREF_MIN_INT, CONFIG_BT_PERIPHERAL_PREF_MAX_INT,
    CONFIG_BT_PERIPHERAL_PREF_LATENCY, CONFIG_BT_PERIPHERAL_PREF_TIMEOUT);
static const struct bt_le_conn_param conn_param_inactive =
    BT_LE_CONN_PARAM_INIT(80, 80, 19, 600); /* 100 ms, latency 19, 6 s */

/* Which parameter set this link should be running, or NULL if we should keep
 * out of it. Fills *info, which the caller needs anyway.
 *
 * A peer that matches no profile is a pairing in progress: the open profile's
 * address is only recorded once bonding completes (set_profile_address ->
 * profile-changed event -> back here). Leave the host's default preferences in
 * charge until then, or a passkey-entry pairing that outlasts the 5 s timer
 * would idle its own link. */
static const struct bt_le_conn_param *link_params_for(struct bt_conn *conn,
                                                      struct bt_conn_info *info) {
    if (bt_conn_get_info(conn, info) != 0 || info->type != BT_CONN_TYPE_LE ||
        info->role != BT_CONN_ROLE_PERIPHERAL || info->state != BT_CONN_STATE_CONNECTED) {
        return NULL;
    }

    if (zmk_ble_profile_index(bt_conn_get_dst(conn)) < 0) {
        return NULL;
    }

    return is_conn_active_profile(conn) ? &conn_param_active : &conn_param_inactive;
}

/* Microseconds is the canonical unit here: bt_conn_info carries the interval
 * that way (bt_conn_le_info.interval is deprecated, and compiled out entirely
 * under CONFIG_BT_SHORTER_CONNECTION_INTERVALS), while bt_le_conn_param and the
 * le_param_updated() callback both use 1.25 ms units. */
#define LINK_PARAM_INTERVAL_US(units) ((uint32_t)(units) * 1250U)

static bool link_params_match(const struct bt_le_conn_param *param, uint32_t interval_us,
                              uint16_t latency, uint16_t timeout) {
    return interval_us >= LINK_PARAM_INTERVAL_US(param->interval_min) &&
           interval_us <= LINK_PARAM_INTERVAL_US(param->interval_max) &&
           latency == param->latency && timeout == param->timeout;
}

/* Whether a granted parameter set is good enough for this link's role, as
 * opposed to being exactly what we asked for.
 *
 * Deliberately weaker than link_params_match(): a central that grants the
 * request in spirit -- a longer interval on an idle link, more latency, a
 * shorter supervision timeout -- is not worth fighting. Our own dongle does
 * exactly that (le_param_req() in hid-remapper-private pins interval_max to
 * interval_min and clamps the timeout to CONN_TIMEOUT_AWAKE while its host is
 * awake), so an exact test here would read every clamped grant as a refusal
 * and disarm the re-assert below for the rest of the connection.
 *
 * Two things have to hold. The wake period, interval x (1 + latency), must be
 * no shorter than the role asks for -- that is the only property of a granted
 * set that can pre-empt another link's connection events, and the whole point
 * of the inactive set. And on the active link the interval is also the report
 * cadence, so a longer one is a real loss there even when latency makes the
 * wake period look fine; an inactive link is idle and may have any interval. */
static bool link_params_ok(const struct bt_le_conn_param *param, uint32_t interval_us,
                           uint16_t latency) {
    if (param == &conn_param_active &&
        interval_us > LINK_PARAM_INTERVAL_US(param->interval_max)) {
        return false;
    }

    uint64_t granted = (uint64_t)interval_us * (1U + latency);
    uint64_t wanted = (uint64_t)LINK_PARAM_INTERVAL_US(param->interval_min) * (1U + param->latency);

    return granted >= wanted;
}

static void request_link_params(struct bt_conn *conn, const struct bt_le_conn_param *param) {
    char addr[BT_ADDR_LE_STR_LEN];
    const char *role = (param == &conn_param_active) ? "active" : "inactive";

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    /* Before the host's 5 s post-connect update timer has fired this only
     * stores the request and the timer sends it in place of the Kconfig
     * preferences; afterwards it goes out right away. */
    int err = bt_conn_le_param_update(conn, param);
    if (err) {
        LOG_WRN("%s: %s-profile param update (%u-%u/%u/%u) failed: %d", addr, role,
                param->interval_min, param->interval_max, param->latency, param->timeout, err);
    } else {
        LOG_DBG("%s: requesting %s-profile params %u-%u/%u/%u", addr, role, param->interval_min,
                param->interval_max, param->latency, param->timeout);
    }
}

static void apply_link_params_cb(struct bt_conn *conn, void *data) {
    struct bt_conn_info info;
    const struct bt_le_conn_param *param = link_params_for(conn, &info);

    if (param == NULL) {
        return;
    }

    /* Already there (e.g. re-applied on another profile's connect): don't
     * start a procedure for nothing. */
    if (link_params_match(param, info.le.interval_us, info.le.latency, info.le.timeout)) {
        return;
    }

    request_link_params(conn, param);
}

/* Re-evaluate every peripheral link's parameters against the active profile.
 * Called on connect and whenever the active profile changes. */
static void apply_link_params(void) {
    bt_conn_foreach(BT_CONN_TYPE_LE, apply_link_params_cb, NULL);
}

/* Connection parameters are not ours to keep: the central may start an update
 * at any time, and both of ours do -- BlueZ over the life of a HID link, and
 * our own host CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT ms after connect. Whatever
 * it picks replaces what apply_link_params() asked for, and since that only
 * runs on connect and on a profile switch, an idle link could sit at the
 * active profile's 7.5 ms / latency 30 for the rest of its life -- waking every
 * 31 intervals and stealing one connection event from the active link every
 * 232.5 ms, which is a burst pair on the host and a discarded frame in
 * libinput. Measured that way: aster issues/inactive-link-params-not-reasserted.md.
 *
 * So an update that leaves a link off its role's parameters is answered by
 * asking again, with one credit per link: spent on the re-assert, refilled
 * whenever the link is *seen* at the parameters its role wants. A central that
 * simply refuses gets asked once and then left alone (we never see the match
 * that would refill the credit), while a central that keeps overriding a link
 * it had granted gets answered every time -- no unbounded ping-pong either
 * way.
 *
 * The work item is not optional: le_param_updated() runs on the host RX thread
 * (hci_core.c, LE Connection Update Complete) and bt_conn_le_param_update()
 * blocks on an HCI command once the peripheral param-update timer has expired
 * (send_conn_le_param_update -> bt_conn_le_conn_update ->
 * bt_hci_cmd_send_sync), which is exactly the case on a link that has been up
 * a while. Re-asserting inline would block HCI RX on a command completion that
 * HCI RX is itself responsible for delivering. The system workqueue is where
 * the connect and profile-switch paths already call apply_link_params() from. */
BUILD_ASSERT(CONFIG_BT_MAX_CONN <= 32, "link_reassert_pending is a 32-bit conn-index mask");
static atomic_t link_reassert_pending;
static bool link_reassert_spent[CONFIG_BT_MAX_CONN];

static void link_reassert_cb(struct bt_conn *conn, void *data) {
    uint8_t index = bt_conn_index(conn);

    /* Claim the request, so a link whose bit was set exactly once is
     * re-asserted exactly once even if the work runs late and coalesced. */
    if ((atomic_and(&link_reassert_pending, ~BIT(index)) & BIT(index)) == 0) {
        return;
    }

    /* Re-reads the link's current parameters, so an update that arrived in the
     * meantime (ours included) simply makes this a no-op. */
    apply_link_params_cb(conn, NULL);
}

static void link_reassert_work_cb(struct k_work *work) {
    bt_conn_foreach(BT_CONN_TYPE_LE, link_reassert_cb, NULL);
}

K_WORK_DEFINE(link_reassert_work, link_reassert_work_cb);

static void connected(struct bt_conn *conn, uint8_t err) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;
    LOG_DBG("Connected thread: %p", k_current_get());

    bt_conn_get_info(conn, &info);

    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        LOG_DBG("SKIPPING FOR ROLE %d", info.role);
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    advertising_status = ZMK_ADV_NONE;

    if (err) {
        LOG_WRN("Failed to connect to %s (%u)", addr, err);
        update_advertising();
        return;
    }

    LOG_DBG("Connected %s", addr);

    /* A new link (or a reused conn slot) starts with its re-assert credit
     * intact and nothing pending from whoever held the slot before. */
    atomic_and(&link_reassert_pending, ~BIT(bt_conn_index(conn)));
    link_reassert_spent[bt_conn_index(conn)] = false;

    update_advertising();
    apply_link_params();

    if (is_conn_active_profile(conn)) {
        LOG_DBG("Active profile connected");
        k_work_submit(&raise_profile_changed_event_work);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Disconnected from %s (reason 0x%02x)", addr, reason);

    bt_conn_get_info(conn, &info);

    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        LOG_DBG("SKIPPING FOR ROLE %d", info.role);
        return;
    }

    // We need to do this in a work callback, otherwise the advertising update will still see the
    // connection for a profile as active, and not start advertising yet.
    adv_burst_arm();
    k_work_submit(&update_advertising_work);

    if (is_conn_active_profile(conn)) {
        LOG_DBG("Active profile disconnected");
        k_work_submit(&raise_profile_changed_event_work);
    }
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_DBG("Security changed: %s level %u", addr, level);
        return;
    }

    LOG_ERR("Security failed: %s level %u err %d", addr, level, err);

    bt_conn_get_info(conn, &info);
    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        return;
    }

    /* Tear the link down instead of sitting on an unencrypted one.
     *
     * The usual way to get here is a host reconnecting with a bond we no
     * longer have (we were unpaired on the keyboard side): it starts
     * encryption with its stale LTK, our controller answers the LTK request
     * negatively, and the resulting Encryption Change failure lands here as
     * BT_SECURITY_ERR_PIN_OR_KEY_MISSING. Nothing tore the connection down
     * afterwards, so the host kept a live-but-useless link and went on
     * showing the keyboard as connected. A HID peripheral can do nothing on
     * an unencrypted link (every characteristic needs encryption, so
     * bt_gatt_notify() just returns -EPERM), so dropping it costs nothing and
     * makes the failure unambiguous to the host.
     *
     * This does NOT delete the bond the host is holding -- BLE has no message
     * for that, the user has to remove the device on the host side.
     */
    int derr = bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
    if (derr) {
        LOG_WRN("Failed to disconnect %s after security failure (err %d)", addr, derr);
    }
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                             uint16_t timeout) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("%s: interval %d latency %d timeout %d", addr, interval, latency, timeout);

    const struct bt_le_conn_param *param = link_params_for(conn, &info);
    if (param == NULL) {
        return;
    }

    uint8_t index = bt_conn_index(conn);
    const char *role = (param == &conn_param_active) ? "active" : "inactive";

    if (link_params_ok(param, LINK_PARAM_INTERVAL_US(interval), latency)) {
        /* Good enough: nothing here can pre-empt the active link, so take it
         * and hand the link a fresh credit for the next override. */
        link_reassert_spent[index] = false;
        return;
    }

    if (link_reassert_spent[index]) {
        /* Asked once already and the central still wants a shorter wake than
         * this link's role does; leaving it alone rather than ping-ponging. On
         * an inactive link this is the 232.5 ms pre-emption above, so it is
         * worth seeing in a log rather than only in an arrival histogram. */
        LOG_WRN("%s: %s-profile link left waking every %u us (%u/%u/%u), wanted %u-%u/%u/%u", addr,
                role, (unsigned int)(LINK_PARAM_INTERVAL_US(interval) * (1U + latency)), interval,
                latency, timeout, param->interval_min, param->interval_max, param->latency,
                param->timeout);
        return;
    }

    LOG_INF("%s: %s-profile link updated to %u/%u/%u by the peer; re-asserting %u-%u/%u/%u", addr,
            role, interval, latency, timeout, param->interval_min, param->interval_max,
            param->latency, param->timeout);
    link_reassert_spent[index] = true;
    atomic_or(&link_reassert_pending, BIT(index));
    k_work_submit(&link_reassert_work);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed,
    .le_param_updated = le_param_updated,
};

/*
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Passkey for %s: %06u", addr, passkey);
}
*/

#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)

static void auth_passkey_entry(struct bt_conn *conn) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Passkey entry requested for %s", addr);
    ring_buf_reset(&passkey_entries);
    auth_passkey_entry_conn = bt_conn_ref(conn);
}

#endif

static void auth_cancel(struct bt_conn *conn) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)
    if (auth_passkey_entry_conn) {
        bt_conn_unref(auth_passkey_entry_conn);
        auth_passkey_entry_conn = NULL;
    }

    ring_buf_reset(&passkey_entries);
#endif

    LOG_DBG("Pairing cancelled: %s", addr);
}

/* Submit a passkey to a pending passkey-entry pairing directly, bypassing the
 * on-keyboard digit-by-digit entry. Used by the factory USB interface to relay
 * a host-displayed passkey over the wire (keeps MITM: the passkey never goes
 * over the air). Returns -ENOTSUP if passkey entry isn't built in, -ENOTCONN if
 * no pairing is currently awaiting a passkey. */
int zmk_ble_passkey_entry(uint32_t passkey) {
#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)
    if (!auth_passkey_entry_conn) {
        return -ENOTCONN;
    }
    int err = bt_conn_auth_passkey_entry(auth_passkey_entry_conn, passkey);
    bt_conn_unref(auth_passkey_entry_conn);
    auth_passkey_entry_conn = NULL;
    return err;
#else
    ARG_UNUSED(passkey);
    return -ENOTSUP;
#endif
}

static bool pairing_allowed_for_current_profile(struct bt_conn *conn) {
    return zmk_ble_active_profile_is_open() ||
           (IS_ENABLED(CONFIG_BT_SMP_ALLOW_UNAUTH_OVERWRITE) &&
            bt_addr_le_cmp(zmk_ble_active_profile_addr(), bt_conn_get_dst(conn)) == 0);
}

static enum bt_security_err auth_pairing_accept(struct bt_conn *conn,
                                                const struct bt_conn_pairing_feat *const feat) {
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);

    LOG_DBG("role %d, open? %s", info.role, zmk_ble_active_profile_is_open() ? "yes" : "no");
    if (info.role == BT_CONN_ROLE_PERIPHERAL && !pairing_allowed_for_current_profile(conn)) {
        LOG_WRN("Rejecting pairing request to taken profile %d", active_profile);
        return BT_SECURITY_ERR_PAIR_NOT_ALLOWED;
    }

    return BT_SECURITY_ERR_SUCCESS;
};

static void auth_pairing_complete(struct bt_conn *conn, bool bonded) {
    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];
    const bt_addr_le_t *dst = bt_conn_get_dst(conn);

    bt_addr_le_to_str(dst, addr, sizeof(addr));
    bt_conn_get_info(conn, &info);

    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        LOG_DBG("SKIPPING FOR ROLE %d", info.role);
        return;
    }

    if (!pairing_allowed_for_current_profile(conn)) {
        LOG_ERR("Pairing completed but current profile is not open: %s", addr);
        bt_unpair(BT_ID_DEFAULT, dst);
        return;
    }

    set_profile_address(active_profile, dst);
    update_advertising();
};

static struct bt_conn_auth_cb zmk_ble_auth_cb_display = {
    .pairing_accept = auth_pairing_accept,
// .passkey_display = auth_passkey_display,

#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)
    .passkey_entry = auth_passkey_entry,
#endif
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb zmk_ble_auth_info_cb_display = {
    .pairing_complete = auth_pairing_complete,
};

static void zmk_ble_ready(int err) {
    LOG_DBG("ready? %d", err);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    /* Boot with a bond is the first reconnect opportunity. */
    adv_burst_arm();
    update_advertising();
}

static int zmk_ble_complete_startup(void) {

#if IS_ENABLED(CONFIG_ZMK_BLE_CLEAR_BONDS_ON_START)
    LOG_WRN("Clearing all existing BLE bond information from the keyboard");

    bt_unpair(BT_ID_DEFAULT, NULL);

    for (int i = 0; i < 8; i++) {
        char setting_name[15];
        sprintf(setting_name, "ble/profiles/%d", i);

        int err = settings_delete(setting_name);
        if (err) {
            LOG_ERR("Failed to delete setting: %d", err);
        }
    }

    // Hardcoding a reasonable hardcoded value of peripheral addresses
    // to clear so we properly clear a split central as well.
    for (int i = 0; i < 8; i++) {
        char setting_name[32];
        sprintf(setting_name, "ble/peripheral_addresses/%d", i);

        int err = settings_delete(setting_name);
        if (err) {
            LOG_ERR("Failed to delete setting: %d", err);
        }
    }

#endif // IS_ENABLED(CONFIG_ZMK_BLE_CLEAR_BONDS_ON_START)

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&zmk_ble_auth_cb_display);
    bt_conn_auth_info_cb_register(&zmk_ble_auth_info_cb_display);

    zmk_ble_ready(0);

    return 0;
}

#if IS_ENABLED(CONFIG_SHELL)
/* `bt_diag` shell command: on-demand dump of the controller's link-layer
 * diagnostic counters (defined in this tree's zephyr fork -- lll_adv.c and
 * hal/nrf5/radio/radio.c). These count CONNECT_IND accept/reject decisions
 * and the nRF54L address-resolver (AAR) health that are otherwise invisible
 * outside the radio ISR; the same numbers are readable over USB via factory
 * HID (DAISY_FACTORY_CMD_BT_DIAG, `aster v1 --bt-diag`). Declared __weak so
 * the file links against an uninstrumented controller too. */
#include <zephyr/shell/shell.h>

/* Diagnostic: is the controller's resolving list live (resolution enabled with
 * at least one entry)? Defined by the controller (ull_filter.c); __weak here so
 * this file also links against a build without LL privacy. Only bonds that
 * carry a peer IRK ever reach the RL while CONFIG_BT_PRIVACY=n. Stealth
 * advertising itself no longer depends on the RL (unlike the directed
 * advertising it replaced), but CONNECT_IND-acceptance debugging still wants
 * this view. */
__weak bool ull_filter_lll_rl_enabled(void) { return false; }

__weak uint32_t zmk_diag_ci_seen;
__weak uint32_t zmk_diag_ci_accepted;
__weak uint32_t zmk_diag_ci_rl_not_allowed;
__weak uint32_t zmk_diag_ci_adva_bad;
__weak uint32_t zmk_diag_ci_tgta_bad;
__weak uint32_t zmk_diag_ci_rx_unresolved;
__weak uint32_t zmk_diag_ci_last_rx_rl_idx = 0xff;
__weak uint32_t zmk_diag_ci_last_lll_rl_idx = 0xff;
__weak uint32_t zmk_diag_ar_cfg;
__weak uint32_t zmk_diag_ar_no_bc;
__weak uint32_t zmk_diag_ar_end_timeout;
__weak uint32_t zmk_diag_ar_notresolved;
__weak uint32_t zmk_diag_ar_resolved;

/* Host-side bond flags feeding the controller resolving list (host/id.c
 * zmk_diag_rl_state, TEMP patch in this tree's zephyr fork). Only bonds that
 * carry a peer IRK ever reach the RL while CONFIG_BT_PRIVACY=n. Pairs with
 * ull_filter_lll_rl_enabled() above. */
__weak void zmk_diag_rl_state(const bt_addr_le_t *peer, uint8_t *entries, uint8_t *size,
                              bool *bonded, bool *has_irk, bool *id_added, bool *id_pending) {
    *entries = *size = 0xff;
    *bonded = *has_irk = *id_added = *id_pending = false;
}

static int cmd_bt_diag(const struct shell *sh, size_t argc, char **argv) {
    static const char *const adv_names[] = {"none", "stealth", "open"};
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_t *peer = zmk_ble_active_profile_addr();
    uint8_t rl_entries, rl_size;
    bool bonded, has_irk, id_added, id_pending;

    shell_print(sh, "adv: %s%s (permit_adv %d)",
                advertising_status <= ZMK_ADV_CONN ? adv_names[advertising_status] : "?",
                advertising_status == ZMK_ADV_STEALTH ? (stealth_adv_bursting ? " burst" : " slow")
                                                      : "",
                permit_adv);

    bt_addr_le_to_str(peer, addr_str, sizeof(addr_str));
    shell_print(sh, "profile %u peer: %s (connected %d)", active_profile, addr_str,
                zmk_ble_active_profile_is_connected());

    zmk_diag_rl_state(peer, &rl_entries, &rl_size, &bonded, &has_irk, &id_added, &id_pending);
    shell_print(sh, "resolving list: lll-enabled %d entries %u/%u (host privacy %d, ctlr %d)",
                ull_filter_lll_rl_enabled(), rl_entries, rl_size,
                IS_ENABLED(CONFIG_BT_PRIVACY) ? 1 : 0,
                IS_ENABLED(CONFIG_BT_CTLR_PRIVACY) ? 1 : 0);
    shell_print(sh, "  peer bond: stored %d peer-irk %d rl-added %d rl-pending %d", bonded, has_irk,
                id_added, id_pending);
    shell_print(sh, "connect_ind: seen %u accepted %u", zmk_diag_ci_seen, zmk_diag_ci_accepted);
    shell_print(sh, "  rejected: rl-not-allowed %u adva %u tgta %u (initiator-rpa-unresolved %u)",
                zmk_diag_ci_rl_not_allowed, zmk_diag_ci_adva_bad, zmk_diag_ci_tgta_bad,
                zmk_diag_ci_rx_unresolved);
    shell_print(sh, "  last rl_idx: rx %u adv-target %u", zmk_diag_ci_last_rx_rl_idx,
                zmk_diag_ci_last_lll_rl_idx);
    shell_print(sh, "aar: configured %u no-bitcount %u end-timeout %u notresolved %u resolved %u",
                zmk_diag_ar_cfg, zmk_diag_ar_no_bc, zmk_diag_ar_end_timeout,
                zmk_diag_ar_notresolved, zmk_diag_ar_resolved);
    return 0;
}

SHELL_CMD_REGISTER(bt_diag, NULL,
                   "BLE link-layer diagnostics (CONNECT_IND acceptance, AAR health)", cmd_bt_diag);
#endif /* IS_ENABLED(CONFIG_SHELL) */

static int zmk_ble_init(void) {
    int err = bt_enable(NULL);

    if (err < 0 && err != -EALREADY) {
        LOG_ERR("BLUETOOTH FAILED (%d)", err);
        return err;
    }

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_register(&profiles_handler);
    k_work_init_delayable(&ble_save_work, ble_save_profile_work);
#else
    zmk_ble_complete_startup();
#endif

    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)

static bool zmk_ble_numeric_usage_to_value(const zmk_key_t key, const zmk_key_t one,
                                           const zmk_key_t zero, uint8_t *value) {
    if (key < one || key > zero) {
        return false;
    }

    *value = (key == zero) ? 0 : (key - one + 1);
    return true;
}

static int zmk_ble_handle_key_user(struct zmk_keycode_state_changed *event) {
    zmk_key_t key = event->keycode;

    LOG_DBG("key %d", key);

    if (!auth_passkey_entry_conn) {
        LOG_DBG("No connection for passkey entry");
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (event->state) {
        LOG_DBG("Key press, ignoring");
        return ZMK_EV_EVENT_HANDLED;
    }

    if (key == HID_USAGE_KEY_KEYBOARD_ESCAPE) {
        bt_conn_auth_cancel(auth_passkey_entry_conn);
        return ZMK_EV_EVENT_HANDLED;
    }

    if (key == HID_USAGE_KEY_KEYBOARD_RETURN || key == HID_USAGE_KEY_KEYBOARD_RETURN_ENTER) {
        uint8_t digits[PASSKEY_DIGITS];
        uint32_t count = ring_buf_get(&passkey_entries, digits, PASSKEY_DIGITS);

        uint32_t passkey = 0;
        for (int i = 0; i < count; i++) {
            passkey = (passkey * 10) + digits[i];
        }

        LOG_DBG("Final passkey: %d", passkey);
        bt_conn_auth_passkey_entry(auth_passkey_entry_conn, passkey);
        bt_conn_unref(auth_passkey_entry_conn);
        auth_passkey_entry_conn = NULL;
        return ZMK_EV_EVENT_HANDLED;
    }

    uint8_t val;
    if (!(zmk_ble_numeric_usage_to_value(key, HID_USAGE_KEY_KEYBOARD_1_AND_EXCLAMATION,
                                         HID_USAGE_KEY_KEYBOARD_0_AND_RIGHT_PARENTHESIS, &val) ||
          zmk_ble_numeric_usage_to_value(key, HID_USAGE_KEY_KEYPAD_1_AND_END,
                                         HID_USAGE_KEY_KEYPAD_0_AND_INSERT, &val))) {
        LOG_DBG("Key not a number, ignoring");
        return ZMK_EV_EVENT_HANDLED;
    }

    if (ring_buf_space_get(&passkey_entries) <= 0) {
        uint8_t discard_val;
        ring_buf_get(&passkey_entries, &discard_val, 1);
    }
    ring_buf_put(&passkey_entries, &val, 1);
    LOG_DBG("value entered: %d, digits collected so far: %d", val,
            ring_buf_size_get(&passkey_entries));

    return ZMK_EV_EVENT_HANDLED;
}

static int zmk_ble_listener(const zmk_event_t *eh) {
    struct zmk_keycode_state_changed *kc_state;

    kc_state = as_zmk_keycode_state_changed(eh);

    if (kc_state != NULL) {
        return zmk_ble_handle_key_user(kc_state);
    }

    return 0;
}

ZMK_LISTENER(zmk_ble, zmk_ble_listener);
ZMK_SUBSCRIPTION(zmk_ble, zmk_keycode_state_changed);
#endif /* IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY) */

#if IS_ENABLED(CONFIG_ZMK_BLE_STEALTH_ADV_BURST)

/* Kick a fresh burst window when the user acts on a keyboard whose active
 * profile is bonded but disconnected -- the moment reconnect latency is
 * actually felt. Triggers on key presses (keycode events bubble through
 * hid_listener; position events can be truncated by keymap.c before reaching
 * late-linked subscribers) and on the idle->active edge, which also covers
 * pointer activity. Listener context does no BT work itself; the window is
 * armed and the restart deferred, and the cooldown keeps typing at an absent
 * host from bursting continuously. */
static int adv_burst_kick_listener(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *kc = as_zmk_keycode_state_changed(eh);
    const struct zmk_activity_state_changed *act = as_zmk_activity_state_changed(eh);

    if ((kc == NULL || !kc->state) && (act == NULL || act->state != ZMK_ACTIVITY_ACTIVE)) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (!permit_adv || zmk_ble_active_profile_is_open() || zmk_ble_active_profile_is_connected() ||
        stealth_adv_bursting ||
        k_uptime_get() - adv_burst_armed_at < CONFIG_ZMK_BLE_STEALTH_ADV_BURST_COOLDOWN_MS) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    adv_burst_arm();
    k_work_submit(&update_advertising_work);

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_ble_adv_burst_kick, adv_burst_kick_listener);
ZMK_SUBSCRIPTION(zmk_ble_adv_burst_kick, zmk_keycode_state_changed);
ZMK_SUBSCRIPTION(zmk_ble_adv_burst_kick, zmk_activity_state_changed);

#endif /* IS_ENABLED(CONFIG_ZMK_BLE_STEALTH_ADV_BURST) */

uint8_t zmk_ble_adv_phase_diag(void) {
    if (advertising_status != ZMK_ADV_STEALTH) {
        return 0;
    }
    return stealth_adv_bursting ? 1 : 2;
}

SYS_INIT(zmk_ble_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
