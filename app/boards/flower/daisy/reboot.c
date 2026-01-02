#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>
#if IS_ENABLED(CONFIG_RETENTION_BOOT_MODE)
#include <zephyr/retention/bootmode.h>
#endif /* IS_ENABLED(CONFIG_RETENTION_BOOT_MODE) */

/* Handler for 'reset now' */
static int cmd_reset_now(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "Rebooting system...");

    /* Give the shell a moment to print before killing the system */
    k_sleep(K_MSEC(100));

    sys_reboot(SYS_REBOOT_WARM);
    return 0;
}

/* Handler for 'reset bootloader' */
static int cmd_reset_bootloader(const struct shell *sh, size_t argc, char **argv)
{
#if IS_ENABLED(CONFIG_RETENTION_BOOT_MODE)
    shell_print(sh, "Rebooting with retention mode...");
#else
    shell_print(sh, "Rebooting with 0x57...");
#endif

    /* Give the shell a moment to print before killing the system */
    k_sleep(K_MSEC(100));

// #if IS_ENABLED(CONFIG_RETENTION_BOOT_MODE)
//     int ret = bootmode_set(BOOT_MODE_TYPE_BOOTLOADER);
//     if (ret < 0) {
//         LOG_ERR("Failed to set the bootloader mode (%d)", ret);
//         return ZMK_BEHAVIOR_OPAQUE;
//     }
// 
//     sys_reboot(SYS_REBOOT_WARM);
// #else
//     // See
//     // https://github.com/adafruit/Adafruit_nRF52_Bootloader/blob/d6b28e66053eea467166f44875e3c7ec741cb471/src/main.c#L107
//     sys_reboot(0x57);
// #endif /* IS_ENABLED(CONFIG_RETENTION_BOOT_MODE) */

    return 0;
}

/* Define the subcommands: 'now' and 'bootloader' */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_reset,
    SHELL_CMD(now, NULL, "Reset the device immediately", cmd_reset_now),
    SHELL_CMD(bootloader, NULL, "Reset into UF2 Bootloader mode", cmd_reset_bootloader),
    SHELL_SUBCMD_SET_END
);

/* Register the root command 'reset' */
SHELL_CMD_REGISTER(reset, &sub_reset, "System reset commands", NULL);
