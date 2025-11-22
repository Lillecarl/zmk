#include <zephyr/kernel.h>
#include <zephyr/input/input.h>

static void button_switch_callback(struct input_event *evt) {
    /* Ignore unrelated events */
    if (evt->type != INPUT_EV_KEY) {
        return;
    }

    if (evt->code == INPUT_BTN_MODE) {
        if (evt->value) {
            printk("Mode Switch set to LEFT!\n");
        } else {
            printk("Mode Switch set to RIGHT!\n");
        }
    }
    
    if (evt->code == INPUT_BTN_SELECT) {
         if (evt->value) {
            printk("Pairing Button Pressed\n");
         }
    }
}
INPUT_CALLBACK_DEFINE(NULL, button_switch_callback);
