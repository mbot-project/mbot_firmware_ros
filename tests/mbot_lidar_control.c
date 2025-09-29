#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <comms/dual_cdc.h>

// SV0 is connected to GPIO 20
#define SV0_PIN 20

static void core1_usb_task(void) {
    while (true) {
        dual_cdc_task();
        sleep_us(100);
    }
}

int main()
{
    stdio_init_all();
    dual_cdc_init();
    multicore_launch_core1(core1_usb_task);


    // Initialize GPIO 20 as output for SV0
    gpio_init(SV0_PIN);
    gpio_set_dir(SV0_PIN, GPIO_OUT);

    // Set GPIO 20 HIGH 
    gpio_put(SV0_PIN, 1);
    
    printf("GPIO %d set to HIGH\n", SV0_PIN);
    
    // Stay in loop to keep pin HIGH
    while (true) {
        sleep_ms(1000);
    }
    
    return 0;
}