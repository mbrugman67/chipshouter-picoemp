#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "picoemp.h"
#include "serial.h"

#include "trigger_basic.pio.h"

static bool armed = false;
static bool timeout_active = false;  // no timeout by defualt!
static bool hvp_internal = true;
static absolute_time_t timeout_time;
static uint offset = 0xFFFFFFFF;

// defaults taken from original code
#define PULSE_DELAY_CYCLES_DEFAULT 0
#define PULSE_TIME_CYCLES_DEFAULT 625 // 5us in 8ns cycles
#define PULSE_TIME_US_DEFAULT 5 // 5us
#define PULSE_POWER_DEFAULT 0.0122
static uint32_t pulse_time;
static uint32_t pulse_delay_cycles;
static uint32_t pulse_time_cycles;
static union float_union {float f; uint32_t ui32;} pulse_power;

void arm() {
    gpio_put(PIN_LED_CHARGE_ON, true);
    armed = true;
}

void disarm() {
    gpio_put(PIN_LED_CHARGE_ON, false);
    armed = false;
    picoemp_disable_pwm();
}

uint32_t get_status() {
    uint32_t result = 0;
    if(armed) {
        result |= 0b1;
    }
    if(gpio_get(PIN_IN_CHARGED)) {
        result |= 0b10;
    }
    if(timeout_active) {
        result |= 0b100;
    }
    if(hvp_internal) {
        result |= 0b1000;
    }
    return result;
}

void update_timeout() {
    timeout_time = delayed_by_ms(get_absolute_time(), 60 * 1000);
}

void fast_trigger() {
    // Choose which PIO instance to use (there are two instances)
    PIO pio = pio0;

    // Our assembled program needs to be loaded into this PIO's instruction
    // memory. This SDK function will find a location (offset) in the
    // instruction memory where there is enough space for our program. We need
    // to remember this location!
    if (offset == 0xFFFFFFFF) { // Only load the program once
        offset = pio_add_program(pio, &trigger_basic_program);
    }
    
    // Find a free state machine on our chosen PIO (erroring if there are
    // none). Configure it to run our program, and start it, using the
    // helper function we included in our .pio file.
    uint sm = 0;
    trigger_basic_init(pio, sm, offset, PIN_IN_TRIGGER, PIN_OUT_HVPULSE);
    pio_sm_put_blocking(pio, sm, pulse_delay_cycles);
    pio_sm_put_blocking(pio, sm, pulse_time_cycles);

}

int main() {
    // Initialize USB-UART as STDIO
    //stdio_init_all();

    picoemp_init();

    // Init for reset pin (move somewhere else)
    gpio_init(PIN_OUT_READY);
    gpio_set_dir(PIN_OUT_READY, GPIO_OUT);
    gpio_put(PIN_OUT_READY, 0); // ready output off at start

    // Run serial-console on second core
    multicore_launch_core1(serial_console);

    pulse_time = PULSE_TIME_US_DEFAULT;
    pulse_power.f = PULSE_POWER_DEFAULT;
    pulse_delay_cycles = PULSE_DELAY_CYCLES_DEFAULT;
    pulse_time_cycles = PULSE_TIME_CYCLES_DEFAULT;
    bool ready_to_fire = gpio_get(PIN_IN_CHARGED);
    bool pio_ready = false;
    arm();

    while(1) {
        ready_to_fire = gpio_get(PIN_IN_CHARGED);

        gpio_put(PIN_LED_HV, ready_to_fire);
        gpio_put(PIN_OUT_READY, ready_to_fire);

        if(!ready_to_fire && armed) {
            pio_ready = false;
            picoemp_enable_pwm(pulse_power.f);
        }

        if (ready_to_fire && !pio_ready) {
            pio_ready = true;
            fast_trigger();
            while(!pio_interrupt_get(pio0, 0));
            pio_sm_set_enabled(pio0, 0, false);
        }
    }
    
    return 0;
}
