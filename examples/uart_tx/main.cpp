#include <Arduino.h>
#include "pioasm.hpp"
#include "hardware/clocks.h"

// An 8n1 UART transmit program.
// OUT pin 0 and side-set pin 0 are both mapped to UART TX pin.
struct uart_tx_program_builder : pioasm::builder {
    void code() {
        side_set(1, opt)        ;

        PULL()    .side(1)  [7] ;
        SET(x, 7) .side(0)  [7] ;
        label("bitloop")        ;
        OUT(pins, 1)            ;
        JMP(x--, "bitloop") [6] ;
    }

    void init(PIO pio, uint sm, uint offset, uint pin_tx, uint baud) {
        // Tell PIO to initially drive output-high on the selected pin, then map PIO
        // onto that pin with the IO muxes.
        pio_sm_set_pins_with_mask(pio, sm, 1u << pin_tx, 1u << pin_tx);
        pio_sm_set_pindirs_with_mask(pio, sm, 1u << pin_tx, 1u << pin_tx);
        pio_gpio_init(pio, pin_tx);

        pio_sm_config c = get_default_config(offset);

        // OUT shifts to right, no autopull
        sm_config_set_out_shift(&c, true, false, 32);

        // We are mapping both OUT and side-set to the same pin, because sometimes
        // we need to assert user data onto the pin (with OUT) and sometimes
        // assert constant values (start/stop bit)
        sm_config_set_out_pins(&c, pin_tx, 1);
        sm_config_set_sideset_pins(&c, pin_tx);

        // We only need TX, so get an 8-deep FIFO!
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

        // SM transmits 1 bit per 8 execution cycles.
        float div = (float)clock_get_hz(clk_sys) / (8 * baud);
        sm_config_set_clkdiv(&c, div);

        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);
    }

    void putc(PIO pio, uint sm, char c) {
        pio_sm_put_blocking(pio, sm, (uint32_t)c);
    }

    void puts(PIO pio, uint sm, const char *s) {
        while (*s)
            putc(pio, sm, *s++);
    }
};

PIO pio = pio0;
int sm = 0;
uart_tx_program_builder uart_tx_program;

void setup() {
    Serial.begin(115200);

    // We're going to use PIO to print "Hello, world!" on the same GPIO which we
    // normally attach UART0 to.
    const uint PIN_TX = 0;
    // This is the same as the default UART baud rate on Pico
    const uint SERIAL_BAUD = 115200;

    uint offset = pio_add_program(pio, uart_tx_program.program());
    uart_tx_program.init(pio, sm, offset, PIN_TX, SERIAL_BAUD);
}

void loop() {
    uart_tx_program.puts(pio, sm, "Hello, world! (from PIO!)\n");
    delay(1000);
}
