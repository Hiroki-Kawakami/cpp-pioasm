#include <Arduino.h>
#include "pioasm.hpp"

struct pwm_program_builder : pioasm::builder {
    void code() {
        side_set(1, opt)                ;

        PULL(noblock)          .side(0) ;
        MOV(x, osr)                     ;
        MOV(y, isr)                     ;
        label("countloop")              ;
        JMP(x!=y, "noset")              ;
        JMP("skip")            .side(1) ;
        label("noset")                  ;
        NOP()                           ;
        label("skip")                   ;
        JMP(y--, "countloop")           ;
    }

    void init(PIO pio, uint sm, uint offset, uint pin) {
        pio_gpio_init(pio, pin);
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
        pio_sm_config c = get_default_config(offset);
        sm_config_set_sideset_pins(&c, pin);
        pio_sm_init(pio, sm, offset, &c);
    }
};

// Write `period` to the input shift register
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

// Write `level` to TX FIFO. State machine will copy this into X.
void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
    pio_sm_put_blocking(pio, sm, level);
}

PIO pio = pio0;
int sm = 0;
int level = 0;

void setup() {
    Serial.begin(115200);

    pwm_program_builder pwm_program;
    uint offset = pio_add_program(pio, pwm_program.program());
    Serial.println("Loaded program at " + String(offset));

    pwm_program.init(pio, sm, offset, PICO_DEFAULT_LED_PIN);
    pio_pwm_set_period(pio, sm, (1u << 16) - 1);
}

void loop() {
    Serial.println("Level = " + String(level));
    pio_pwm_set_level(pio, sm, level * level);
    level = (level + 1) % 256;
    delay(10);
}
