/*
 * Copyright (c) 2024 Hiroki Kawakami
 */

#pragma once
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

namespace pioasm {

struct builder {
private:
    struct context {
        std::map<std::string, uint8_t> jmp_table;
        std::vector<std::pair<uint8_t, std::string>> lazy_link;
    };
    context *ctx;
    std::vector<uint16_t> instructions;
    struct generate {
        uint wrap_target = 0, wrap = 0;
        uint8_t side_set = 0;
        bool side_set_opt = false, side_set_pindirs = false;
#if !PICO_NO_HARDWARE
        struct pio_program program = {
            .instructions = nullptr,
            .length = 0,
            .origin = -1,
        };
#endif
    };
    generate gen;

    template<uint8_t Bit, uint8_t Size>
    struct asm_param {
        uint8_t value;
        uint16_t apply(uint16_t from) {
            uint16_t mask = ~(((1 << Size) - 1) << Bit);
            return (from & mask) | (value << Bit);
        }
    };
    struct instruction_ref {
        builder &b;
        size_t index;

        instruction_ref side(uint8_t value) && {
            b.instructions[index] |= value << (13 - b.gen.side_set);
            if (b.gen.side_set_opt) b.instructions[index] |= 1 << 12;
            return std::move(*this);
        }
        instruction_ref operator[](uint8_t value) && {
            b.instructions[index] |= value << 8;
            return std::move(*this);
        }
    };
    instruction_ref add_instruction(uint8_t op, uint8_t raw = 0) {
        instructions.push_back(op << 13 | raw);
        return instruction_ref{*this, instructions.size() - 1};
    }
    template<typename T>
    instruction_ref add_instruction(uint8_t op, uint8_t raw, T param) {
        auto iref = add_instruction(op, raw);
        instructions[iref.index] = param.apply(instructions[iref.index]);
        return iref;
    }
    template<typename T, typename... Args>
    instruction_ref add_instruction(uint8_t op, uint8_t raw, T param, Args... remain) {
        auto iref = add_instruction(op, raw, remain...);
        instructions[iref.index] = param.apply(instructions[iref.index]);
        return iref;
    }
    void check_label(size_t index, std::string label) {
        if (ctx->jmp_table.find(label) != ctx->jmp_table.end()) {
            instructions[index] += ctx->jmp_table[label];
        } else {
            ctx->lazy_link.emplace_back(index, label);
        }
    }
    void build() {
        ctx = new context{};
        code();
        for (auto jmp : ctx->lazy_link) {
            if (ctx->jmp_table.find(jmp.second) != ctx->jmp_table.end()) {
                instructions[jmp.first] += ctx->jmp_table[jmp.second];
            }
        }
        if (gen.wrap == 0) gen.wrap = static_cast<uint>(instructions.size() - 1);
        delete ctx;
    }

protected:
    struct symbols {
        struct not_y {};
        struct bitwise_not_y {};
        struct y_dec {};
        struct bit_reverse_y{};
        struct y {
            not_y operator!() const { return not_y{}; }
            bitwise_not_y operator~() const { return bitwise_not_y{}; }
            y_dec operator--(int) const { return y_dec{}; }
            bit_reverse_y operator*() const { return bit_reverse_y{}; }
        };
        struct not_x {};
        struct bitwise_not_x {};
        struct x_dec {};
        struct bit_reverse_x{};
        struct x_not_y {};
        struct x {
            not_x operator!() const { return not_x{}; }
            bitwise_not_x operator~() const { return bitwise_not_x{}; }
            x_dec operator--(int) const { return x_dec{}; }
            bit_reverse_x operator*() const { return bit_reverse_x{}; }
            x_not_y operator!=(const y&) const { return x_not_y{}; }
        };
        struct pin {};
        struct not_pins {};
        struct bitwise_not_pins {};
        struct bit_reverse_pins{};
        struct pins {
            not_pins operator!() const { return not_pins{}; }
            bitwise_not_pins operator~() const { return bitwise_not_pins{}; }
            bit_reverse_pins operator*() const { return bit_reverse_pins{}; }
        };
        struct pindirs {};
        struct pc {};
        struct not_isr {};
        struct bitwise_not_isr {};
        struct bit_reverse_isr {};
        struct isr {
            not_isr operator!() const { return not_isr{}; }
            bitwise_not_isr operator~() const { return bitwise_not_isr{}; }
            bit_reverse_isr operator*() const { return bit_reverse_isr{}; }
        };
        struct not_osr {};
        struct bitwise_not_osr {};
        struct bit_reverse_osr {};
        struct osr {
            not_osr operator!() const { return not_osr{}; }
            bitwise_not_osr operator~() const { return bitwise_not_osr{}; }
            bit_reverse_osr operator*() const { return bit_reverse_osr{}; }
        };
        struct not_osre {};
        struct osre {
            not_osre operator!() const { return not_osre{}; }
        };
        struct gpio {};
        struct irq {};
        struct rel {};
        struct exec {};
        struct not_null {};
        struct bitwise_not_null {};
        struct bit_reverse_null {};
        struct null {
            not_null operator!() const { return not_null{}; }
            bitwise_not_null operator~() const { return bitwise_not_null{}; }
            bit_reverse_null operator*() const { return bit_reverse_null{}; }
        };
        struct not_status {};
        struct bitwise_not_status {};
        struct bit_reverse_status {};
        struct status {
            not_status operator!() const { return not_status{}; }
            bitwise_not_status operator~() const { return bitwise_not_status{}; }
            bit_reverse_status operator*() const { return bit_reverse_status{}; }
        };
        struct iffull {};
        struct ifempty {};
        struct block {};
        struct noblock {};
        struct set {};
        struct wait {};
        struct nowait {};
        struct clear {};
        struct opt {};
    };
    static constexpr symbols::x x{};
    static constexpr symbols::y y{};
    static constexpr symbols::pin pin{};
    static constexpr symbols::pins pins{};
    static constexpr symbols::pindirs pindirs{};
    static constexpr symbols::pc pc{};
    static constexpr symbols::isr isr{};
    static constexpr symbols::osr osr{};
    static constexpr symbols::osre osre{};
    static constexpr symbols::gpio gpio{};
    static constexpr symbols::irq irq{};
    static constexpr symbols::rel rel{};
    static constexpr symbols::exec exec{};
    static constexpr symbols::null null{};
    static constexpr symbols::status status{};
    static constexpr symbols::iffull iffull{};
    static constexpr symbols::ifempty ifempty{};
    static constexpr symbols::block block{};
    static constexpr symbols::noblock noblock{};
    static constexpr symbols::opt opt{};

    void origin(uint16_t value) {
        gen.program.origin = value;
    }
    void side_set(uint8_t size) {
        gen.side_set = size;
    }
    void side_set(uint8_t size, symbols::opt) {
        gen.side_set = size + 1;
        gen.side_set_opt = true;
    }
    void side_set(uint8_t size, symbols::pindirs) {
        gen.side_set = size;
        gen.side_set_pindirs = true;
    }
    void side_set(uint8_t size, symbols::opt, symbols::pindirs) {
        gen.side_set = size + 1;
        gen.side_set_opt = true;
        gen.side_set_pindirs = true;
    }
    void wrap_target() {
        gen.wrap_target = static_cast<uint>(instructions.size());
    }
    void wrap() {
        gen.wrap = static_cast<uint>(instructions.size() - 1);
    }
    void word(uint16_t value) {
        instructions.push_back(value);
    }
    void label(std::string label) {
        ctx->jmp_table.emplace(label, instructions.size());
    }

    struct jmp_condition : asm_param<5, 3> {
        jmp_condition()                  : asm_param{0b000} {}; // always
        jmp_condition(symbols::not_x)    : asm_param{0b001} {}; // scratch X zero
        jmp_condition(symbols::x_dec)    : asm_param{0b010} {}; // scratch X non-zero, prior to decrement
        jmp_condition(symbols::not_y)    : asm_param{0b011} {}; // scratch Y zero
        jmp_condition(symbols::y_dec)    : asm_param{0b100} {}; // scratch Y non-zero, prior to decrement
        jmp_condition(symbols::x_not_y)  : asm_param{0b101} {}; // scratch X not equal scratch Y
        jmp_condition(symbols::pin)      : asm_param{0b110} {}; // branch on input pin
        jmp_condition(symbols::not_osre) : asm_param{0b111} {}; // output shift register not empty
    };
    instruction_ref JMP(jmp_condition condition, uint8_t address) {
        return add_instruction(0b000, address, condition);
    }
    inline instruction_ref JMP(uint8_t address) {
        return JMP({}, address);
    }
    instruction_ref JMP(jmp_condition condition, std::string label) {
        auto iref = add_instruction(0b000, 0, condition);
        check_label(iref.index, label);
        return iref;
    }
    inline instruction_ref JMP(std::string label) {
        return JMP({}, label);
    }

    struct wait_source : asm_param<5, 2> {
        wait_source(symbols::gpio) : asm_param{0b00} {}; // System GPIO input selected by Index. This is an absolute GPIO index, and is not affected by the state machine’s input IO mapping.
        wait_source(symbols::pin)  : asm_param{0b01} {}; // Input pin selected by Index. This state machine’s input IO mapping is applied first, and then Index selects which of the mapped bits to wait on. In other words, the pin is selected by adding Index to the PINCTRL_IN_BASE configuration, modulo 32.
        wait_source(symbols::irq)  : asm_param{0b10} {}; // PIO IRQ flag selected by Index
    };
    instruction_ref WAIT(uint8_t polarity, wait_source source, uint8_t index) {
        uint8_t raw = (polarity ? (1 << 7) : 0) + index;
        return add_instruction(0b001, raw, source);
    }
    inline instruction_ref WAIT(uint8_t polarity, symbols::irq, uint8_t index, symbols::rel) {
        return WAIT(polarity, irq, (1 << 4) | index);
    }

    struct in_source : asm_param<5, 3> {
        in_source(symbols::pins) : asm_param{0b000} {};
        in_source(symbols::x)    : asm_param{0b001} {}; // (scratch register X)
        in_source(symbols::y)    : asm_param{0b010} {}; // (scratch register Y)
        in_source(symbols::null) : asm_param{0b011} {}; // (all zeroes)
        in_source(symbols::isr)  : asm_param{0b110} {};
        in_source(symbols::osr)  : asm_param{0b111} {};
    };
    instruction_ref IN(in_source source, uint8_t bit_count) {
        return add_instruction(0b010, bit_count, source);
    }

    struct out_destination : asm_param<5, 3> {
        out_destination(symbols::pins)    : asm_param{0b000} {};
        out_destination(symbols::x)       : asm_param{0b001} {}; // (scratch register X)
        out_destination(symbols::y)       : asm_param{0b010} {}; // (scratch register Y)
        out_destination(symbols::null)    : asm_param{0b011} {}; // (discard data)
        out_destination(symbols::pindirs) : asm_param{0b100} {};
        out_destination(symbols::pc)      : asm_param{0b101} {};
        out_destination(symbols::isr)     : asm_param{0b110} {}; // (also sets ISR shift counter to Bit count)
        out_destination(symbols::exec)    : asm_param{0b111} {}; // (Execute OSR shift data as instruction)
    };
    instruction_ref OUT(out_destination destination, uint8_t bit_count) {
        return add_instruction(0b011, bit_count, destination);
    }

    struct fifo_block : asm_param<5, 1> {
        fifo_block(symbols::block)   : asm_param{1} {};
        fifo_block(symbols::noblock) : asm_param{0} {};
    };
    instruction_ref PUSH(fifo_block block = builder::block) {
        return add_instruction(0b100, 0b00 << 6, block);
    }
    instruction_ref PUSH(symbols::iffull, fifo_block block = builder::block) {
        return add_instruction(0b100, 0b01 << 6, block);
    }
    instruction_ref PULL(fifo_block block = builder::block) {
        return add_instruction(0b100, 0b10 << 6, block);
    }
    instruction_ref PULL(symbols::ifempty, fifo_block block = builder::block) {
        return add_instruction(0b100, 0b11 << 6, block);
    }

    struct mov_destination : asm_param<5, 3> {
        mov_destination(symbols::pins) : asm_param{0b000} {}
        mov_destination(symbols::x)    : asm_param{0b001} {}
        mov_destination(symbols::y)    : asm_param{0b010} {}
        mov_destination(symbols::exec) : asm_param{0b100} {}
        mov_destination(symbols::pc)   : asm_param{0b101} {}
        mov_destination(symbols::isr)  : asm_param{0b110} {}
        mov_destination(symbols::osr)  : asm_param{0b111} {}
    };
    struct mov_source : asm_param<0, 5> {
        mov_source(symbols::pins)               : asm_param{0b00000} {}
        mov_source(symbols::not_pins)           : asm_param{0b01000} {}
        mov_source(symbols::bitwise_not_pins)   : asm_param{0b01000} {}
        mov_source(symbols::bit_reverse_pins)   : asm_param{0b10000} {}
        mov_source(symbols::x)                  : asm_param{0b00001} {}
        mov_source(symbols::not_x)              : asm_param{0b01001} {}
        mov_source(symbols::bitwise_not_x)      : asm_param{0b01001} {}
        mov_source(symbols::bit_reverse_x)      : asm_param{0b10001} {}
        mov_source(symbols::y)                  : asm_param{0b00010} {}
        mov_source(symbols::not_y)              : asm_param{0b01010} {}
        mov_source(symbols::bitwise_not_y)      : asm_param{0b01010} {}
        mov_source(symbols::bit_reverse_y)      : asm_param{0b10010} {}
        mov_source(symbols::null)               : asm_param{0b00011} {}
        mov_source(symbols::not_null)           : asm_param{0b01011} {}
        mov_source(symbols::bitwise_not_null)   : asm_param{0b01011} {}
        mov_source(symbols::bit_reverse_null)   : asm_param{0b10011} {}
        mov_source(symbols::status)             : asm_param{0b00101} {}
        mov_source(symbols::not_status)         : asm_param{0b01101} {}
        mov_source(symbols::bitwise_not_status) : asm_param{0b01101} {}
        mov_source(symbols::bit_reverse_status) : asm_param{0b10101} {}
        mov_source(symbols::isr)                : asm_param{0b00110} {}
        mov_source(symbols::not_isr)            : asm_param{0b01110} {}
        mov_source(symbols::bitwise_not_isr)    : asm_param{0b01110} {}
        mov_source(symbols::bit_reverse_isr)    : asm_param{0b10110} {}
        mov_source(symbols::osr)                : asm_param{0b00111} {}
        mov_source(symbols::not_osr)            : asm_param{0b01111} {}
        mov_source(symbols::bitwise_not_osr)    : asm_param{0b01111} {}
        mov_source(symbols::bit_reverse_osr)    : asm_param{0b10111} {}
    };
    instruction_ref MOV(mov_destination destination, mov_source source) {
        return add_instruction(0b101, 0, destination, source);
    }
    inline instruction_ref NOP() {
        return MOV(y, y);
    }

    instruction_ref IRQ(uint8_t index) {
        return add_instruction(0b110, index);
    }
    inline instruction_ref IRQ(uint8_t index, symbols::rel) {
        return IRQ((1 << 4) | index);
    }
    template<typename... Args>
    inline instruction_ref IRQ(symbols::set, Args... args) {
        return IRQ(args...);
    }
    template<typename... Args>
    inline instruction_ref IRQ(symbols::nowait, Args... args) {
        return IRQ(args...);
    }
    inline instruction_ref IRQ(symbols::wait, uint8_t index) {
        return IRQ((1 << 5) | index);
    }
    inline instruction_ref IRQ(symbols::wait, uint8_t index, symbols::rel) {
        return IRQ((1 << 5) | index, rel);
    }
    inline instruction_ref IRQ(symbols::clear, uint8_t index) {
        return IRQ((1 << 6) | index);
    }
    inline instruction_ref IRQ(symbols::clear, uint8_t index, symbols::rel) {
        return IRQ((1 << 6) | index, rel);
    }

    struct set_destination : asm_param<5, 3> {
        set_destination(symbols::pins)    : asm_param{0b000} {}
        set_destination(symbols::x)       : asm_param{0b001} {} // (scratch register X) 5 LSBs are set to Data, all others cleared to 0
        set_destination(symbols::y)       : asm_param{0b010} {} // (scratch register Y) 5 LSBs are set to Data, all others cleared to 0
        set_destination(symbols::pindirs) : asm_param{0b100} {}
    };
    instruction_ref SET(set_destination destination, uint8_t data) {
        return add_instruction(0b111, data, destination);
    }

    virtual void code() = 0;

public:
    const uint16_t *data() {
        if (instructions.size() == 0) build();
        return instructions.data();
    }
#if !PICO_NO_HARDWARE
    const struct pio_program *program() {
        gen.program.instructions = data();
        gen.program.length = static_cast<uint>(instructions.size());
        return &gen.program;
    }
    inline pio_sm_config get_default_config(uint offset) {
        if (instructions.size() == 0) build();
        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, offset + gen.wrap_target, offset + gen.wrap);
        if (gen.side_set) sm_config_set_sideset(&c, gen.side_set, gen.side_set_opt, gen.side_set_pindirs);
        return c;
    }
#endif
};

}
