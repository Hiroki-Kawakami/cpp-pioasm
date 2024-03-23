# cpp-pioasm
Write RP2040 Programmable I/O assembly with C++

## How to import

This is a header-only library. 
Simply place the `pioasm.hpp` file into your project directory and `#include` it within your `.cpp` file!

## How to Use

1. Create a class that inherits from `pioasm::builder`.
2. Override the `void code()` member function and write your pioasm code inside it.
3. If necessary, add member functions such as `init`.
  - You can use `pio_sm_config pioasm::builder::get_default_config(uint offset)` to obtain a `pio_sm_config`.
4. When initializing the state machine, pass the value obtained from `const struct pio_program *pioasm::builder::program()` to `pio_add_program` to load the program.

## How to write pioasm code

### Directives

- `.define ( PUBLIC ) <symbol> <value>`
  - This is unnecessary. Use C++ macros or variables instead.
- `.program <name>`
  - This is unnecessary. Programs should be identified by their class names.
- `.origin <offset>`
  - Place `origin(<offset>);` at the top of the `code()` function.
- `.side_set <count> (opt) (pindirs)`
  - Place `side_set(<count>);` at the top of the `code()` function.
  - `opt` and `pindirs` can be specified as `side_set(<count>, opt, pindirs);`.
- `.wrap_target`
  - Insert `wrap_target();` within the `code()` function.
- `.wrap`
  - Insert `wrap();` within the `code()` function.
- `.lang_opt <lang> <name> <option>`
  - This is unnecessary. The assembler does not produce code in any high-level language.
- `.word <value>`
  - Place `word(<value>);` within the `code()` function.

### Labels

Within the `code()` function, include `label(<symbol>);` where `<symbol>` can be any `std::string` value.

### Pseudoinstructions

#### `NOP`

- functions
  - `NOP()`

### Instructions

#### `JMP`

- functions
  - `JMP(<target>)`
  - `JMP(<cond>, <target>)`
- `<cond>`
  - `!x`
  - `x--`
  - `!y`
  - `y--`
  - `x!=y`
  - `pin`
  - `!osre`
- `<target>`
  - Any `std::string` value.

#### `WAIT`

- functions
  - `WAIT(<polarity>, gpio, <gpio_num>)`
  - `WAIT(<polarity>, pin, <pin_num>)`
  - `WAIT(<polarity>, irq, <irq_num>)`
  - `WAIT(<polarity>, irq, <irq_num>, rel)`
- `<polarity>`
  - `0` or `1`
- `<gpio_num>` `<pin_num>` `<irq_num>`
  - Integer number

#### `IN`

- functions
  - `IN(<source>, <bit_count>)`
- `<source>`
  - `pins`
  - `x`
  - `y`
  - `null`
  - `isr`
  - `osr`
- `<bit_count>`
  - Integer number

#### `OUT`

- functions
  - `OUT(<destination>, <bit_count>)`
- `<destination>`
  - `pins`
  - `x`
  - `y`
  - `null`
  - `pindirs`
  - `pc`
  - `isr`
  - `osr`
- `<bit_count>`
  - Integer number

#### `PUSH`

- functions
  - `PUSH()`
  - `PUSH(block)`
  - `PUSH(noblock)`
  - `PUSH(iffull)`
  - `PUSH(iffull, block)`
  - `PUSH(iffull, noblock)`

#### `PULL`

- functions
  - `PULL()`
  - `PULL(block)`
  - `PULL(noblock)`
  - `PULL(ifempty)`
  - `PULL(ifempty, block)`
  - `PULL(ifempty, noblock)`

#### `MOV`

- functions
  - `MOV(<destination>, <sources>)`
  - `MOV(<destination>, <op><sources>)`
- `<destination>`
  - `pins`
  - `x`
  - `y`
  - `exec`
  - `pc`
  - `isr`
  - `osr`
- `<op>`
  - `!` `~` : Invert
  - `*` : Bit-reverse
    - C++ doesn't allow overloading the `::` operator, so I use `*` instead
- `<sources>`
  - `pins`
  - `x`
  - `y`
  - `null`
  - `status`
  - `isr`
  - `osr`

#### `IRQ`

- functions
  - `IRQ(<irq_num>)`
  - `IRQ(<irq_num>, rel)`
  - `IRQ(set, <irq_num>)`
  - `IRQ(set, <irq_num>, rel)`
  - `IRQ(nowait, <irq_num>)`
  - `IRQ(nowait, <irq_num>, rel)`
  - `IRQ(wait, <irq_num>)`
  - `IRQ(wait, <irq_num>, rel)`
  - `IRQ(clear, <irq_num>)`
  - `IRQ(clear, <irq_num>, rel)`
- `<irq_num>`
  - Integer number

#### `SET`

- functions
  - `SET(<destination>, <value>)`
- `<destination>`
  - `pins`
  - `x`
  - `y`
  - `pindirs`

### Side-set Data

Invoke `side(<data>)` on the return value of the Instruction function.  
example:
```c
PULL(noblock).side(0);
```

### Delay

Append a subscript operator, `[<delay>]`, to the return value of either the instruction function or `side(<data>)`.  
example:
```c
PULL()                .side(1)  [7] ;
JMP(x--, "bitloop")             [6] ;
```
