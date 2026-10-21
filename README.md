# SortingSys_EMBEDDED

Interrupt-driven firmware for an **automated conveyor-belt sorting system** running on an **ATmega2560** (bare-metal AVR C). Objects travel down a DC-motor-driven conveyor, are identified by a reflectivity sensor sampled through the ADC, and are dropped into one of four bins on a stepper-motor-driven rotating tray.

The system sorts four material types:

| Type     | ID | Typical ADC reading (min. reflectivity) |
|----------|----|-----------------------------------------|
| Aluminum | 1  | 0 – 255                                 |
| Steel    | 3  | 400 – 700                               |
| White    | 2  | 900 – 950                               |
| Black    | 0  | 956 – 1026                              |

> Developed as the final project (Milestone 5) for a microcontroller systems course by **Damon Hunka** and **Matt MacLeod**, 2020.

---

## Table of contents

- [System overview](#system-overview)
- [Hardware](#hardware)
- [Software architecture](#software-architecture)
- [Mechanisms in detail](#mechanisms-in-detail)
- [Repository layout](#repository-layout)
- [Building and flashing](#building-and-flashing)
- [Configuration](#configuration)
- [Operating the system](#operating-the-system)
- [Known limitations](#known-limitations)
- [Authors](#authors)
- [License](#license)

---

## System overview

```
           OI            OR + RL (ADC)                       EX
 object →  ●────────────────●───────────────────────────────●  ┐
          [enqueue]     [sample & classify]         [stop belt, │
                                                     rotate tray,│
 ══════════════════ conveyor (DC motor, PWM) ═══════ resume ]  ▼
                                                           ┌─────────┐
                                                           │ 4-bin   │  ← stepper motor
                                                           │ tray    │    homed by Hall
                                                           └─────────┘    effect sensor
```

1. **OI (optical, inductive-stage) sensor** sees a new object → a node is enqueued in a FIFO linked list.
2. **OR (optical, reflective-stage) sensor** goes high while the object is in front of the reflectivity sensor → the ADC continuously samples and keeps the **minimum** reading. When the object leaves, the minimum is used to classify the oldest unclassified node.
3. **EX (exit) sensor** sees the object at the end of the belt → the belt brakes, the tray rotates the shortest path to the matching bin, the node is freed, and the belt restarts.
4. Two push buttons provide **pause/resume** and **ramp-down** (graceful shutdown after the belt has been empty for a few seconds).
5. A 16×2 character **LCD** shows status and the final per-type counts.

---

## Hardware

| Component | Interface | MCU pins |
|-----------|-----------|----------|
| ATmega2560 (16 MHz crystal, prescaled to 8 MHz) | – | – |
| DC conveyor motor + driver (e.g. Pololu) | PWM (Timer0 OC0A) + direction/brake nibble | PB7 (PWM), PB0–PB3 |
| Stepper motor (tray), 200 steps/rev | 2-phase full-step drive | PA0–PA5 |
| OI optical sensor – object entry | INT0, falling edge, active low | PD0 |
| HE Hall-effect sensor – tray home | INT1, falling edge, active low | PD1 |
| OR optical sensor – object at reflectivity sensor | INT2, any edge, active high | PD2 |
| EX optical sensor – object at end of belt | INT3, falling edge, active low | PD3 |
| Reflectivity sensor (RL) | ADC1, 10-bit, AREF w/ ext. cap | PF1 |
| Pause button | INT4, falling edge, active low | PE4 |
| Ramp-down button | INT5, falling edge, active low | PE5 |
| HD44780 16×2 LCD (4-bit mode) | D4–D7, RW, RS, E | PC0–PC3, PC5, PC6, PC7 |

DC motor control words written to `PORTB` (lower nibble):

| Constant | Value | Effect |
|----------|-------|--------|
| `DC_MOTOR_CCW`   | `0b0111` | Drive belt forward |
| `DC_MOTOR_BRAKE` | `0b1111` | Brake to VCC |
| `DC_MOTOR_OFF`   | `0b0000` | Coast / off |

---

## Software architecture

The firmware follows a **"flags set by ISRs, work done in the main loop"** design:

- **Interrupt service routines** ([isr.c](isr.c)) do only the minimum time-critical work (read a pin, stop the belt, grab an ADC sample) and then *raise a state flag*.
- A **bit-flag finite state machine** ([fsm_api.h](fsm_api.h), [fsm.c](fsm.c)) is polled by the kernel loop in [main.c](main.c). Each pending flag is cleared and its handler is executed.
- A **FIFO linked list** ([linkedlist_util.c](linkedlist_util.c)) carries each object from detection through classification to sorting.

### State flags

States are bits in a single `volatile uint8_t g_CurrentState`, so several events can be pending at the same time:

| State | Bit | Raised by | Handler | Purpose |
|-------|-----|-----------|---------|---------|
| `INIT_STATE`         | `0x01` | `main()` at boot | `InitState()` | Home tray, clear queue/counters, start belt |
| `NEW_OBJ_STATE`      | `0x04` | INT0 (OI)  | `NewObjState()` | Allocate and enqueue a node |
| `SYSTEM_PAUSE_STATE` | `0x08` | INT4 (toggle) / Timer3 expiry | `SystemEndState()` | Stop belt, show stats, block until resumed |
| `SYSTEM_RAMP_STATE`  | `0x10` | INT5 | – (consumed by Timer3 ISR & `PositionTrayState`) | Ramp-down mode active |
| `POS_TRAY_HARD`      | `0x20` | INT3 (EX)  | `PositionTrayState()` | Rotate tray, free node, restart belt |
| `CLASS_STATE`        | `0x80` | INT2 (OR) after sampling | `ClassifyState()` | Classify object from min. ADC value |

Helper macros:

```c
#define TRIGGER_STATE(STATE)   (g_CurrentState |=  STATE)
#define PROCESS_STATE(STATE)   (g_CurrentState &= ~STATE)
#define TOGGLE_STATE(STATE)    (g_CurrentState ^=  STATE)
#define EVAL_STATE(reg, STATE) (reg & STATE)
```

The kernel loop takes a snapshot (`shadowState`) of `g_CurrentState` each iteration and services the flags in priority order: **pause → new object → classify → position tray → init**.

### Data flow per object

```
INT0 (OI) ─► NEW_OBJ_STATE ─► EnqueueNode()          node{type=?}  appended at tail
INT2 (OR) ─► ADC sampling ─► CLASS_STATE ─► DequeueCurrentNode()  node.type = class(minADC)
INT3 (EX) ─► POS_TRAY_HARD ─► DequeueHeadNode() ─► STMotorMove() ─► free(node)
```

The list keeps three pointers — `lHead`, `lCurr`, `lTail` — so classification (`lCurr`) and sorting (`lHead`) advance independently. This lets several objects be on the belt at once, each at a different stage.

---

## Mechanisms in detail

### Clocks and timers

| Timer | Mode | Prescaler | Compare | Resulting period | Use |
|-------|------|-----------|---------|------------------|-----|
| System clock | `CLKPR = 0x01` | /2 | – | 8 MHz | CPU |
| Timer0 | Fast PWM, OC0A non-inverting | /64 | `OCR0A = 0x70` | ≈ 488 Hz, ≈ 44 % duty | Conveyor speed |
| Timer1 | CTC | /8 | `OCR1A = 1000` | 1 ms tick | Busy-wait `mTim1_DelayMs()` (stepper timing, debouncing, LCD) |
| Timer3 | CTC + compare-A interrupt | /1024 | `OCR3A = 7812` | ≈ 1 s tick | Ramp-down watchdog `mTim3_SetWatchDogS()` |

### Object classification (ADC)

- ADC1 runs at 125 kHz (8 MHz / 64), right-adjusted, 10-bit.
- On the **rising edge** of OR (object arrives) the ADC interrupt is enabled and a conversion is started. Each `ADC_vect` completion stores `min(g_ADCSample, result)` and immediately starts the next conversion (free-running in software).
- On the **falling edge** of OR (object leaves) sampling stops. If at least `MIN_ADC_SAMPLES` (50) samples were taken, the minimum is latched into `g_ADCMinResult` and `CLASS_STATE` is raised; otherwise the reading is discarded as noise.
- `ClassifyState()` compares the minimum against the threshold windows in [project.h](project.h) and writes the 2-bit type into the node's bit-field.

Taking the **minimum** reflectance over the object's whole pass makes the reading robust to object orientation and sensor edge effects.

### Tray positioning (stepper)

The tray has four quadrants, one per material. Moves use a precomputed **shortest-path transition table** — positive = clockwise, negative = counter-clockwise, never more than 2 quadrants (180°):

```c
// from \ to    0   1   2   3
/* 0 */       { 0,  1,  2, -1 },
/* 1 */       {-1,  0,  1,  2 },
/* 2 */       { 2, -1,  0,  1 },
/* 3 */       { 1,  2, -1,  0 }
```

- The stepper is driven in **2-phase full-step** mode from a 4-entry step table; the current phase index is remembered between moves so the motor never skips or reverses unexpectedly.
- A quadrant is 50 steps (200 steps/rev).
- **Trapezoidal acceleration profile** (`STMotorDelayProfile()`): the step delay ramps down from 20 ms to 5 ms over the first 15 steps, cruises, then ramps back up at the end of the move. Separate step limits are used for 90° and 180° moves.
- **Homing**: at start-up, `mTray_Init()` steps the tray at constant speed until the Hall-effect sensor (INT1) sets `g_HomingFlag`, then disables INT1. The homed position is quadrant 0 (Black).

### Pause / resume

INT4 debounces for 50 ms, brakes the belt and **toggles** `SYSTEM_PAUSE_STATE`. `SystemEndState()` brakes for 500 ms, turns the motor off, prints the number of objects still on the belt and the per-type sorted counts, then spins until the flag is toggled off by a second press. On resume the LCD is reset and the belt restarts.

### Ramp-down (graceful shutdown)

Pressing the ramp button (INT5) raises `SYSTEM_RAMP_STATE` and arms a **6 s software watchdog** on Timer3. Every time an object is sorted while ramping, the watchdog is reset. When 6 s pass with no sorted object, the Timer3 ISR clears the ramp flag, raises `SYSTEM_PAUSE_STATE` (showing the final statistics) and disables its own interrupt.

### Fault handling

`BADISR_vect` catches any unexpected interrupt, drives `PORTC` high and halts — making stray interrupts immediately visible during development.

### Compile-time checks

`COMPILE_VERIFY()` uses a negative-size array typedef to assert at compile time that a list node is exactly 3 bytes (1-byte bit-field payload + 2-byte pointer), keeping heap usage predictable on the 8 KB SRAM part.

---

## Repository layout

| File | Description |
|------|-------------|
| [main.c](main.c) | System initialization and the kernel (FSM polling) loop |
| [fsm.c](fsm.c) / [fsm_api.h](fsm_api.h) | State definitions, state macros and state handlers |
| [isr.c](isr.c) | All interrupt service routines (sensors, buttons, ADC, Timer3, bad ISR) |
| [gpio.c](gpio.c) | Port directions, external-interrupt configuration, ADC init |
| [timer.c](timer.c) | Timer0 PWM, Timer1 ms delay, Timer3 seconds watchdog, µs delay |
| [motor_util.c](motor_util.c) | Stepper step tables, acceleration profile, `STMotorMove()`, tray homing |
| [linkedlist_util.c](linkedlist_util.c) / [linkedlist_api.h](linkedlist_api.h) | FIFO queue of objects on the belt |
| [LCD_util.c](LCD_util.c) / [LCD_api.h](LCD_api.h) | HD44780 4-bit LCD driver |
| [project.h](project.h) | Configuration switches, pin map, thresholds, LCD layout, shared globals |
| `Final_Proj.cproj`, `Final_Proj.componentinfo.xml` | Atmel/Microchip Studio project files |

---

## Building and flashing

### Requirements

- **Microchip Studio** (formerly Atmel Studio 7) with the AVR 8-bit GCC toolchain (`com.Atmel.AVRGCC8.C`), **or** a standalone `avr-gcc` + `avr-libc` + `avrdude` install.
- An ATmega2560 board (e.g. Arduino Mega 2560 or course development board) and a programmer (e.g. Atmel-ICE, AVRISP mkII, or the Arduino bootloader).

### Microchip Studio

1. Open `Final_Proj.cproj`.
2. Select the **Release** (or **Debug**) configuration.
3. *Build → Build Solution* (`F7`).
4. *Tools → Device Programming*, select the ATmega2560 and your tool, and program the generated `.hex`.

### Command line (avr-gcc)

```bash
avr-gcc -mmcu=atmega2560 -DF_CPU=8000000UL -Os -std=gnu99 -o sortingsys.elf *.c
```

```bash
avr-objcopy -O ihex -R .eeprom sortingsys.elf sortingsys.hex
```

```bash
avrdude -p m2560 -c wiring -P COM3 -b 115200 -D -U flash:w:sortingsys.hex:i
```

Adjust the programmer (`-c`) and port (`-P`) for your setup.

---

## Configuration

Compile-time switches in [project.h](project.h):

| Switch | Default | Effect |
|--------|---------|--------|
| `ENABLE_SMALL_STEPPER` | `0` | `1` = small 2048-step geared stepper (bench testing); `0` = 200-step lab stepper |
| `ENABLE_MOTOR_PROFILE` | `0` | `0` = trapezoidal acceleration profile; `1` = constant step delay |
| `ENABLE_DEBUG_BUILD`   | `0` | `1` = show current state, live object count and raw ADC value on the LCD |

Tunable parameters:

| Define | Default | Meaning |
|--------|---------|---------|
| `DC_MOTOR_SPEED` | `0x70` | Belt PWM duty (0–255) |
| `DEBOUNCE_DELAY_MS` | `50` | Pause button debounce |
| `RAMP_DELAY_S` | `6` | Idle time before ramp-down completes |
| `MIN_ADC_SAMPLES` | `50` | Minimum samples for a valid classification |
| `*_TH_MIN` / `*_TH_MAX` | see table above | Classification windows — **calibrate for your sensor and lighting** |

---

## Operating the system

1. Power on. The LCD briefly shows the banner, the tray homes (`HOMED`) and the belt starts.
2. Place objects on the belt; they are classified and sorted automatically.
3. **Pause button** — stops the belt and shows statistics; press again to resume.
4. **Ramp button** — finishes the objects currently on the belt and stops after 6 s of inactivity, showing the final counts.

LCD layout while paused / finished:

```
L:02|PAUSE|S:???      L = objects still on belt
05S|03A|04B|02W       sorted count of Steel, Aluminum, Black, White
```

---

## Known limitations

- Timing delays (`mTim1_DelayMs`) are busy-waits; the stepper move and the pause handler block the main loop (interrupts remain active).
- The pause button debounce delay runs inside the INT4 ISR.
- Readings outside every threshold window fall back to `ALUM_TYPE`.
- Thresholds are hard-coded and specific to the original lab hardware.
- `LCD_util.c` defaults `F_CPU` to 16 MHz while the CPU is prescaled to 8 MHz; LCD timing therefore runs with extra margin.

---

## Authors

- **Damon Hunka** — [@DamonAknuh](https://github.com/DamonAknuh)
- **Matt MacLeod**

---

## License

This project is released under the **MIT License** — see [LICENSE](LICENSE) for the full text.
