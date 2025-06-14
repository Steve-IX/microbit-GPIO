# microbit-GPIO Lab - Register-Level GPIO, ADC and PWM on the nRF52833

**microbit-GPIO Lab** is a pure-C exploration of the BBC micro\:bit’s I/O subsystem.
Every peripheral is driven directly by writing to nRF52 registers, with no high-level CODAL helpers and no Arduino-style delays. The code base walks through five progressively richer demonstrations:&#x20;

| Demo                       | What it showcases                               | Key peripherals             |
| -------------------------- | ----------------------------------------------- | --------------------------- |
| 1. Binary counter @ 5 Hz   | Bit-banging a row of the 5 × 5 LED matrix       | GPIO output, SysTick timing |
| 2. Button-driven up/down   | Debounced input on push-buttons A & B           | GPIO input with pull-ups    |
| 3. Live voltage probe      | 8-bit sampling on edge-pin P0, 0–3 V            | SAADC with simple scaling   |
| 4. Breathing RGB LED       | 1 kHz PWM fades plus colour sweep               | PWM peripheral, GPIO ALT    |
| 5. Touch-activated counter | Capacitive-touch logo used as a digital trigger | GPIOTE sense mechanism      |

---

## Why this repo

* **Register-first mindset** - learn where the nRF52833 hides direction bits, pull-config, sense levels, PWM duty and SAADC channels.
* **Clean abstractions** - each task exposes a tiny, reusable API (`displayBinary`, `sampleVoltage`, `driveRGB`, …) instead of one monolithic `main()`.
* **Hardware-accurate timing** - the 5 Hz counter is calibrated to ±5 % using HFCLK-derived delays instead of guesswork loops.

---

## Repository layout

```
.
├── CW1.cpp               # all five demos and helper routines
├── MainSubtaskRunner.cpp # optional test harness (not submitted)
├── photos/
│   ├── ST3.jpg           # analogue-probe breadboard
│   └── ST4.jpg           # RGB LED wiring
├── docs/
│   └── SCC.369_week2_GPIO_spec.pdf     # original brief (reference only)
└── README.md
```

---

## Hardware bill of materials

| Qty      | Component                     | Notes                  |
| -------- | ----------------------------- | ---------------------- |
| 1        | BBC micro\:bit v2 (nRF52833)  | USB power and flashing |
| 1        | Breakout board + breadboard   | For edge-pin wiring    |
| 1        | 10 kΩ potentiometer           | Voltage source on P0   |
| 1        | Common-anode RGB LED          | R→P1, G→P9, B→P8       |
| 1        | 220 Ω resistor (red)          | Current limit          |
| 2        | 100 Ω resistors (green, blue) | Current limit          |
| assorted | Dupont jump-wires             | Male-to-male           |

---

## Build and flash

```bash
# clone scaffold that already includes MicroBit.h and NRF SDK paths
yotta build                     # or mbed compile -t GCC_ARM
cp build/bbc-microbit-classic-gcc/source/CW1-combined.hex /media/MICROBIT
```

No `main()` is present in the final submission; the board boots to idle.
Use `MainSubtaskRunner.cpp` during development to select which demo runs.

---

## API overview

```c
/* --- Display helpers --- */
void displayBinary(uint8_t value);        // update 5 LEDs from LSB..MSB
void countUpBinary(uint8_t initialValue); // free-runs at 5 Hz

/* --- Button & touch demos --- */
void countWithButtonsBinary(uint8_t initialValue); // A = −1, B = +1
void countWithTouchesBinary(uint8_t initialValue); // logo touch = +1

/* --- Analogue path --- */
uint8_t sampleVoltage(void);              // returns 0–255 for 0–3 V
void displayVoltageBinary(void);          // live bar graph

/* --- RGB driver --- */
void driveRGB(void);                      // breathing + colour knob
```

Each public function configures the required peripheral only once by means of static guards, avoiding double initialisation.

---

## Implementation highlights

* **Pin-level debounce** - A/B buttons sample at 1 kHz with a 15 ms stable-state filter, yielding single counts on press but never on release.
* **SAADC 8-bit mode** - channel 0, reference = VDD, gain = 1/6, 10-bit raw values right-shifted by 2 to give a 0–255 mapping.
* **PWM breathing** - duty sweeps via a pre-computed `uint16_t sinLUT[128]`, period 1 kHz; a potentiometer read every 50 ms morphs hue.
* **Touch sensing** - logo input configured with `SENSE = LOW` and an internal pull-up; a GPIOTE event triggers the counter ISR for ultra-low-latency taps.
* **Code quality extras** - Doxygen-style comments, bit-mask macros instead of magic numbers, and inline `static` helpers for toggling only the necessary bits (`SET`, `CLR`, `OUTSET`, `OUTCLR`).

---

## Extending the lab

| Idea                         | Direction                                                                                                                  |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| Timer-driven display refresh | Replace blocking delays with a PPI-linked `TIMER1` interrupt for precise 5 Hz counting and flicker-free LED duty control.  |
| DMA waveform                 | Use PWM + EasyDMA to stream arbitrary RGB patterns without CPU intervention.                                               |
| Continuous ADC               | Switch SAADC to free-run mode at 1 kHz, buffer to RAM via EasyDMA, and compute moving averages for a smoother voltage bar. |
| Power profiling              | Compare current draw between busy-wait, WFE sleep, and full system-off wake scenarios using an energy probe.               |

---
