# GEL372 — FPGA Driving Test Simulator

A full Verilog implementation of a driving-test game for the Altera DE2 board
(Cyclone II EP2C35F672C6), developed against the GEL372 FPGA Build Guide.

---

## 1. File structure

| File                       | Role                                                              |
| -------------------------- | ----------------------------------------------------------------- |
| `top_level.v`              | Master wiring, reset synchroniser, collision-event edge detector  |
| `clock_divider.v`          | Generates 25 MHz VGA pixel clock and ~60 Hz game tick             |
| `input_handler.v`          | Debounces KEY push buttons and SW slide switches                  |
| `game_fsm.v`               | 6-state master FSM: IDLE / PLAYING / COLLIDED / AT_STOP / PARKING / DONE |
| `vehicle_controller.v`     | Car position (x,y), speed (0..7), 2-bit direction                 |
| `collision_detector.v`     | Track defined as union of 7 rectangles; exports STOP/PARK zones   |
| `vga_controller.v`         | 640×480 @ 60 Hz VGA: track, cones, car, nose indicator, zones     |
| `score_display.v`          | 7-seg: `HEX1 HEX0` = speed, `HEX5..HEX2` = MM:SS elapsed           |
| `lcd_controller.v`         | HD44780 16×2 LCD driver with per-state messages                   |
| `led_controller.v`         | Red = collision flash, Green = speed bar / pass / park            |
| `audio_controller.v`       | Full WM8731 I2C init + I2S tone generator for feedback            |
| `driving_test.qsf`         | Quartus project + all DE2 pin assignments                         |
| `driving_test.sdc`         | TimeQuest timing constraints (50 MHz master, false paths)         |
| `tb_*.v`                   | Testbenches for each major module                                 |

---

## 2. Controls

| Control     | Function                                          |
| ----------- | ------------------------------------------------- |
| `SW[17]`    | Start the driving test (IDLE → PLAYING)           |
| `KEY[0]`    | Steer left                                        |
| `KEY[1]`    | Steer right                                       |
| `KEY[2]`    | Accelerate (gas)                                  |
| `SW[0]`     | Brake                                             |
| `KEY[3]`    | Reset (active-low, synchronised internally)       |

### Objective
Drive from the START corridor (top-left), through the diagonal, **stop briefly at
the STOP zone** (red tint), follow the loop and slalom, and finally **come to a
complete halt inside the PARK zone** (yellow tint, bottom-left) to pass.

- 3 lives — each collision costs one.
- Red LEDs flash during the collision penalty, including cone hits.
- Green LEDs show a speed thermometer and light up fully on PASS.
- Speed on HEX is shown as a km/h-style value (00..70).
- Timer reaching 00 ends the run immediately in GAME OVER.
- Hitting a cone applies an extra -5 second time penalty.
- LCD displays a message for each game state.
- Audio tones: 220 Hz on collision, 880 Hz on PASS, 440 Hz while in STOP/PARK zones.

---

## 3. Building in Quartus II (13.0 sp1 recommended for Cyclone II)

1. Create a new empty project in Quartus named `driving_test`.
2. Copy all `.v` files, `driving_test.qsf` and `driving_test.sdc` into the
   project directory.
3. **Project → Add/Remove Files**: the `.qsf` already lists every `.v` source;
   confirm they appear.
4. **Assignments → Device**: should pre-populate from `.qsf` as
   `EP2C35F672C6`.
5. **Processing → Start Compilation** (Ctrl+L).
6. Open the programmer (**Tools → Programmer**), select `driving_test.sof`,
   and program the DE2 over USB-Blaster (JTAG mode).

The `.qsf` contains the full standard DE2 pin map (clock, 18 switches, 4 keys,
18 red LEDs, 9 green LEDs, all six 7-segs, 12-bit VGA DAC, 16×2 LCD, and the
WM8731 codec pins — `I2C_SDAT/SCLK`, `AUD_XCK/BCLK/DACLRCK/DACDAT`).

---

## 4. Track layout (circuit.jpeg → VGA coordinates)

All track rectangles are expressed in a 640×480 frame (origin top-left):

| Seg | Role                  | x-range     | y-range     |
| --- | --------------------- | ----------- | ----------- |
| A   | START corridor + STOP | 20..360     | 40..100     |
| B   | Diagonal link (vert)  | 240..310    | 80..230     |
| C   | Middle crossbar      | 120..520    | 210..270    |
| D   | Right vertical loop  | 460..540    | 120..430    |
| E   | Bottom right U-turn  | 200..540    | 370..430    |
| F   | Slalom lane          | 60..310     | 330..430    |
| G   | FINISH corridor      | 20..280     | 400..470    |

Zones (required for PASS):
- **STOP zone** — x∈[280,360], y∈[40,100], part of segment A.
- **PARK zone** — x∈[20,120], y∈[420,470], part of segment G.

Cones are drawn as small 6×6 orange squares at 10 positions that visually
match circuit.jpeg. They are also collision hazards, so driving into one
triggers the same penalty and red-light flash as leaving the road.

---

## 5. Development iterations and bugs fixed

### Iteration 1 — Guide review (conceptual)
- `clk_divider` had no reset and used a derived clock as a register clock.
- `LCD_DATA` was declared `inout` but only ever written — changed to `output`.
- Module-name mismatches (`vehicle_ctrl` vs `vehicle_controller`, etc.) would
  cause "module not found" errors.
- `game_fsm` needed an internal 1-second tick counter; the guide didn't supply one.
- `led_controller` referenced `pass` in its logic but didn't declare it as a port.

### Iteration 2 — Architecture decisions
- Adopted proper multi-file structure (one `.v` per module).
- Decided on full WM8731 I2C+I2S audio path (user's choice).
- Modelled the track as a union of 7 rectangles to match `circuit.jpeg` layout.
- Added debounce to inputs, clock-domain sync on reset, edge detection on
  steering so holding a key doesn't spin the car.

### Iteration 3 — Initial implementation
- All 11 modules written, .qsf and .sdc created, 8 testbenches.

### Iteration 4 — Syntax / logic bug hunt
- Removed dead `msg_byte`, `msg_is_cmd`, `char_out` signals in `lcd_controller`.
- Fixed unused `direction` input in `vga_controller` by adding a visible
  "nose" indicator on the front of the car.
- Simplified VGA colour mux — grass becomes the default, then priority mux
  overrides for cones/car/zones/roads.
- **Track gap fix**: segment F ended at y=400, segment G started at y=410,
  leaving a 10-pixel dead stripe the car would fall through. Extended F to
  y=430 and G to y=400 so they overlap.

### Iteration 5 — Deep review
- **Critical fix**: the STOP zone (x=280..360) was **not part of any road
  rectangle** — driving into it would immediately trigger a collision and
  drop a life. Extended segment A from x≤280 to x≤360 so the STOP zone is a
  proper extension of the START corridor.
- Verified start position (60,60) is safely inside segment A.
- Verified each road segment overlaps its neighbour (A↔B, B↔C, C↔D, D↔E, E↔F, F↔G).
- Clock-domain crossing between `clk_game` (car state) and `clk_25` (VGA) is
  acceptable as only pixel comparisons read the crossing signals — a torn
  read is invisible at 60 Hz refresh.

---

## 6. Notes on the WM8731 audio path

The codec is configured over I2C at boot:
- **R6** power-up, **R0/R1** line-in unmuted, **R2/R3** HP 0 dB, **R4** analogue
  path to DAC, **R5** digital path unmuted, **R7** I2S 16-bit, **R8** USB mode
  48 kHz, **R9** active.

After config completes, `AUD_BCLK` runs at 3.125 MHz, `AUD_DACLRCK` toggles every
32 BCLK cycles (~48.8 kHz sample rate), and `AUD_DACDAT` streams a 16-bit signed
mono square-wave tone chosen from the current game state.

---

## 7. Running the testbenches (ModelSim)

```
vlib work
vmap work work
vlog clock_divider.v tb_clock_divider.v
vsim tb_clock_divider
run -all
```

Repeat the pattern for each `tb_*.v`. The top-level testbench (`tb_top_level.v`)
exercises reset, SW[17] start, and VGA sync toggling; it does not wait the full
~1 s required for a real game transition because of real-time simulation cost.
