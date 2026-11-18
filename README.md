# Gloria

The Gloria firmware (`Gloria.ino`, in this folder - the factory builds the repo from it) and test code for the **Gloria shield**, which plugs into a **Valentino Build 3** base board. The hardware design (KiCad 9) is in `HardwareDesign/Gloria`.

| Folder | What |
|---|---|
| `TestRain485/` | RS485 test: finds a Modbus sensor (rain, soil) through the Clipsall RS485 board and prints its registers. `Baud#4800` for sensors that are not 9600. |
| `TestGloria/` | Hardware test sketch: `ValentinoTest` plus the shield (OLED, LEDs, relay, sensors, switches). See `TestGloria/README.md`. |
| `Gloria.ino` (this folder) | The Gloria firmware. Langley.ino (v19) with the fence parts removed and the Gloria shield added, plus the pump: a user-written rule for AUTO (see `GloriaPumpRule.h`), set on the web page (P) or with `SetPumpRule`. Build like Langley: board ESP32 Dev Module (esp32da), Flash Size 8MB, the sketch's own `partitions.csv`. Edit the web page in `web/`; `build_web.py` packs it into one gzipped `data/index.html.gz` (as Daffodil), `uploadData.sh` builds and flashes it. A factory release runs `build_web.py` itself. |

## What the shield has

- 9 x WS2812B LEDs on GPIO16
- 1.54" SSD1309 OLED (I2C)
- Relay (Q3 on GPIO17) in series with an emergency slide switch SW1: the relay only closes when SW1 is closed and GPIO17 is high
- Relay contacts on the 3-pin terminal J5: pin 1 NO, pin 2 COM, pin 3 NC (silkscreen "NO  COM  NC"). It is a dry contact meant for the float-switch input of a solar pump controller; it carries only the controller's sense current. Use the pair that stops the pump when the relay is off (GPIO17 low, SW1 in the emergency position, or the shield unpowered):
  - Controller stops the pump when the float terminals are **joined** (for example the Sunpump tested on 2026-10-26: open = runs, joined = stops): use **NC + COM**.
  - Controller stops the pump when the float terminals are **open**: use **NO + COM**.
  - With either pair, GPIO17 high = relay on = pump allowed to run. A broken cable to the controller reads as "open", so with an NC-type controller like the Sunpump it lets the pump run.
- Four sensor channels on terminals J3 and J6. Each has a level-shifted digital input and a jumper-selected analog input through ADS1115 U2
- Config DIP switch and emergency-switch sense through ADS1115 U1
- START button S2 and AUTO/MANUAL slide switch SW2
- TPS2051 load switch (U4) that turns the OLED and LED rail (`V50_SW`) on and off from GPIO19. The sensor side stays on `V50`

## Header pins

Shield **J1** mates with Valentino **J3** (left). Shield **J4** mates with Valentino **J4** (right). The two headers are shown side by side, pin 1 at the top.

| Pin | J1 (Valentino J3) | Where it goes on the shield | | J4 (Valentino J4) | Where it goes on the shield |
|---|---|---|---|---|---|
| 1 | GPIO32 | Q1, digital input of terminal J6 pin 2 (analog is AIN2 on U2 through JP3) | | GPIO5 | SW2 AUTO/MANUAL switch (R3 47k pull-up) |
| 2 | GPIO33 | Q6, digital input of terminal J3 pin 2 (analog is AIN0 on U2 through JP8) | | GPIO18 | Q7, digital input of terminal J3 pin 1 (analog is AIN1 on U2 through JP7) |
| 3 | GPIO16 | WS2812B LED chain data, through R9 330 | | GPIO23 | Q2, digital input of terminal J6 pin 1 (analog is AIN3 on U2 through JP4). Can also be a 1-wire bus, see below |
| 4 | GPIO17 | Relay driver Q3, through R18 | | GPIO19 | U4 TPS2051 ~EN (LOW = OLED and LEDs on, R17 10k pull-up to V33) |
| 5 | V33 | 3.3 V from Valentino | | SDA | I2C SDA: ADS1115 U1 and U2, OLED |
| 6 | GPIO27 | START button S2 (R11 47k pull-up) | | SCL | I2C SCL: ADS1115 U1 and U2, OLED |
| 7 | NC | Not connected (VDD on Valentino J3) | | V33 | 3.3 V from Valentino |
| 8 | NC | Not connected (VDD on Valentino J3) | | V50 | 5 V from Valentino: U4 input, SW1 and the relay coil, sensor terminal J7, both ADS1115 |
| 9 | GND | Ground | | GND | Ground |
| 10 | GND | Ground | | GND | Ground |

All nine usable GPIOs are in use. GPIO25/26 (the TPL5010 watchdog) and GPIO21/22 (I2C) belong to the Valentino.

## Power rails

| Rail | Source | Feeds |
|---|---|---|
| `V50` | Valentino 5 V (J4 pin 8), always on | TPS2051 input and its 100 nF cap C2, emergency switch SW1, relay coil, sensor supply terminal J7, both ADS1115 (U1, U2, including U2 ADDR), the sensor pull-ups R4/R7/R8/R10, config ladder pull-up R12 |
| `V50_SW` | TPS2051 output, switched by GPIO19 | The 9 WS2812B LEDs and OLED1 only |
| `V33` | Valentino 3.3 V | Level-shifter gates, pull-ups on the ESP32 side |

GPIO19 only switches the display and LEDs, so the OLED can be turned off at night while the sensors keep working. The ADS1115 chips answer on I2C whatever GPIO19 is. The sensors on J7 are powered whenever the Valentino 5 V is, and firmware can't switch them off.

## I2C devices

Everything on the bus once the shield is fitted, in address order. Reference designators are from each board's own schematic, so U1, U2 and U3 on the shield are not the same parts as U3 on the Valentino.

| Address | Device | Board | What it does |
|---|---|---|---|
| 0x18 | MCP9808 (U11) | Valentino | Board temperature |
| 0x3C | SSD1309 OLED (OLED1) | Gloria | Display. 0x3D if the DC pad is high; not confirmed from the datasheet |
| 0x40 | INA219 (U8) | Valentino | Solar input current |
| 0x41 | INA219 (U9) | Valentino | Battery current |
| 0x48 | ADS1115 (U10) | Valentino | Analog inputs on the base board |
| 0x49 | ADS1115 (U2) | Gloria | Sensors AIN0-AIN3 (ADDR to VDD) |
| 0x4A | ADS1115 (U1) | Gloria | Config DIP on AIN2, emergency-switch sense on AIN3 (ADDR to SDA) |
| 0x51 | PCF8563 (U3) | Valentino | Real-time clock |

The Valentino rows come from the `ValentinoTest` / `TestGloria` sketch defines. Both Gloria ADS1115 chips are on the always-on `V50`. The OLED is on `V50_SW`, so it only answers when GPIO19 is LOW.

## Sensor terminals

| Terminal pin | GPIO | Digital path | Analog path |
|---|---|---|---|
| J3 pin 2 | 33 | JP5, Q6, R1 | JP8, U2 AIN0 |
| J3 pin 1 | 18 | JP6, Q7, R2 | JP7, U2 AIN1 |
| J6 pin 2 | 32 | JP1, Q1, R5 | JP3, U2 AIN2 |
| J6 pin 1 | 23 | JP2, Q2, R6 | JP4, U2 AIN3 |

Terminal J7 supplies the sensors: pin 1 is `V50` (always on), pin 2 is GND. J3 and J6 have no power pins. The digital jumper (JP5, JP6, JP1, JP2) connects the terminal to the level shifter, and the analog jumper (JP8, JP7, JP3, JP4) connects it to the ADS1115 input. The pull-up on the terminal side of each shifter is a 10k to `V50` (R4, R10, R7 and R8 for Q6, Q7, Q1 and Q2), and each GPIO side has a 4.7k to V33.

### 1-wire probe on GPIO23

A DS18B20 can use J6 pin 1: close JP2, leave JP4 open, take the probe's VCC and GND from J7. R8 (10k to `V50`) is the pull-up. `TestGloria` has a `OneWire` command that scans and reads it. Not tested on hardware yet.

## Status

Schematic finished (ERC 0 errors). PCB in sync with the schematic and nearly routed (2026-10-26). Nothing has been run on hardware.
