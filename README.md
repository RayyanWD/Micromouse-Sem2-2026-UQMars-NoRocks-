# Micromouse-Sem2-2026-UQMars-NoRocks-

# NO ROCKS ? – UQ Micromouse 2026

An autonomous micromouse built for the UQ Micromouse competition (4 October 2026).
It explores a 9×9 maze using a flood-fill algorithm, maps the walls as it goes,
and uses that saved map for faster runs to the centre.

**Team:** Rayyan Dewangga and Gordon Tam

**Video:** [YouTube / Google Drive link]
**Design document:** [docs/Design_Document.pdf](docs/Design_Document.pdf)
**Budget:** $180.17 AUD of $1,500 – see [docs/Micromouse_Budget.xlsx](docs/Micromouse_Budget.xlsx)

---

## Hardware

| Part | Purpose |
|---|---|
| ESP32-32U dev board | Main controller |
| 2× N20 motors with encoders (30:1, 7 PPR) | Drive + distance measurement |
| Pololu 32 mm wheels | Traction |
| DFRobot TB6612FNG motor driver (DRI0044) | Drives both motors |
| 3× VL53L0X time-of-flight sensors | Front, left and right wall detection |
| TCS34725 colour sensor | Floor / tile detection |
| MPU-6500 gyro | Mounted for future turn correction |
| MP1584EN buck converter | Battery → 3.3 V logic supply |
| 6×AA battery pack + rocker switch | Power |
| 3D-printed PLA chassis | Single flat deck, under 16 × 16 × 8 cm |

## Wiring summary

| Connection | ESP32 pin |
|---|---|
| Left motor PWM / DIR | GPIO 25 / 26 |
| Right motor PWM / DIR | GPIO 27 / 13 |
| Left encoder A / B | GPIO 34 / 35 |
| Right encoder A / B | GPIO 36 (VP) / 39 (VN) |
| ToF sensors SDA / SCL (I2C bus 1) | GPIO 21 / 22 |
| ToF XSHUT front / left / right | GPIO 16 / 17 / 18 |
| Colour sensor SDA / SCL (I2C bus 2) | GPIO 32 / 33 |
| Start / mode button | GPIO 0 (on-board BOOT) |
| Status LED | GPIO 2 (on-board) |

Power: battery → switch → motor driver VM and buck converter IN+.
Buck converter OUT+ (3.3 V) → ESP32 3V3, driver VCC, all sensors and encoders.
All grounds are common. Full diagram: [electronics/wiring.png](electronics/wiring.png)

## Software

`code/micromouse/micromouse.ino` – a single Arduino sketch for the ESP32.

- **Flood-fill search:** at every cell the mouse reads its walls, updates the map,
  recomputes distances to the centre and moves to the lowest-distance neighbour.
- **Map saved to flash**, so later runs use the known map for faster, straighter paths.
- **Wall centring** using the side ToF sensors, with encoder-based straight-line
  correction where there are no walls.
- **Colour sensor** reads the floor and can optionally treat a marked tile as the goal.

### Modes (BOOT button: short press = next mode, hold ~1 s = start)
The blue LED blinks the mode number.

1. **Explore** – flood-fill search to the centre, then back to start
2. **Speed run** – fastest known path using the saved map
3. **Sensors** – live ToF, colour and encoder readings over Serial (calibration)
4. **Test** – motor direction check, one cell straight, four 90° turns (calibration)
5. **Clear map** – wipe the saved maze

## Building and uploading

1. Install [Arduino IDE](https://www.arduino.cc/en/software) and the **esp32** board package.
2. Install the libraries **VL53L0X** (Pololu) and **Adafruit TCS34725** from Library Manager.
3. Open `code/micromouse/micromouse.ino`, select **ESP32 Dev Module**, and upload.
4. Open Serial Monitor at **115200 baud** to see sensor status and debug output.

Calibration values (wheel size, turn scale, wall thresholds, motor speeds)
are all in the `CALIBRATION` section at the top of the sketch.

## Repository structure

```
code/           Arduino source
docs/           Design document, budget spreadsheet, receipts
cad/            Chassis STL/STEP files
electronics/    Wiring diagram
media/          Photos and video link
```
