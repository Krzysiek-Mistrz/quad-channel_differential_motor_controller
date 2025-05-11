
# Differential-Drive Motor Controller

This project implements  
- 4-motor H-bridge control via ESP32’s LEDC PWM  
- Quadrature encoder counting via GPIO interrupts  
- Optional PID-closed‐loop differential drive  

A serial command interpreter (in `main.cpp`) exposes multiple commands for I/O, raw motor control, encoder readings, and (planned) diff-drive targets.

---

## Table of Contents

- [Hardware & Wiring](#hardware--wiring)  
- [Project Structure](#project-structure)  
- [Serial Commands](#serial-commands)  
- [Configuration & Tuning](#configuration--tuning)  
- [PID / Diff-Drive Usage](#pid--diff-drive-usage)  
- [Building & Uploading](#building--uploading)  

---

## Hardware & Wiring

1. **ESP32 module**  
2. **4× DC motors** with H-bridge (IN1, IN2, PWM)  
3. **4× quadrature encoders** (channels A/B)  

Wiring defaults live in each `*_driver.h`:

- `encoder_driver.h` → `encPins[i][0..1]`  
- `motor_driver.h`   → `motorPins[i][0..2]`  

Be sure to edit those arrays if your pinout differs.

---

## Project Structure

```
/AOVV/
 ├─ main.cpp             # Serial‐command dispatcher, scheduler
 ├─ commands.h           # ‘a’..‘x’ command codes
 ├─ encoder_driver.h/.cpp
 ├─ motor_driver.h/.cpp
 ├─ diff_controller.h/.cpp
 └─ README.md
```

- **main.cpp**  
  - Reads lines over Serial, splits tokens, and `switch`es on the first char.  
  - Setup calls `initEncoderDriver()`, `initMotorController()`, `resetPID()`.  
  - Loop schedules `updatePID()` every PID interval and auto‐stops on timeout.  

- **encoder_driver**  
  - Exposes `initEncoderDriver()`, `readEncoder(i)`, `resetEncoders()`.  
  - Uses ISRs on rising edges to increment `encCount[i]`.  

- **motor_driver**  
  - Exposes `initMotorController()`, `setMotorSpeed()`, `setMotorSpeeds()`.  
  - Uses ESP32’s LEDC channels + two GPIO outputs per motor.  

- **diff_controller**  
  - Defines `PIDController` struct and globals `leftPID`, `rightPID`.  
  - Provides `initDiffController()`, `setDiffTarget(v,w)`, `resetPID()`, `updatePID()`.  
  - **Note**: `initDiffController()` & `setDiffTarget()` are not yet invoked by main.cpp—see next section.

---

## Serial Commands

| Command | Char | Description                          |
|:-------:|:----:|:-------------------------------------|
| Baud    |  b   | Print current baud rate              |
| A-read  |  a   | `analogRead(pin)`                    |
| D-read  |  d   | `digitalRead(pin)`                   |
| A-write |  x   | `analogWrite(pin,value)`             |
| D-write |  w   | `digitalWrite(pin,value)`            |
| PinMode |  c   | `pinMode(pin,INPUT/OUTPUT)`          |
| ReadEnc |  e   | Print 4 encoder counts               |
| RstEnc  |  r   | `resetEncoders()` + `resetPID()`     |
| M-speeds|  m   | `setDiffTarget(v, w);`               |
| M-raw   |  o   | `setMotorSpeeds(m1…m4)`              |
| PID↑    |  u   | Adjust Kp/Kd/Ki/Ko for both sides     |  

> **Note**  
> v and w in setDiffTarget are linear velocity in x and w is yaw angular vel in CCW direction  

---

## Configuration & Tuning

Open the driver headers to match your hardware:

```c
// encoder_driver.h
static const uint8_t encPins[4][2] = {
  {A1, A2}, … // your encoder A/B pins
};

// motor_driver.h
static const uint8_t motorPins[4][3] = {
  {PWM1, IN11, IN12}, … // your H-bridge pins
};
```

In `diff_controller.cpp` adjust:

```cpp
static constexpr float ENCODER_PPR = <your-encoder-CPR>;
```

Tweak PID gains either in code via in `main.cpp` or at runtime with the `u` command:  
```cpp
float dt        = ...f / PID_RATE;      // control period [s]
float wheel_b   = ...f;                 // track width [m]
float wheel_r   = ...f;                 // wheel radius [m]
initDiffController(dt, wheel_b, wheel_r,
                    1.2f, 0.01f, 0.1f,  //here goes Kp, Kd, Ki
                    1.2f, 0.01f, 0.1f);
```

---

## Building & Uploading

In Arduino IDE:  
1. Select your ESP32 board & port.  
2. Adjust pin & PID parameters as above.  
3. **Upload** and open Serial Monitor at `BAUDRATE`.  
4. Send commands like `r` to reset, `e` to read encoders, `m 0.1 0.2` for diff drive (once added).
  
In PlatformIO:  
1. In ur terminal type: `pio run`.  
2. `pio run -t upload`.  
3. `pio device monitor`.  

---

Enjoy driving your robot!  
Feel free to adapt the code to your own wiring and control scheme under GNU GPL V3

Written by Krzychu @ 2025