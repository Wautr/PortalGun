# PortalGun
Code to control a Portal Gun (From Rick and Morty) with updated hardware and functionality.

## What's New in This Fork
- **ESP32 Support**: The project is updated to work with ESP32.
- **UART Debugging**: Added UART debug output to monitor encoder rotation, button presses, and system events.
- **Improved Rotary Encoder Handling**: Better detection of rotation (clockwise/counterclockwise) and button presses.

---

## Libraries
Download and install the following libraries:

- [ESP32Encoder](https://github.com/madhephaestus/ESP32Encoder)
- [Adafruit_GFX](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit_LEDBackpack](https://github.com/adafruit/Adafruit-LED-Backpack-Library)

---

## Pin Definitions

### ESP32 Version

| **Component**   | **ESP32 Pin**  |
|-----------------|----------------|
| **Rotary Encoder (CLK)** | GPIO 32  |
| **Rotary Encoder (DT)**  | GPIO 33  |
| **Rotary Encoder Button (SW)**  | GPIO 25 |
| **LED Display (SCL)**  | GPIO 22 |
| **LED Display (SDA)**  | GPIO 21 |
| **Top Bulb LED**  | GPIO 15 |
| **Front Right LED**  | GPIO 26 |
| **Front Center LED**  | GPIO 27 |
| **Front Left LED**  | GPIO 14 |


---

## Installing Firmware (ESP32)
1. [Set up the Arduino IDE for ESP32](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-macos-linux/).
2. Connect the ESP32 and upload the sketch.
3. Use the Serial Monitor (set to `115200` baud) to view debug output.

---

## Button & Encoder Behavior
- **Single Click**: Wakes the ESP32 from deep sleep.
- **Single Click**: Turns off the LEDs and enters deep sleep.
- **Encoder Rotation**: Detects and logs clockwise/counterclockwise movement.

haven't actually wired the LED's yet and checked the main device behavior :) just got the sketch + display running on this platform.

---

## Debugging (UART)
All actions, such as encoder rotations and button presses, are logged to the serial monitor via UART at `115200` baud for easy troubleshooting.

