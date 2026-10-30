# 🚁 CYD Pro-Deck: ESP32 FPV Drone Controller

A high-performance, custom-built FPV dashboard and flight controller for generic Wi-Fi toy drones (like JJRC/Flow), built entirely on the ESP32 CYD (Cheap Yellow Display).

This project replaces the laggy official phone apps with a dedicated hardware deck featuring a flawless, flicker-free UI, hardware-level video cropping, and maximum Wi-Fi transmission power.

## ✨ Features
* **Zero-Flicker Hardware UI:** Utilizes `ClipRect` math to physically prevent the video decoder from drawing over the touchscreen buttons.
* **Cinematic Widescreen Crop:** Captures the drone's native 240x320 vertical video stream and performs a hardware-level 90-degree rotation and perfect center-crop for full widescreen FPV.
* **Spectator Stealth Mode:** Completely mutes outbound UDP flight commands while maintaining the RTSP video stream, allowing you to fly with your original physical remote while using the CYD solely as a passive FPV monitor.
* **On-Screen Stunt Pad:** Pop-up directional overlay to trigger 360-degree flips on demand.
* **Maximized Range:** Forces the ESP32 Wi-Fi radio to broadcast at its absolute physical limit (`19.5dBm`).
* **Overclocked Rendering:** CPU locked at 240MHz and SPI bus overclocked to 80MHz for maximum MJPEG decoding framerates.

## 🛠 Hardware Requirements
* **ESP32-2432S028R (Cheap Yellow Display):** 2.8" TFT Touchscreen with ILI9341 display driver and XPT2046 resistive touch chip.
* **Wi-Fi Toy Drone:** Any drone broadcasting an unencrypted Wi-Fi access point with an RTSP video stream on `192.168.1.1:7070` and UDP control on port `7099` (e.g., Flow UFO, JJRC).

## 📚 Required Libraries
Install these via the Arduino Library Manager:
1. `LovyanGFX` (Display Driver)
2. `XPT2046_Touchscreen` (Hardware Touch Driver)

## 🚀 Installation & Setup
1. Clone this repository or download the ZIP.
2. Open `CYD_Pro_Deck.ino` in the Arduino IDE.
3. In the code, change `const char* DRONE_SSID = "FLOW-UFO-8c5405";` to match your drone's actual Wi-Fi network name.
4. Set your Arduino Board to **ESP32 Dev Module**.
5. Upload the code to your CYD.
6. Power on your drone, wait for its Wi-Fi network to appear, and tap **CONNECT** on the CYD dashboard.

## 🎮 Controls
* **Fly Pad:** Standard Mode-2 virtual joysticks (Throttle/Yaw on left, Pitch/Roll on right).
* **Calib:** Sends the gyroscope calibration sequence to fix drifting before takeoff.
* **Stop!:** Emergency motor kill-switch.
