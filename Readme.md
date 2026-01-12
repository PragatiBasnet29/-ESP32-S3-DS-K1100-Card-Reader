### 📋 Copy-Paste Ready `README.md`


# ESP32-S3 DS-K1100 Card Reader Controller

A high-performance IoT access control interface built with **ESP-IDF** and **LVGL**. This project integrates an **ESP32-S3** with a **Hikvision DS-K1100** series card reader to provide a secure, touch-capable (optional) graphical interface for access management.

## 🛠 Features
- **Modern UI:** Powered by LVGL 8.x for smooth graphical feedback.
- **Card Reader Integration:** Handles data from Hikvision DS-K1100 (Wiegand/RS-485).
- **ESP32-S3 Optimized:** Utilizes the S3's native USB-JTAG and hardware acceleration.
- **Modular Design:** Clear separation between hardware drivers and application logic.

## 🔌 Hardware Connection (Standard Setup)

| DS-K1100 Wire | ESP32-S3 Pin | Description |
| :--- | :--- | :--- |
| Red (PWR) | 12V (External) | Card reader requires 12V DC |
| Black (GND) | GND | Common ground with ESP32 |
| Green (D0) | GPIO 4* | Wiegand Data 0 |
| White (D1) | GPIO 5* | Wiegand Data 1 |
| Blue (LED) | GPIO 6* | LED Control (Active Low) |

*\*Pins can be reconfigured in `menuconfig` or `main.c`.*

## 🚀 Getting Started

### 1. Prerequisites
- **ESP-IDF v5.0+** (Recommended: v5.1 or v5.2)
- Visual Studio Code with the ESP-IDF Extension.

### 2. Installation
```bash
# Clone the repository
git clone [https://github.com/PragatiBasnet29/-ESP32-S3-DS-K1100-Card-Reader.git](https://github.com/PragatiBasnet29/-ESP32-S3-DS-K1100-Card-Reader.git)
cd -ESP32-S3-DS-K1100-Card-Reader

# Set the target to ESP32-S3
idf.py set-target esp32s3

```

### 3. Configuration

Open the configuration menu to adjust display pins, card reader GPIOs, and LVGL settings:

```bash
idf.py menuconfig

```

### 4. Build and Flash

```bash
idf.py build
idf.py -p [YOUR_PORT] flash monitor

```

## 📂 Project Structure

* `main/`: Main application code, including `main.c` and UI logic.
* `components/`: Custom drivers for the DS-K1100 and LVGL display porting.
* `sdkconfig.defaults`: Default configuration for S3 hardware.

## ⚠️ Troubleshooting

* **Build Errors:** If `lvgl.h` is not found, run `idf.py reconfigure` to ensure the component manager fetches dependencies.
* **Card Reading:** Ensure the DS-K1100 has a common ground with the ESP32. Wiegand lines typically require a level shifter if the reader outputs 5V signals.

## 📜 License

This project is licensed under the MIT License - see the `LICENSE` file for details.


### 💡 Pro-Tips for your GitHub:

1.  **Add a `LICENSE` file:** GitHub makes this easy. Go to your repo, click **Add file** -> **Create new file**, name it `LICENSE`, and GitHub will let you choose a template (MIT is standard for projects like this).
2.  **Add Images:** I recommend taking a photo of your hardware setup or a video of the screen working. You can link it in the README like this:  
    `![Project Demo](https://github.com/PragatiBasnet29/-ESP32-S3-DS-K1100-Card-Reader/raw/main/demo.jpg)`
3.  **The build error:** If you still get an error during `idf.py build`, it is almost certainly because the **LVGL component** isn't being found. 




