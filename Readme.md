# ESP32-S3 DS-K1100 Card Reader Controller

ESP-IDF project for controlling Hikvision DS-K1100 Series Card Readers using ESP32-S3 with 4.3" touchscreen display.

## Project Structure

```
card_reader_controller/
├── CMakeLists.txt              # Root CMake configuration
├── sdkconfig.defaults          # Default SDK configuration
├── README.md                   # This file
├── components/                 # External components (LVGL)
│   ├── lvgl/
│   └── lvgl_esp32_drivers/
└── main/
    ├── CMakeLists.txt          # Main component CMake
    └── main.cpp                # Main application code
```

## Hardware Requirements

### ESP32-S3 Development Board
- ESP32-S3 with 4MB+ Flash
- PSRAM (recommended for LVGL)

### 4.3" TFT Display
- Controller: ILI9488 or ILI9341
- Resolution: 480x320
- Touch: XPT2046 or FT6236

### DS-K1100 Series Card Reader
- DS-K1104M or DS-K1104MK (Metal vandal-proof)
- Supports RS-485 and Wiegand protocols

## Pin Configuration

### Display Connections
```
LCD_MOSI  -> GPIO 11
LCD_MISO  -> GPIO 13
LCD_CLK   -> GPIO 12
LCD_CS    -> GPIO 10
LCD_DC    -> GPIO 14
LCD_RST   -> GPIO 9
LCD_BCKL  -> GPIO 15
TOUCH_CS  -> GPIO 33
```

### RS-485 Connections
```
RS485_TX     -> GPIO 17  -> Card Reader Yellow (485+)
RS485_RX     -> GPIO 18  -> Card Reader Blue (485-)
RS485_DE_RE  -> GPIO 16  -> Direction control
GND          -> GND      -> Card Reader Black (GND)
+12V         -> +12V     -> Card Reader Red (PWR)
```

### Wiegand Connections
```
WIEGAND_D0   -> GPIO 19  -> Card Reader Green (W0)
WIEGAND_D1   -> GPIO 20  -> Card Reader White (W1)
LED_CONTROL  -> GPIO 21  -> Card Reader Brown (LED)
BEEP_CONTROL -> GPIO 22  -> Card Reader Purple (Beep)
CASE_SENSOR  -> GPIO 23  -> Card Reader Gray (Tamper)
GND          -> GND      -> Card Reader Black (GND)
+12V         -> +12V     -> Card Reader Red (PWR)
```

## Software Setup

### 1. Install ESP-IDF

Follow the official ESP-IDF installation guide:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/

For VS Code, install the ESP-IDF extension.

### 2. Add LVGL Components

Clone LVGL and drivers into the `components` folder:

```bash
cd card_reader_controller
mkdir -p components
cd components

# Clone LVGL v8.3
git clone -b release/v8.3 https://github.com/lvgl/lvgl.git

# Clone LVGL ESP32 Drivers
git clone https://github.com/lvgl/lvgl_esp32_drivers.git
```

### 3. Configure LVGL

Copy LVGL configuration:
```bash
cp lvgl/lv_conf_template.h lvgl/lv_conf.h
```

Edit `components/lvgl/lv_conf.h`:
```c
// Change line 15 from:
#if 0 /* Set it to "1" to enable content */

// To:
#if 1 /* Set it to "1" to enable content */
```

### 4. Build the Project

Using ESP-IDF Terminal:
```bash
cd card_reader_controller
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor  # Replace COMx with your port
```

Using VS Code ESP-IDF Extension:
1. Press `Ctrl+E` then `D` to select device
2. Press `Ctrl+E` then `B` to build
3. Press `Ctrl+E` then `F` to flash

### 5. Configure Card Reader DIP Switches

#### For RS-485 Mode:
- Switch 1-4: Set reader address (default: 0001 = address 1)
- Switch 5: 0 (disable encryption for testing)
- Switch 6: **0** (RS-485 protocol)
- Switch 7: N/A
- Switch 8: 1 if last device on bus (termination)

#### For Wiegand Mode:
- Switch 1-4: Don't care
- Switch 5: Set based on card security needs
- Switch 6: **1** (Wiegand protocol)
- Switch 7: 1 for 26-bit, 0 for 34-bit
- Switch 8: N/A

## Usage

### Interface Overview

The touchscreen displays:
- **Mode Selection**: Switch between RS-485 and Wiegand protocols
- **Open Door**: Send door unlock command
- **Reset Alarm**: Clear tamper alarm
- **Read Card Status**: Query card reader status
- **Status Display**: Shows connection status, last card ID, and alerts

### RS-485 Mode Features
- Query card reader status
- Send door open commands
- Monitor reader online/offline status
- View card IDs from successful reads

### Wiegand Mode Features
- Receive 26-bit or 34-bit Wiegand data
- Control LED indicators
- Control buzzer feedback
- Monitor card reads in real-time

## Troubleshooting

### Display Not Working
1. Check SPI wiring connections
2. Verify display controller type in `sdkconfig.defaults`
3. Adjust pin definitions in `main.cpp`
4. Check backlight connection and polarity

### Touch Not Responding
1. Verify touch controller type (XPT2046 or FT6236)
2. Check TOUCH_CS connection
3. Calibrate touch in code if needed
4. Test touch with LVGL examples first

### RS-485 No Communication
1. Check TX/RX aren't swapped
2. Verify DE/RE pin controls direction correctly
3. Ensure proper termination (120Ω if needed)
4. Check card reader DIP switch 6 is set to 0
5. Verify baud rate (default 9600)
6. Check cable quality and length (<100m)

### Wiegand No Data
1. Check D0/D1 wiring and polarity
2. Verify card reader DIP switch 6 is set to 1
3. Set correct protocol (26-bit or 34-bit) with switch 7
4. Test with known working card
5. Check pull-up resistors on D0/D1

### Build Errors
```bash
# Clean and rebuild
idf.py fullclean
idf.py build

# Update components
cd components/lvgl
git pull
cd ../lvgl_esp32_drivers
git pull
```

## Advanced Configuration

### Change Display Resolution

Edit `sdkconfig.defaults`:
```
CONFIG_LV_HOR_RES_MAX=480  # Your width
CONFIG_LV_VER_RES_MAX=320  # Your height
```

### Adjust Touch Calibration

In `main.cpp`, modify touch calibration values:
```cpp
uint16_t calData[5] = {275, 3620, 264, 3532, 1};
```

### Multiple Card Readers

To support multiple readers on RS-485 bus:
1. Set different addresses via DIP switches
2. Modify code to track multiple reader_status structures
3. Add UI to select active reader

### Custom Protocol Implementation

The RS-485 protocol in the code is simplified. For production:
1. Implement actual Hikvision protocol
2. Add CRC checking
3. Handle retries and timeouts
4. Add authentication if required

## License

This project is provided as-is for educational and development purposes.

## Support

For issues related to:
- ESP-IDF: https://github.com/espressif/esp-idf/issues
- LVGL: https://forum.lvgl.io/
- Card Reader: Consult Hikvision documentation

## References

- [DS-K1100 User Manual](UD05659B-E_Baseline_DS-K1100-Series_Card-Reader_User-Manual_V2.0_20200617.pdf)
- [DS-K1104 Datasheet](DS-K1104_Datasheet_20240222.pdf)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [LVGL Documentation](https://docs.lvgl.io/)
