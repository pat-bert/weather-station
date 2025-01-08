# ESP32 Weather Station

This project provides the software to run a smart weather station with a small color display based on the popular ESP32 microcontroller.

## Features

- Measurement of ambient temperature, relative humidity, barometric pressure
- Automatic time and date synchronisation over Wi-Fi
- Automatic light sleep
- Color display with three views:
  - Welcome page
  - Sensor dashboard with current readings
  - Sensor history over the last 24 hours

## Details

The weather station uses the following sensors to measure environmental factors:

1. [BME280](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/): Combined ambient temperature, relative humidity and barometric pressure sensor connected via I²C
2. [BH1750](https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf?srsltid=AfmBOoqjl0lo1nhJkEdfEoB3mzjctO5_Caf2fZlqDm33lOlVTLgv2z1l): Digital ambient light sensor connected via I²C

The sensor readings are displayed on a 128x160 pixel TFT color display based on the [ST7735 controller](https://www.displayfuture.com/Display/datasheet/controller/ST7735.pdf) connected via SPI and uses DMA to update the screen without additional CPU load.

The screens are created using [LVGL v9.3](https://lvgl.io/) which can redraw subsets of the whole screen based on which widgets (temperature labels, animations etc.) need to be updated.

## Configuration

The pin mappings and usage of internal I²C and SPI hosts can be parametrized as well as the refresh period of the sensors (default: once per minute).

## Roadmap

- [ ] Sleep display after n seconds of user inactivity
- [ ] Wake display based on proximity reading from time-of-flight sensor
- [ ] Check feasibility of Matter integration, e.g. with Google Home to provide a warning based on humidity and temperature thresholds
- [ ] Implement averaging for correct 24 h history instead of 24 min
- [ ] Read sensors in deep-sleep using LP core

## How to build the software

The software can be built and flashed using the [ESP-IDF extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) provided by Espressif.

The target hardware is the [FireBeetle 2 ESP32-C6](https://wiki.dfrobot.com/SKU_DFR1075_FireBeetle_2_Board_ESP32_C6) which is a low-power IoT board supporting Wi-Fi, Bluetooth, Zigbee and Thread. The board can be powered using a 3.7 V Li-Po battery via the JST-connector. Charging is taken care of by an onboard IC while additional battery protection (e.g. overcurrent while discharging, undervoltage) must be provided externally.

## How to connect the hardware

The GPIO pins of the **Firebeetle** are connected as follows:

GPIO-Pin-Number|Function
---|--------
1  | LCD Reset
2  | User Push-Button  
6  | LCD SPI Clock
7  | LCD SPI Master-out-Slave-in
16 | LCD Chip Select
18 | LCD Data/Command Select
19 | I²C data
20 | I²C clock
