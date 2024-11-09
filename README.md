# Bits 'n Bytes (Embedded)
Bits 'n Bytes is a next generation vending machine by Computer Science House. This repository contains the software for the various sensors and actuators embedded in the cabinet.

# Contributing or Distributing
To contribute to the project or distribute it to targets (ESP32s), you will first need to prepare your environment. Note that I have only done this on MacOS machines, so the instructions here might not work exactly for Linux or Windows. Feel free to make a PR and update them based on your own experience!
## 1. Install ESP-IDF
This project is built on ESP-IDF.

[Install Guide for MacOS and Linux](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html)

[Install Guide for Windows](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html)

## 2. Clone this Repository
Clone this repository to somewhere on your system.

## 3. Export ESP-IDF
In order to use ESP-IDF, you have to export the commands each time you open a new shell.
```
. ~/path/to/esp-idf/export.sh

# Example
. ~/esp/esp-idf/export.sh
```

## 4. Navigate to a Component
Navigate to the location of one your clone for this repository, and then to the component you want to build (doors, shelf). Note that the command is completely dependent on your checkout location.
```
cd path/to/clone/doors
cd path/to/clone/shelf

# Example
cd ~/Documents/bits-n-bytes-embedded/doors
cd ~/Documents/bits-n-bytes-embedded/shelf
```

## 5. Set the Target Device
By default, the target device is `esp32`. For Bits 'n Bytes, we have been using ESP32-S2. To reflect this, run
```
idf.py set-target esp32s2
```

## 6. Open Menuconfig
For ESP-IDF to generate things properly, I have found that it is useful to open menuconfig. To do this, run
```
idf.py menuconfig
```
Once it opens, just hit `q` to quit. You don't need to modify any settings.

## 7. Build the Component
```
idf.py build
```
