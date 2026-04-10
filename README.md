# Bits 'n Bytes (Embedded)
Bits 'n Bytes is a next generation vending machine by Computer Science House. This repository contains the software for the various sensors and actuators embedded in the cabinet.

# Contributing or Distributing
To contribute to the project or distribute it to targets (ESP32s), you will first need to prepare your environment. Currently, these modules have been tested with MacOS and Windows, but should work for any operating system.

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
By default, the target device is `esp32`. For Bits 'n Bytes, this is the correct device. Should you run into errors pertaining to this, you can manually set the target device using the following command: 
```
idf.py set-target esp32
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

## 8. Flash the Component
Once you have verified that the component has built sucessfully, you must flash the compiled file to the esp32. This can be done using the following command:
```
idf.py flash
```

## 9. Monitor the Device
The easiest way to debug the component during runtime is by monitoring the esp32. This can be done using the following command:
```
idf.py monitor
```
> Note: It is useful to add ESP-specific logging functions, such as `ESP_LOGI` for general logs and `ESP_LOGW`for warnings, as the usual print methods don't work with the monitor command.
