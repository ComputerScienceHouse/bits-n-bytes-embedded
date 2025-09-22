# Atlas

Atlas is the ESP32 responsible for all embedded cabinet operations. This includes the following:
- Receiving weight data from the Shelves (over ESP-NOW) and distributing it to the decision makers (Raspberry Pi or Jetson, over UART).
- Controlling the LEDs.
- Controlling and reading the state of the doors and the hatch.
- Controlling the fans that keep the cabinet cool.

## LEDs
The LEDs use [Espressif's LED Strip library](https://espressif.github.io/idf-extra-components/latest/led_strip/api.html).