# Hecate
NFC for the Bits n Bytes Embedded Project

Purpose: Handle the NFC connections within the Bits 'n Bytes machine.
Controls the PN532 reader and reports to the main Raspberry Pi when
a valid ID is recieved. Also handles the LED lighting for the cabinet
mirror, with a variety of color settings. This module acts as the
gatekeeper for entering the cabinet, hence the name Hecate, who guards
the gates between the overworld and underworld.