# AirClock 2 - Wifi Clock and Air Quality Monitor

Feng Zhou, July-September, 2020

## Overview

This is a Wifi clock and air quality monitor based on [AirClock](https://github.com/greenpig/airclock), which is in turn inspired by [Kevin Norman's Air Quaility Meter project](https://kn100.me/where-embedded-meets-the-internet-building-your-own-air-quality-meter/)(See [PDF](hardware.pdf) for a local copy).  Compared to AirClock, the text LCD screen is changed to a 5-inch 256x128 graphics LCD, and
a 3D-printed enclusure design is included. 

Features,

 * Air quality monitoring: VOC, CO2, temperature, humidity, air pressure and overall Air-Quality Index.
 * Always-accurate clock (network-synced).
 * Long lasting Lithium battery lasting 1 month per charge (through standard micro-USB port).
 * Reflective monochrome LCD screen for easy viewing in bright environment including outdoors. For dark
   environment, back light could be turned on by pressing button.
 * Easy Wifi configuration, not hard-coded Wifi password.

Enjoy!

## Final product

TODO: See [inside image](doc/AirClock_1.jpg), [front panel](doc/AirClock_2.jpg), and [breadboard version](doc/AirClock_breadboard.jpg).

## Hardware
 
See photos of [board up side](doc/Board_up.jpeg), [board bottom side](doc/Board_bottom.jpeg), [LCD pcb](doc/LCD_board.jpeg).

Bill of materials:

|---|---|
| ESP32 board  | Wemos Lolin32 v1.0.0 |
| LCD screen | JLX256128G-931-PN (I2C variant) |
| Sensor module | Bosch BME680 module (see board photo) | 
| Battery    | 2000mAh Lipo battery |
| Button     | See board photo |
| PCB        | 2.54mm-pitch 7cm*9cm experiment PCB |
|---|---|

Wirings,

 * LCD to ESP32
   * D0 - SCL, D1 - SDA, GND/CS - GND, RST - 16, RS - VDD - 3V, VSS - GND, LEDA - 17.
 * LCD to LCD
   * D1 - D2 - D3
   * VDD - RW - RD - D4 - D5
   * D6 - D7 - CS
 * BME680 to ESP32
   * SDA - SDA, SCL - SCL, GND - GND, VCC - 3V
 * Button between pin 19 and GND.

## Instructions

1. Get the necessary components as mentioned in the original [article](hardware.pdf), in addition to the
   following:

   a. A micro switch. See image above for what I use.
   b. (Optional) A LiPo battery. The battery connector on the Wemos Lolin32 is JST XH2-2.54mm. The one I use has 2000mAh capacity.
   c. (Optional) I used a 5x7cm prototype PCB for the circuits and a 125x80x32mm ABS case for holding everything.

2. Assembly everything according to Kevin's article.

3. Install [Arduino IDE](https://www.arduino.cc/en/main/software) if you have not yet done so.

4. Within Arduino IDE, use `Sketch->Library->Manage Libraries` to install the following libraries:
  * BSEC Library by Bosch Sensortec (v1.5.1474) (needs extra changes to install, see 'Talking to the BME680' in [Kevin's article](https://kn100.me/where-embedded-meets-the-internet-building-your-own-air-quality-meter/))
  * U8g2 2.27.6
  * NTPClient 3.2.0
  * PubSubClient 2.8.0
  * EasyButton 2.0
  * PageBuilder 1.4.2
  * AutoConnect 1.1.7

5. Open `airclock2/airclock2.ino` in Arduino IDE.

6. Connect the dev board through USB, and install the firmware with `Sketch->Upload`.

## Usage

Once installed, the firmware works as follows,

1. The first time when the wifi clock is powered up, it needs some setup for it to connect
   to the Wifi. Just follow the instructions on the screen. Use your phone to connect to the
   clock's Wifi network. Then in the pop-up screen on your phone, choose "add new AP", put in
   Wifi information and you will be done. Remember the board only support 2.4Ghz networks, 
   not 5Ghz ones.

2. After the setup is done, the clock should show accurate time as displayed in the picture
   above. 

3. Press the button once to turn on backlight. Press it again to switch to air quality 
monitoring, where you can see air quality index, measure temperature, humidity, pressure, 
VOC and CO2. Press it again to show Wifi configuration page. The last part is not finished 
yet.

## Notes
 
 * Time is synced from 'ntp.aliyun.com' every 10 minutes. Change it to other servers in airclock.ino if
   if you need to.

 * For initial use, the air quality sensor (BME680) needs quite a long time to 'calibrate' itself.
   Before that is done, the readings will be off and it is indicated with a '?' at bottom right 
   of screen. It may take 2-3 days for this process to complete.
   
 * MQTT reporting to server is currently disabled in the firmware as I do not need it. Refer to 
   Kevin's original code if you want to restore it.

## Changelog

 * 2.0
   * Switch to new 256x128 graphical LCD
 * 0.6
   * In wifi screen, long press to re-setup Wifi.
   * Reduce clock frequency to 80Mhz to save power.
   * Only turn on Wifi at NTP time to save power. Power is now at around 20-30mA @ 5V.
 * 0.5
   * Initial version.