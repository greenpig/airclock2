# ESP32-based Wifi Clock and Air Quality Monitor

Feng Zhou, July-August, 2020

# Changes

2020-9-6, enabling deep sleep

  * [Guide](https://randomnerdtutorials.com/esp32-timer-wake-up-deep-sleep/)
  * Design:
    1. Wake up (reboot) every 3 seconds, or until next minute start, which ever is shorter.
    2. Initialize everything.
    3. Check button press, if yes, turn on backlight or switch to next screen.
    4. Start wifi and update NTP every 30 minutes. Turn off wifi afterwards.
    5. Read BME680 readings. (Need more instruction on how to use BSEC library with deep sleep).
    6. Check current screen. Update time if it is clock screen. Update sensor reading if air quality screen.
  * TODO: Button is currently on GPIO19, which is not [RTC GPIO](https://i2.wp.com/randomnerdtutorials.com/wp-content/uploads/2018/08/ESP32-DOIT-DEVKIT-V1-Board-Pinout-36-GPIOs-updated.jpg?w=750&ssl=1). Should
   go with something like RTC_GPIO0/GPIO36.
  * BUG: Reset接在16上面，会因为处理器断电而走低，所以需要一个pull-up resistor？
    * 正确解决办法应该是接在RTC GPIO上面，这样就可以保证不要走低

2020-8-8
 
 * Created HugeNumbers_I2C.cpp to display `4*4` larger numbers.

2020-8-2 Optimizaing power

Original power @ 5V: 0.07A. 

 * Change clock to 80Mhz: `setCpuFrequencyMhz(80);`:
   0.05A 

# Future work

 * Lower power consumption by adding
   power management to the firmware (currently unoptimized version is at around 80mA @ 5V). 

 * Press buttom for 10 seconds in Wifi screen to clear Wifi AutoConnect data.

 * Show current SSID in Wifi screen.

 * Finish configuration-through-wifi. It would be nice to be able to do things like setting
   alarms, changing time zones and etc.

 * MQTT for reporting air quality to server.

# Design notes

## Buttons

 * [Push button with ESP32 – GPIO pins as digital input](https://microcontrollerslab.com/push-button-esp32-gpio-digital-input/)
  * GPIO 12 should be OK. There's internal pull-up resistors, needs to be enabled.

## Soft AP for setting up Wifi

 * [ESP8266/ESP32 Connect WiFi Made Easy](https://www.hackster.io/hieromon-ikasamo/esp8266-esp32-connect-wifi-made-easy-d75f45)

## WebServer for configuration

 * For setting time zone, alarm time, NTP server address and etc.

## EEPROM

 * Save wifi ssid/password, wall clock time, air quality sensor data

## Time keeping and NTP

 * NTP to get time after each power-up.
 * cn.ntp.org.cn
 * [Getting Date and Time with ESP32 on Arduino IDE (NTP Client)](https://randomnerdtutorials.com/esp32-ntp-client-date-time-arduino-ide/)
 * [Accuracy is about 1%, so need to sync once per hour to get to 1 min accuracy](https://github.com/espressif/arduino-esp32/issues/3641)

## Display BigNumber

 * `#include <BigNumbers_I2C.h>`

## Fancy 2004 LCD demo

## Power Saving

 * [Max.K's low power handheld computer](https://hackaday.io/project/169103-low-power-esp32-handheld)
 * [ESP32 Power Modes](https://lastminuteengineers.com/esp32-sleep-modes-power-consumption/)
   * With radio: 160-260mA
   * Without radio: 3-20mA
 * [ESP32: Tips to increase battery life](https://www.savjee.be/2019/12/esp32-tips-to-increase-battery-life/)
 * [ESP32 Deep Sleep Tutorial](https://www.hackster.io/nickthegreek82/esp32-deep-sleep-tutorial-4398a7)
 * [Save power by (reliably) switching the ESP WiFi on and off](https://esp8266hints.wordpress.com/2017/06/29/save-power-by-reliably-switching-the-esp-wifi-on-and-off/)
 * [ESP32 low power](https://community.hiveeyes.org/t/low-power-esp32-hardware-and-software/538/2)
 * [ESP + BME280 low power](https://github.com/G6EJD/ESP32-Deep-Sleep-Ultra-Low-Power-Trial/blob/master/ESP32_Thingspeak_Deep_Sleep_BME280.ino)
 * [Tech Note 062 - WEMOS Lolin32 (surprisingly low power demands when powered by 3v3)](https://www.youtube.com/watch?v=k_7eZ5ZpSMY)

