
// AirClock 2 - Wifi Clock and Air Quality Monitor
// Based on Arduairs
// Github: https://github.com/greenpig/airclock

// Configurarions

// Need to enable this to enable air quality monitoring
#define ENABLE_BME680

// End configuration

#include "bsec.h"
#include <U8g2lib.h>
#include <WiFi.h>
#include <WifiUdp.h>
#include <NTPClient.h>
#include <EEPROM.h>
#include <EasyButton.h>
#include <time.h>

#include "util.h"

#define VERSION "v2.5"

// Controls which pins are the I2C ones.
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#define BUTTON_PIN 19
#define BACKLIGHT_PIN 17

#define FONT78 u8g2_font_logisoso78_tn
#define FONT16 u8g2_font_crox2hb_tf

// Controls how often the code persists the BSEC Calibration data
#define STATE_SAVE_PERIOD  UINT32_C(60 * 60 * 1000) // every 60 minutes

char ssid[] = "zfzl-iot";
char passwd[] = "work4you";

// Controls the offset for the temperature sensor. My BME680 was reading 4 degrees higher than another thermometer I trusted more, so my offset is 4.0.
const float tempOffset = 4.0;

// Configuration data for the BSEC library - telling it it is running on a 3.3v supply, that it is read from every 3 seconds, and that the sensor should take into account the last 4 days worth of data for calibration purposes.
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

// Create an object of the class Bsec
Bsec iaqSensor;

String output;

// This stores a question mark and is eventually replaced with a space character once the BSEC library has decided it has enough calibration data to persist. It is displayed in the bottom right of the LCD as a kind of debug symbol.
RTC_DATA_ATTR String sensorPersisted = "?";

uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

WiFiClient espClient;

// The I2C address of your LCD, it will likely either be 0x27 or 0x3F
const uint8_t LCD_ADDR = 0x27;

// 256*128 LCD screen
//U8G2_ST75256_JLX256128_F_HW_I2C lcd(U8G2_R0, /* reset */ 16);
U8G2_ST75256_JLX256128_F_HW_I2C lcd(U8G2_R0, U8X8_PIN_NONE);

// Current active screen
RTC_DATA_ATTR int screen = 0;       // Current screen
RTC_DATA_ATTR int active = 1;       // Are we "active" (backlight on)
RTC_DATA_ATTR int activeMillis = millis(); // Millis when we become active
// Stay active for this long (millis)
#define ACTIVE_DURATION 10000


// NTP and time keeping
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.aliyun.com");
int timezone = 8;   // Beijing time
RTC_DATA_ATTR unsigned long ntpEpoch = 1577808000;    // 2020-1-1 0:0:0 Beijing time
RTC_DATA_ATTR unsigned long ntpMillis;    // millis() value at NTP sync time
RTC_DATA_ATTR unsigned long ntpAttemptMillis;    // 上一次尝试同步时间的millis，不管成功没成功（可能Wifi不通什么的）

// Keep track of last displayed minutes. Screen is refreshed only when this changes.
RTC_DATA_ATTR int lastMin = -1;

EasyButton button(BUTTON_PIN);

#define SCREEN_CLOCK 0
#define SCREEN_AIR 1
#define SCREEN_WIFI 2

// Deep sleep state
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR unsigned long millisOffset=0;

unsigned long offsetMillis() {
    return millis() + millisOffset;
}

void backlight(bool on) {
  if (on) {
    digitalWrite(BACKLIGHT_PIN, 1);
  } else {
    digitalWrite(BACKLIGHT_PIN, 0);
  }
}

void onButtonPressed() {
  Serial.println("Button pressed");
  screen = (screen + 1) % 2;
}

bool wifiOn(const char* ssid, const char* password, unsigned long timeout) {
  Serial.printf("Wifi connecting to %s\n", ssid);
  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.begin(ssid, password);
  unsigned long tm = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - tm > timeout)
      return false;
  }
  return true;
}

bool wifiOff() {
  Serial.println("Wifi off.");
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}

bool lcdInitialized = false;

void initLcd() {
  if (!lcdInitialized) {
    lcd.initDisplay();
    lcd.setFont(FONT16);
    lcd.setFontRefHeightExtendedText();
    lcd.setDrawColor(1);
    lcd.setFontPosTop();
    lcd.setFontDirection(0);    
    lcd.setContrast(0xa0);  
    lcd.setPowerSave(0);
    lcdInitialized = true;  
  }
}

void initAllBoot() {  
//  setCpuFrequencyMhz(80);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);  

  // Set up screen
  pinMode(BACKLIGHT_PIN, OUTPUT);

  // Set up button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button.begin();

  button.onPressed(onButtonPressed);
}

void initFirstBoot() {
  Serial.println("Airmonitor started.\n");
  disableCore0WDT();    // see: https://forum.arduino.cc/index.php?topic=621311.0
  disableCore1WDT();
}

void initBME() {
#ifdef ENABLE_BME680
  // Your module MAY use the primary address, which is available as BME680_I2C_ADDR_PRIMARY
  Serial.println("Starting BME680");
//  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  Serial.println("BME680 started");
  output = "BSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();
  
  Serial.println("Setting bsec config");
  iaqSensor.setConfig(bsec_config_iaq);

  //loadState here refers to the BSEC Calibration state.
  Serial.println("Loading state");
//  loadState();
  Serial.println("State loaded");

  // List all the sensors we want the bsec library to give us data for
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.setTemperatureOffset(tempOffset);

  // Receive data from sensor list above at BSEC_SAMPLE_RATE_LP rate (every 3 seconds). There is also BSEC_SAMPLE_RATE_ULP - which requires a configuration change above.
  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
#else
  Serial.println("Firmware has air quality sensor disabled.");
#endif  
}

void clockScreen() {
  unsigned long nowMillis = offsetMillis();
  unsigned long nowEpoch = ntpEpoch + (nowMillis - ntpMillis) / 1000;

  struct tm dt;
  epochToDateTime(nowEpoch, timezone, &dt);

  if (dt.tm_min != lastMin) {
    initLcd();
    lastMin = dt.tm_min;
    // print time
    lcd.clearBuffer();
    lcd.setFont(FONT78);
    lcd.setCursor(15,8);
    lcd.printf("%02d:%02d", dt.tm_hour, dt.tm_min);
    lcd.setFont(FONT16);
    lcd.setCursor(90,110);
    lcd.printf("%04d-%02d-%02d", dt.tm_year, dt.tm_mon + 1, dt.tm_mday);
    lcd.sendBuffer();  
//    delay(1000);
  }
}

void airQualityScreen() {
  initLcd();  
  lcd.clearBuffer();
  lcd.setCursor(0, 0);
  lcd.println("This is the air quality screen");
  lcd.sendBuffer();

  /*
  // iaqSensor.run() will return true once new data becomes available
  if (iaqSensor.run()) {
//    Serial.println("New sensor data is available");
    lcd.clearBuffer();
    displayIAQ(String(iaqSensor.staticIaq));
    displayTemp(String(iaqSensor.temperature));
    displayHumidity(String(iaqSensor.humidity));
    displayPressure(String(iaqSensor.pressure / 100));
    displayCO2(String(iaqSensor.co2Equivalent));
    displayVOC(String(iaqSensor.breathVocEquivalent));
    displaySensorPersisted();
    updateState();
    lcd.sendBuffer();
  } else {
//    Serial.println("Waiting for sensor data");
    checkIaqSensorStatus();
  }
  */
}


void setup(void)
{
  Serial.begin(115200);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  pinMode(16, OUTPUT);
  digitalWrite(16, 1);
  if (bootCount == 1) {
    // Manually reset display
    digitalWrite(16, 0);
    delay(100);
    digitalWrite(16, 1);  
  }

  initAllBoot();

  // #1: First boot, connect to wifi and get initial NTP time
  if (bootCount == 1) {
    initFirstBoot();

    int tries = 0;
    initLcd();
    while (true) {
      lcd.clearBuffer();
      lcd.setCursor(0, 0);
      lcd.printf("Connecting to network %s...", ssid);
      if (tries > 0) {
        lcd.setCursor(0, 20);
        lcd.printf("Retry #%d.", tries);
      }
      lcd.sendBuffer();
  
      if (wifiOn(ssid, passwd, 30000))
        break;      
    }
    lcd.clearBuffer();
    lcd.drawStr(0, 0, "Getting time from server...");
    lcd.sendBuffer();
  
    Serial.println("Syncing time...");
    timeClient.begin();
    while (!timeClient.update())
      timeClient.forceUpdate();
    ntpEpoch = timeClient.getEpochTime();
    ntpAttemptMillis = ntpMillis = offsetMillis();
    Serial.printf("NTP epoch = %ul, ntpAttemptMillis=%ul\n", ntpEpoch, ntpAttemptMillis);
  
    wifiOff();
  }

  // #2, respond to button press
  button.read();      // this will call onButtonPressed() to switch screen

  // #3, Update NTP every 30 minutes.
  unsigned long now = offsetMillis();
  if (now - ntpAttemptMillis > 30 * 60 * 1000) {
    Serial.printf("NTP update: start. now=%ul, ntpAttemptMillis=%ul\n", now, ntpAttemptMillis);
    ntpAttemptMillis = now;
    if (wifiOn(ssid, passwd, 30000)) {
      Serial.println("Syncing time...");
      timeClient.begin();
      if (timeClient.update()) {
        ntpEpoch = timeClient.getEpochTime();
        ntpMillis = offsetMillis();
        Serial.printf("NTP update: %ul\n", ntpEpoch);
      }
      wifiOff();
    }
  }

  // #4 Update display according to current screen
  Serial.printf("Update screen %d\n", screen);
  switch (screen) {
    case 0: {
        clockScreen();
        break;
      }
    case 1: {
        airQualityScreen();
        break;
      }
  }
//  vTaskDelay(10);

  Serial.println("Going to sleep");
  // Go to deep sleep, wake up every 3 seconds
  int sleepMillis = 3000;
  esp_sleep_enable_timer_wakeup(sleepMillis * 1000);
  millisOffset = offsetMillis() + sleepMillis;
//  esp_sleep_enable_ext0_wakeup(GPIO_NUM_12, 0);    // Press button to wake. 0 means wake up on high to low
  esp_deep_sleep_start();

}


// Function that is looped forever
void loop(void)
{
  // XXX: this is never reach because we go to deep sleep at end of setup()
  
  // 0: clock, 1: air quality, 2: Wifi Setup
  //  Serial.println(digitalRead(BUTTON_PIN));
  button.read();


}

// checks to make sure the BME680 Sensor is working correctly.
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor error");
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
    delay(5000);
    lcd.clear();
    checkIaqSensorStatus();
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor error");
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
    delay(5000);
    lcd.clear();
  }
}

// The below display functions display data as well as publishing it to the MQTT broker. They expect the area that they are rendering to to be free of characters.
void displayIAQ(String iaq)
{
  lcd.setCursor(0, 0);
  lcd.print("IAQ   ");
  lcd.print(iaq);
  char carr[iaq.length()];
  iaq.toCharArray(carr, iaq.length());
  //  broker.publish("bme680/iaq", carr);
}

void displayTemp(String tmp)
{
  lcd.setCursor(128, 0);
  lcd.print("TEMP  ");
  lcd.print(tmp);
  lcd.write(248);  // degree symbol
  lcd.print("C");
  char carr[tmp.length()];
  tmp.toCharArray(carr, tmp.length());
  //  broker.publish("bme680/temperature", carr);
}

void displayHumidity(String humidity)
{
  lcd.setCursor(0, 20);
  lcd.print("HUMID ");
  lcd.print(humidity + "%");
  char carr[humidity.length()];
  humidity.toCharArray(carr, humidity.length());
  //  broker.publish("bme680/humidity", carr);
}

void displayPressure(String pressure)
{
  lcd.setCursor(128, 20);
  lcd.print("PRESS ");
  lcd.print(pressure);
  char carr[pressure.length()];
  pressure.toCharArray(carr, pressure.length());
  //  broker.publish("bme680/pressure", carr);
}

void displayCO2(String co)
{
  lcd.setCursor(0, 40);
  lcd.print("CO2   " + co + "ppm");
  char carr[co.length()];
  co.toCharArray(carr, co.length());
  //  broker.publish("bme680/co", carr);
}

void displayVOC(String voc)
{
  lcd.setCursor(128, 40);
  lcd.print("VOC " + voc + "ppm");
  char carr[voc.length()];
  voc.toCharArray(carr, voc.length());
  //  broker.publish("bme680/voc", carr);
}

void displaySensorPersisted()
{
  lcd.setCursor(248, 112);
  lcd.print(sensorPersisted);
  Serial.print("Sensor persisted: ");
  Serial.println(sensorPersisted);
}

// loadState attempts to read the BSEC state from the EEPROM. If the state isn't there yet - it wipes that area of the EEPROM ready to be written to in the future. It'll also set the global variable 'sensorPersisted' to a space, so that the question mark disappears forever from the LCD.
void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }
    sensorPersisted = " ";
    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

// updateState waits for the in air quality accuracy to hit '3' - and will then write the state to the EEPROM. Then on every STATE_SAVE_PERIOD, it'll update the state.
void updateState(void)
{
  bool update = false;
  /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
  if (stateUpdateCounter == 0) {
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}
