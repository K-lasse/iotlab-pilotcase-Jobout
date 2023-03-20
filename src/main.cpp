/*
 * HelTec Automation(TM) LoRaWAN 1.0.2 OTAA example use OTAA, CLASS A
 *
 * Function summary:
 *
 * - use internal RTC(150KHz);
 *
 * - Include stop mode and deep sleep mode;
 *
 * - 15S data send cycle;
 *
 * - Informations output via serial(115200);
 *
 * - Only ESP32 + LoRa series boards can use this library, need a license
 *   to make the code run(check you license here: http://www.heltec.cn/search/);
 *
 * You can change some definition in "Commissioning.h" and "LoRaMac-definitions.h"
 *
 * HelTec AutoMation, Chengdu, China.
 * 成都惠利特自动化科技有限公司
 * https://heltec.org
 * support@heltec.cn
 *
 *this project also release in GitHub:
 *https://github.com/HelTecAutomation/ESP32_LoRaWAN
 */

#include "Arduino.h"
#include <TTN_esp32.h>
#include <stdio.h>

/*
  Deep sleep
*/
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 15          /* Time ESP32 will go to sleep (in seconds) */

/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t license[4] = {0xD5397DF0, 0x8573F814, 0x7A38C73D, 0x48E68607};

// Array of bits where each bit represent every 5 minutes of a day.
// With this it enables us to store and manipulate 288 individual bits (9 elements x 32 bits per element).
RTC_DATA_ATTR uint32_t data[9] = {0};
RTC_DATA_ATTR int element_index = 8;
RTC_DATA_ATTR int bit_index = 30;
RTC_DATA_ATTR bool sent = false;
uint32_t *ptr = &data[element_index];
RTC_DATA_ATTR uint8_t appData[36];
RTC_DATA_ATTR uint8_t voltage = 0;

const int GPIO_WAKEUP_PIN = GPIO_NUM_33;
const int GPIO_BATTERY_PIN = GPIO_NUM_35;

#define INTERVAL 30000
TTN_esp32 ttn;

TaskHandle_t joinLoRaHandler = NULL;
TaskHandle_t batteryHandler = NULL;
bool batteryMeasured = false;

// End device ID: eui-70b3d57ed005a424
// AppKEY: B6D065CF1800017D25DB531118B3C202
// DevEUI: 70B3D57ED005A424

/* OTAA para*/
const char *DevEui = "70B3D57ED005A424";
const char *AppEui = "0000000000000101";
const char *AppKey = "B6D065CF1800017D25DB531118B3C202";

void print_bits(uint32_t num, int num_bits) {
  for (int i = num_bits - 1; i >= 0; i--) {
    bool bit = (num >> i) & 1;
    Serial.print(bit ? "1" : "0");
  }
  Serial.println();
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
  // Wakeup caused by timer
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");

    // Set the GPIO pin mode to input
    pinMode(GPIO_WAKEUP_PIN, INPUT);

    // Check value of GPIO_WAKEUP_PIN and update data array
    if (digitalRead(GPIO_WAKEUP_PIN) == 1) {
      Serial.println("GPIO_WAKEUP_PIN is HIGH");

      // SET BIT TO 1
      *ptr |= (1 << bit_index);
    } else {
      Serial.println("GPIO_WAKEUP_PIN is LOW");
    }

    bit_index++;

    // print the array
    for (int i = 0; i < 9; i++) {
      print_bits(data[i], 32);
    }

    break;

  // Wakeup not caused by deep sleep
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

// Add your initialization code here
void setup() {
  // Start the serial communication with a baud rate of 115200
  Serial.begin(115200);
  Serial.println("Starting ...!");

  // Reset the value of the data array if the data has been sent.
  if (sent) {
    for (int i = 0; i < 9; i++) {
      data[i] = 0;
    }
    element_index = 0;
    bit_index = 0;
    ptr = &data[element_index];
    sent = false;
  }

  print_wakeup_reason();
  Serial.println("LoRaWan AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH");

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void prepareData() {
  int appDataSize = 36;
  uint8_t *p = (uint8_t *)&data;
  for (int i = 0; i < appDataSize; i++) {
    appData[i] = *p;
    p++;
  }
}

void joinLoRa(void *pvParameters) {
  ttn.join(DevEui, AppEui, AppKey);
  vTaskDelay(30000);

  while (!ttn.isJoined()) {
    ttn.join(DevEui, AppEui, AppKey);
    vTaskDelay(25000);
  }

  Serial.println("Joined");
}

void measureBattery(void *pvParameters) {
  // Measure battery voltage
  int adcValue = analogRead(GPIO_BATTERY_PIN);
  voltage = (adcValue) / 4095.0 * 3300.0;
  Serial.print("Battery voltage: ");
  Serial.println(voltage);

  batteryMeasured = true;
  vTaskDelete(batteryHandler);
}

// The loop function is called in an endless loop
void loop() {
  if (bit_index == 32 && element_index == 8) {
    Serial.println("Resetting indexes");
    ttn.begin();

    int numTasks = uxTaskGetNumberOfTasks();
    Serial.println(numTasks);
    xTaskCreate(
        joinLoRa,          /* Task function. */
        "ttn",             /* String with name of task. */
        10000,             /* Stack size in bytes. */
        NULL,              /* Parameter passed as input of the task */
        1,                 /* Priority of the task. */
        &joinLoRaHandler); /* Task handle. */

    xTaskCreate(
        measureBattery,   /* Task function. */
        "battery",        /* String with name of task. */
        10000,            /* Stack size in bytes. */
        NULL,             /* Parameter passed as input of the task */
        1,                /* Priority of the task. */
        &batteryHandler); /* Task handle. */

    while (uxTaskGetNumberOfTasks() > numTasks) {
      if (ttn.isJoined() && batteryMeasured == true) {
        vTaskDelete(joinLoRaHandler);
        break;
      }
      Serial.println(uxTaskGetNumberOfTasks());
    }
    Serial.println(uxTaskGetNumberOfTasks());
    prepareData();
    ttn.sendBytes(appData, sizeof(appData));
    // ttn.sendBytes(voltage, sizeof(voltage));
    Serial.println("Data sent");
    sent = true;
  }

  else if (bit_index == 32 && element_index < 8) {
    bit_index = 0;
    element_index++;
    ptr = &data[element_index];
  }

  // put your main code here, to run repeatedly:
  Serial.println("Loop ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
}