#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_log.h"

#define SCL 7
#define SDA 6
#define GYRO_POWER 8
#define LED 15

#define ACTIVATION_SECONDS 60
#define uS_TO_S_FACTOR 1000000ULL  
#define SLEEP_SECONDS 1

Adafruit_MPU6050 mpu;

RTC_DATA_ATTR int activation = 0;
RTC_DATA_ATTR bool active = 0;

//Public Key (Set your own)
uint8_t public_key[28] = {
    0x20, 0xB7, 0xA5, 0xCE, 0x10, 0x6D, 0xD9, 0xA8, 0x2D, 0x2E, 0xFA, 0xCA, 0x39, 0x45, 0x15, 0x1D, 
    0xDC, 0x69, 0x2F, 0xE7, 0x3E, 0x0F, 0x27, 0x66, 0x18, 0x6F, 0x43, 0x64,};

// https://github.com/liucoj/OHS-Arduino/blob/main/OpenHS.ino
/** Random device address */
esp_bd_addr_t rnd_addr = { 0xFF, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };

/** Advertisement payload */
uint8_t adv_data[31] = {
    0x1e, /* Length (30) */
    0xff, /* Manufacturer Specific Data (type 0xff) */
    0x4c, 0x00, /* Company ID (Apple) */
    0x12, 0x19, /* Offline Finding type and length */
    0x00, /* State */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, /* First two bits */
    0x00, /* Hint (0x00) */
};

/* https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gap_ble.html#_CPPv420esp_ble_adv_params_t */
static esp_ble_adv_params_t ble_adv_params = {
    // Advertising min interval:
    // Minimum advertising interval for undirected and low duty cycle
    // directed advertising. Range: 0x0020 to 0x4000 Default: N = 0x0800
    // (1.28 second) Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
    .adv_int_min = 0x0021, // 20ms
    // Advertising max interval:
    // Maximum advertising interval for undirected and low duty cycle
    // directed advertising. Range: 0x0020 to 0x4000 Default: N = 0x0800
    // (1.28 second) Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
    .adv_int_max = 0x0021, // 20ms
    // Advertisement type
    .adv_type = ADV_TYPE_NONCONN_IND,
    // Use the random address
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    // All channels
    .channel_map = ADV_CHNL_ALL,
    // Allow both scan and connection requests from anyone.
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  esp_err_t err;

  switch (event) {
  case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
    esp_ble_gap_start_advertising(&ble_adv_params);
    break;

  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    //adv start complete event to indicate adv start successfully or failed
    if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
      Serial.printf("advertising start failed: %s", esp_err_to_name(err));
    } else {
      Serial.printf("advertising has started.");
    }
    break;

  case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
      Serial.printf("adv stop failed: %s", esp_err_to_name(err));
    } else {
      Serial.printf("stop adv successfully");
    }
    break;
  default:
    break;
  }
}

void set_addr_from_key(esp_bd_addr_t addr, uint8_t *public_key) {
  addr[0] = public_key[0] | 0b11000000;
  addr[1] = public_key[1];
  addr[2] = public_key[2];
  addr[3] = public_key[3];
  addr[4] = public_key[4];
  addr[5] = public_key[5];
}

void set_payload_from_key(uint8_t *payload, uint8_t *public_key) {
  /* copy last 22 bytes */
  memcpy(&payload[7], &public_key[6], 22);
  /* append two bits of public key */
  payload[29] = public_key[0] >> 6;
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(GYRO_POWER, OUTPUT);
  Serial.begin(9600);

  if(!active){
    digitalWrite(GYRO_POWER, HIGH);
    Wire.begin(SDA, SCL);

    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
    }
    else{
      Serial.println("MPU6050 Found!");
    }
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);
  }

  esp_sleep_enable_timer_wakeup(SLEEP_SECONDS * uS_TO_S_FACTOR);

  set_addr_from_key(rnd_addr, public_key);
  set_payload_from_key(adv_data, public_key);

  BLEDevice::init("");
  BLEDevice::setCustomGapHandler(esp_gap_cb);

  esp_err_t status;
  if ((status = esp_ble_gap_set_rand_addr(rnd_addr)) != ESP_OK) {
    Serial.printf("Couldn't set random address: %s", esp_err_to_name(status));
  }

  BLEAdvertisementData advertisementData = BLEAdvertisementData();
  advertisementData.setManufacturerData(String((char *)adv_data + 2, sizeof(adv_data) - 2));

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setScanResponse(false);
  pAdvertising->setMaxInterval(0x0C80);
  pAdvertising->setMinInterval(0x0640);
  pAdvertising->setAdvertisementData(advertisementData);

  if(!active && mpu.getMotionInterruptStatus()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Serial.println("Moved");
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    activation = 0;
  }
  else{
    activation += SLEEP_SECONDS;
  } 

  if(!active && activation >= ACTIVATION_SECONDS){
    BLEDevice::startAdvertising();
    digitalWrite(GYRO_POWER, LOW);
    active = 1;
  }
  
  if(active){
    BLEDevice::startAdvertising();
    vTaskDelay(200);
  }

  esp_deep_sleep_start();
}

void loop() {}