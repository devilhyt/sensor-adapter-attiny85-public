#include <Arduino.h>
#include <SoftwareSerial.h>
#include <lump.h>

LUMPValue value_0_1023{.low = 0, .high = 1023};
LUMPValue value_0_100{.low = 0, .high = 100};
LUMPValue value_0_1{.low = 0, .high = 1};

LUMPMode modes[]{
    {"Analog", "raw", 1, DATA16, 5, 0, &value_0_1023, &value_0_100, &value_0_1023},
    {"Digital", "raw", 1, DATA8, 1, 0, &value_0_1, &value_0_100, &value_0_1},
    {"Up time", "sec", 1, DATA32, 5, 0, &value_0_1023, &value_0_100, &value_0_1023}};
u8_t mode_num{sizeof(modes) / sizeof(LUMPMode)};

#ifndef ATTINY_CORE
LUMPSensor<HardwareSerial> sensor(&Serial, 0, 1, 83, 57600, modes, mode_num, VIEW_DEFAULT, true);
#else
SoftwareSerial ss(1, 0);
LUMPSensor<SoftwareSerial> sensor(&ss, 1, 0, 83, 57600, modes, mode_num, VIEW_DEFAULT, true);
#endif

void sensor_mode_init() {
  switch (sensor.get_current_mode()) {
    case 0:
      pinMode(A1, INPUT);
      break;
    case 1:
      pinMode(A1, INPUT);
      break;
    default:
      break;
  }
}

void sensor_running() {
  static u16_t new_value_0  = 0;
  static u16_t last_value_0 = 0;
  static u8_t new_value_1   = 0;
  static u8_t last_value_1  = 0;
  static u32_t new_value_2  = 0;
  static u32_t last_value_2 = 0;
  static bool force_send    = false;
  static u32_t last_millis  = 0;
  static u32_t period       = 8; /* 125Hz */
  u32_t current_millis      = millis();

  switch (sensor.get_current_mode()) {
    case 0:
      new_value_0 = analogRead(A1);
      if (new_value_0 != last_value_0) {
        force_send   = true;
        last_value_0 = new_value_0;
      }
      break;
    case 1:
      new_value_1 = digitalRead(A1);
      if (new_value_1 != last_value_1) {
        force_send   = true;
        last_value_1 = new_value_1;
      }
      break;
    case 2:
      new_value_2 = millis() / 1000;
      if (new_value_2 != last_value_2) {
        force_send   = true;
        last_value_2 = new_value_2;
      }
      break;
    default:
      break;
  }

  if (sensor.get_nack_force_send() || (force_send && (current_millis - last_millis > period))) {
    switch (sensor.get_current_mode()) {
      case 0:
        sensor.send_data16(new_value_0);
        break;
      case 1:
        sensor.send_data8(new_value_1);
        break;
      case 2:
        sensor.send_data32(new_value_2);
        break;
      default:
        break;
    }
    force_send  = false;
    last_millis = current_millis;
  }
}

void setup() {
  wdt_reset();
#ifndef ATTINY_CORE
  wdt_enable(WDTO_2S);
#else
  wdt_enable(WDTO_4S);
#endif
  sensor.set_sensor_mode_init_callback(&sensor_mode_init);
  sensor.set_sensor_running_callback(&sensor_running);
}

void loop() {
  while (true) {
    sensor.run();
  }
}
