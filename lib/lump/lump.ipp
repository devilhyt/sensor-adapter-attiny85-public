#include "lump.h"

template <typename T>
LUMPSensor<T>::LUMPSensor(T *uart, u8_t rx_pin, u8_t tx_pin, u8_t type, u32_t speed, LUMPMode *modes, u8_t mode_num, u8_t view, bool with_xtal)
    : uart{uart}, rx_pin{rx_pin}, tx_pin{tx_pin}, with_xtal{with_xtal}, type{type}, mode_num{mode_num}, view{view}, speed{speed}, modes{modes} {}

template <typename T>
void LUMPSensor<T>::set_sensor_mode_init_callback(void (*sensor_mode_init)()) {
  sensor_mode_init_callback = sensor_mode_init;
}

template <typename T>
void LUMPSensor<T>::set_sensor_running_callback(void (*sensor_running)()) {
  sensor_running_callback = sensor_running;
}

template <typename T>
void LUMPSensor<T>::run() {
  u32_t current_ms{millis()};

  /* Limit the sensor frequency to below 1000 Hz */
  if (current_ms == last_ms)
    return;
  last_ms = current_ms;

  transmit();
  receive();
}

template <typename T>
void LUMPSensor<T>::transmit() {
  memset(tx_buf, 0, sizeof(tx_buf));

  /* Sensor state machine */
  switch (sensor_state) {
    /* Disable UART pins to allow the host to detect this sensor using autoid */
    case SensorReset:
      wdt_reset();
      uart->end();
      uart_pin_setup();
      event_ms     = last_ms;
      sensor_state = SensorWaitForAutoId;
      break;

    /* Wait until detection is done; then enable UART and start the handshake */
    case SensorWaitForAutoId:
      if (last_ms - event_ms > AUTOID_DELAY) {
        wdt_reset();
        uart->begin(LOWEST_BITRATE);
        event_ms      = last_ms;
        sync_attempts = 0;
        sensor_state  = SensorHandshake;
        hsk_state     = HskSync;
      }
      break;

    /* Handshake */
    case SensorHandshake:
      /* Handshake state machine */
      switch (hsk_state) {
        case HskSync:
          // hidden
          break;
        /* Wait for sync from host
         * This state is exited through from the command processing code in the receive() function.
         */
        case HskWaitForSync:
          if (last_ms - event_ms > DELAY_BETWEEN_SYNCS) {
            if (++sync_attempts >= MAX_SYNC_ATTEMPTS)
              sensor_state = SensorReset;
            else
              hsk_state = HskSync;
          }
          break;

        case HskId:
          // hidden
          break;

        case HskModes:
          // hidden
          break;

        case HskSpeed:
          // hidden
          break;

        case HskModesName:
          wdt_reset();
          hsk_mode_ptr = &modes[hsk_mode_index];
          hsk_mode_name_symbol(hsk_mode_ptr->name, INFO_NAME);
          hsk_state = HskModesRaw;
          break;

        case HskModesRaw:
          hsk_mode_value(hsk_mode_ptr->raw, INFO_RAW);
          hsk_state = HskModesPct;
          break;

        case HskModesPct:
          hsk_mode_value(hsk_mode_ptr->pct, INFO_PCT);
          hsk_state = HskModesSi;
          break;

        case HskModesSi:
          hsk_mode_value(hsk_mode_ptr->si, INFO_SI);
          hsk_state = HskModesSymbol;
          break;

        case HskModesSymbol:
          hsk_mode_name_symbol(hsk_mode_ptr->symbol, INFO_SYMBOL);
          hsk_state = HskModesFormat;
          break;

        case HskModesFormat:
          // hidden
          break;

        case HskModesPause:
          if (last_ms - event_ms > INTERMODE_PAUSE) {
            if (++hsk_mode_index < mode_num) { /* equal to "hsk_mode_index++ < mode_num - 1" */
              hsk_state = HskModesName;
            } else {
              hsk_state = HskAck;
            }
          }
          break;

        case HskAck:
          // hidden
          break;

        /* Wait for ACK reply
         * This state is exited through from the command processing code in the receive() function.
         */
        case HskWaitForAck:
          if (last_ms - event_ms > ACK_TIMEOUT)
            sensor_state = SensorReset;
          break;

        /* Switch to high baudrate & data mode */
        case HskSwitchBaudRate:
          wdt_reset();
          uart->end();
          uart->begin(speed);
          sensor_state = SensorModeInit;
          break;

        default:
          while (true)
            ; /* Error occurred. Wait for watchdog reset the sensor. */
          break;
      }
      break;

    case SensorModeInit:
      if (sensor_mode_init_callback)
        sensor_mode_init_callback();
      sensor_state = SensorRunning;
      break;

    case SensorRunning:
      if (sensor_running_callback)
        sensor_running_callback();
      break;

    case SensorNack:
      // hidden
      break;

    default:
      while (true)
        ; /* Error occurred. Wait for watchdog reset the sensor. */
      break;
  }
}

/**
 * @brief Receive data from the host
 *
 * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
 */
template <typename T>
void LUMPSensor<T>::receive() {
  u8_t byte_received{0};
  receive_state = ReceiveGetByte;
  while (true) {
    switch (receive_state) {
      case ReceiveGetByte: {
        if (!uart->available())
          return;
        byte_received = uart->read();
        if (rx_len == 0)
          receive_state = ReceiveCheckMsgType;
        else
          receive_state = ReceiveDecode;
        break;
      }
      case ReceiveCheckMsgType: {
        // hidden
        break;
      }
      case ReceiveDecode: {
        rx_buf[rx_index++] = byte_received;
        if (rx_index >= rx_len) {
          rx_len        = 0;
          receive_state = ReceiveChecksum;
        } else {
          rx_checksum ^= byte_received; /* Calc checksum */
          receive_state = ReceiveGetByte;
        }
        break;
      }
      case ReceiveChecksum: {
        if (rx_checksum == byte_received) {
          receive_state = RecevieOperate;
        } else if (sensor_state == SensorRunning) {
          sensor_state = SensorNack;
          return;
        } else {
          receive_state = ReceiveGetByte;
        }
        break;
      }
      case RecevieOperate: {
        // hidden
        return;
      }
      default:
        while (true)
          ; /* Error occurred. Wait for watchdog reset the sensor. */
        break;
    }
  }
}

template <typename T>
void LUMPSensor<T>::hsk_mode_name_symbol(char *ptr, u8_t info_type) {
  // hidden
}

template <typename T>
void LUMPSensor<T>::hsk_mode_value(LUMPValue *ptr, u8_t info_type) {
  // hidden
}

template <typename T>
u8_t LUMPSensor<T>::msg_encode(u8_t msg_bit, u8_t msg_len, u8_t cmd_bit) {
  return msg_bit | (log2(msg_len) << MSGLEN_SHIFT) | cmd_bit;
}

template <typename T>
u8_t LUMPSensor<T>::msg_checksum(u8_t *dst, u8_t msg_len) {
  u8_t checksum{0xff};
  for (u8_t i = 0; i < msg_len; ++i)
    checksum ^= dst[i];
  return checksum;
}

template <typename T>
u8_t LUMPSensor<T>::log2(u8_t x) {
  switch (x) {
    case 1:
      return 0;
    case 2:
      return 1;
    case 4:
      return 2;
    case 8:
      return 3;
    case 16:
      return 4;
    case 32:
      return 5;
    default:
      return 0;
  }
}

template <typename T>
u8_t LUMPSensor<T>::exp2(u8_t x) {
  switch (x) {
    case 0:
      return 1;
    case 1:
      return 2;
    case 2:
      return 4;
    case 3:
      return 8;
    case 4:
      return 16;
    case 5:
      return 32;
    default:
      return 64;
  }
}

template <typename T>
u8_t LUMPSensor<T>::ceil_power2(u8_t x) {
  if (x == 1 || x == 2)
    return x;
  else if (x <= 4)
    return 4;
  else if (x <= 8)
    return 8;
  else if (x <= 16)
    return 16;
  else if (x <= 32)
    return 32;
  else
    return 0;
}

template <typename T>
u8_t LUMPSensor<T>::get_current_mode() {
  return current_mode;
}

template <typename T>
sensor_state_t LUMPSensor<T>::get_sensor_state() {
  return sensor_state;
}

template <typename T>
bool LUMPSensor<T>::get_nack_force_send() {
  return force_send;
}

template <typename T>
void LUMPSensor<T>::uart_pin_setup() {
  pinMode(rx_pin, INPUT);
  pinMode(tx_pin, OUTPUT);
  digitalWrite(tx_pin, LOW); /* Ground TX Pin */
}

template <typename T>
void LUMPSensor<T>::uart_transmit(u8_t *msg, u8_t msg_len) {
  uart->write(msg, msg_len);
}

template <typename T>
void LUMPSensor<T>::uart_transmit(u8_t msg) {
  uart->write(msg);
}

template <typename T>
u8_t LUMPSensor<T>::read_byte() {
  while (!uart->available())
    ; /* Wait for data */
  return uart->read();
}

template <typename T>
void LUMPSensor<T>::send_data8(u8_t b) {
  send_data8(&b, 1);
}

template <typename T>
void LUMPSensor<T>::send_data8(u8_t *b, u8_t len) {
  // hidden
}

template <typename T>
void LUMPSensor<T>::send_data16(u16_t s) {
  send_data8(reinterpret_cast<u8_t *>(&s), 2);
}

template <typename T>
void LUMPSensor<T>::send_data16(u16_t *s, u8_t len) {
  send_data8(reinterpret_cast<u8_t *>(s), len * 2);
}

template <typename T>
void LUMPSensor<T>::send_data32(u32_t l) {
  send_data8(reinterpret_cast<u8_t *>(&l), 4);
}

template <typename T>
void LUMPSensor<T>::send_data32(u32_t *l, u8_t len) {
  send_data8(reinterpret_cast<u8_t *>(l), len * 4);
}
template <typename T>
void LUMPSensor<T>::send_dataf(f32_t f) {
  send_data8(reinterpret_cast<u8_t *>(&f), 4);
}

template <typename T>
void LUMPSensor<T>::send_dataf(f32_t *f, u8_t len) {
  send_data8(reinterpret_cast<u8_t *>(f), len * 4);
}
