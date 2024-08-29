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
          uart_transmit(SYS_SYNC);
          if (with_xtal) {
            hsk_state = HskId;
          } else {
            event_ms  = last_ms;
            hsk_state = HskWaitForSync;
          }
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
          tx_buf[0] = msg_encode(MSG_CMD, 1, CMD_TYPE);
          tx_buf[1] = type;
          tx_buf[2] = msg_checksum(tx_buf, 2);
          uart_transmit(tx_buf, 3);
          hsk_state = HskModes;
          break;

        case HskModes:
          tx_buf[1] = mode_num - 1;
          if (view == VIEW_DEFAULT) {
            tx_buf[0] = msg_encode(MSG_CMD, 1, CMD_MODES);
            tx_buf[2] = msg_checksum(tx_buf, 2);
            uart_transmit(tx_buf, 3);
          } else {
            tx_buf[0] = msg_encode(MSG_CMD, 2, CMD_MODES);
            tx_buf[2] = view - 1;
            tx_buf[3] = msg_checksum(tx_buf, 3);
            uart_transmit(tx_buf, 4);
          }
          hsk_state = HskSpeed;
          break;

        case HskSpeed:
          tx_buf[0] = msg_encode(MSG_CMD, 4, CMD_SPEED);
          memcpy(&tx_buf[1], &speed, 4);
          tx_buf[5] = msg_checksum(tx_buf, 5);
          uart_transmit(tx_buf, 6);
          hsk_mode_ptr   = nullptr;
          hsk_mode_index = 0;
          hsk_state      = HskModesName;
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
          tx_buf[0] = msg_encode(MSG_INFO, 4, hsk_mode_index);
          tx_buf[1] = INFO_FORMAT;
          tx_buf[2] = hsk_mode_ptr->data_sets;
          tx_buf[3] = hsk_mode_ptr->data_type;
          tx_buf[4] = hsk_mode_ptr->figures;
          tx_buf[5] = hsk_mode_ptr->decimals;
          tx_buf[6] = msg_checksum(tx_buf, 6);
          uart_transmit(tx_buf, 7);
          event_ms  = last_ms;
          hsk_state = HskModesPause;
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
          uart_transmit(SYS_ACK);
          event_ms  = last_ms;
          hsk_state = HskWaitForAck;
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
      uart_transmit(SYS_NACK);
      sensor_state = SensorRunning;
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
        if (byte_received == SYS_SYNC || byte_received == SYS_NACK || byte_received == SYS_ACK) { /* System messages */
          rx_buf[0]     = byte_received;
          receive_state = RecevieOperate;
        } else { /* Other types of messages */
          rx_len = exp2((byte_received & MSG_LEN_MASK) >> MSGLEN_SHIFT) + 2;
          if (rx_len <= MAX_MSG_LEN) {
            rx_index      = 0;
            rx_checksum   = 0xFF;
            receive_state = ReceiveDecode;
          } else { /* Payload length too long. look for next message */
            rx_len        = 0;
            receive_state = ReceiveGetByte;
          }
        }
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
        if (rx_buf[0] == SYS_NACK) {
          wdt_reset();
          force_send = true;
          return;
        }

        switch (sensor_state) {
          /* Handle host messages during handshake */
          case SensorHandshake: {
            if (hsk_state == HskWaitForSync && rx_buf[0] == SYS_SYNC)
              hsk_state = HskId; /* Sync sucess, start handshake */
            if (hsk_state == HskWaitForAck && rx_buf[0] == SYS_ACK)
              hsk_state = HskSwitchBaudRate; /* Ack received, start data mode */
            break;
          }
          /* handle host messages in running condition */
          case SensorRunning: {
            if (rx_buf[0] == (MSG_CMD | CMD_SELECT)) { /* Select mode */
              current_mode = rx_buf[1];
              sensor_state = SensorModeInit;
            }
            break;
          }
          default:
            break;
        }
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
  u8_t len = strlen(ptr);
  memcpy(&tx_buf[2], ptr, len); // No zero termination necessary
  len             = ceil_power2(len);
  tx_buf[0]       = msg_encode(MSG_INFO, len, hsk_mode_index);
  tx_buf[1]       = info_type;
  tx_buf[len + 2] = msg_checksum(tx_buf, len + 2);
  uart_transmit(tx_buf, len + 3);
}

template <typename T>
void LUMPSensor<T>::hsk_mode_value(LUMPValue *ptr, u8_t info_type) {
  if (ptr) {
    tx_buf[0] = msg_encode(MSG_INFO, 8, hsk_mode_index);
    tx_buf[1] = info_type;
    memcpy(&tx_buf[2], &(ptr->low), 4);
    memcpy(&tx_buf[6], &(ptr->high), 4);
    tx_buf[10] = msg_checksum(tx_buf, 10);
    uart_transmit(tx_buf, 11);
  }
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
  tx_buf[0] = msg_encode(MSG_DATA, len, current_mode);
  memcpy(&tx_buf[1], b, len);
  tx_buf[len + 1] = msg_checksum(tx_buf, len + 1);
  uart_transmit(tx_buf, len + 2);
  force_send = false;
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
