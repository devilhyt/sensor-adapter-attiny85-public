/**
 * @file lump.h
 * @author DevilHYT (devilhyt@gmail.com)
 * @brief LEGO UART Message Protocol
 * @version 0.1
 * @date 2022-02-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef LUMP_H
#define LUMP_H

#include <Arduino.h>
#include <avr/wdt.h>
#include <lumpTypedef.h>

/* Message type */
#define MSG_SYS  0x00
#define MSG_CMD  0x40
#define MSG_INFO 0x80
#define MSG_DATA 0xC0

/* System message type */
#define SYS_SYNC 0x00
#define SYS_NACK 0x02
#define SYS_ACK  0x04
#define SYS_ESC  0x06

/* Command message type */
#define CMD_TYPE     0x00
#define CMD_MODES    0x01
#define CMD_SPEED    0x02
#define CMD_SELECT   0x03
#define CMD_WRITE    0x04
#define CMD_UNK1     0x05 // Powered Up only
#define CMD_EXT_MODE 0x06
#define CMD_VERSION  0x07

/* Info message type */
#define INFO_NAME        0x00
#define INFO_RAW         0x01
#define INFO_PCT         0x02
#define INFO_SI          0x03
#define INFO_SYMBOL      0x04
#define INFO_MAPPING     0x05 /* Powered Up only */
#define INFO_MODE_COMBOS 0x06 /* Powered Up only */
#define INFO_UNK7        0x07 /* Powered Up only */
#define INFO_UNK8        0x08 /* Powered Up only */
#define INFO_UNK9        0x09 /* Powered Up only */
#define INFO_UNK10       0x0A /* Powered Up only */
#define INFO_UNK11       0x0B /* Powered Up only */
#define INFO_UNK12       0x0C /* Powered Up only */
#define INFO_MODE_PLUS_8 0x20
#define INFO_FORMAT      0x80

/* Encoded message payload bytes */
#define MSG_LEN_1  0x00
#define MSG_LEN_2  0x08
#define MSG_LEN_4  0x10
#define MSG_LEN_8  0x18
#define MSG_LEN_16 0x20
#define MSG_LEN_32 0x28

/* Number of modes in view and data log */
#define VIEW_1       0x01 /* Only mode 0 */
#define VIEW_2       0x02 /* Mode 0,1 */
#define VIEW_3       0x03 /* Mode 0,1,2 */
#define VIEW_4       0x04 /* Mode 0,1,2,3 */
#define VIEW_5       0x05 /* Mode 0,1,2,3,4 */
#define VIEW_6       0x06 /* Mode 0,1,2,3,4,5 */
#define VIEW_7       0x07 /* Mode 0,1,2,3,4,5,6 */
#define VIEW_8       0x08 /* Mode 0,1,2,3,4,5,6,7 */
#define VIEW_DEFAULT 0xFF /* All modes */

/* Bit mask */
#define MSG_MASK       0xC0
#define SYS_MASK       0x07
#define CMD_MASK       0x07
#define INFO_MODE_MASK 0x07
#define MSG_LEN_MASK   0x38

/* Bit Shift */
#define MSGLEN_SHIFT 3

/* Data set format */
#define DATA8  0x00
#define DATA16 0x01
#define DATA32 0x02
#define DATAF  0x03

/* Timeout threshold */
#define AUTOID_DELAY        500 /* ms */
#define DELAY_BETWEEN_SYNCS 6   /* ms */
#define MAX_SYNC_ATTEMPTS   10
#define INTERMODE_PAUSE     30  /* ms */
#define ACK_TIMEOUT         500 /* ms. Default is 80ms*/

/* Misc */
#define LOWEST_BITRATE 2400
#define MAX_MODES      8
#define MAX_MSG_LEN    32
#define BUF_SIZE       35
#define DEFAULT_VALUE  nullptr

/* Sensor state */
typedef enum {
  SensorReset,
  SensorWaitForAutoId,
  SensorHandshake,
  SensorModeInit,
  SensorRunning,
  SensorNack,
} sensor_state_t;

/* Handshake state */
typedef enum {
  HskSync,
  HskWaitForSync,
  HskId,
  HskModes,
  HskSpeed,
  HskModesName,
  HskModesRaw,
  HskModesPct,
  HskModesSi,
  HskModesSymbol,
  HskModesFormat,
  HskModesPause,
  HskAck,
  HskWaitForAck,
  HskSwitchBaudRate,
} hsk_state_t;

/* Receive state */
typedef enum {
  ReceiveGetByte,
  ReceiveCheckMsgType,
  ReceiveDecode,
  ReceiveChecksum,
  RecevieOperate,
} receive_state_t;

/* LUMP Value */
class LUMPValue {
  public:
    f32_t low;
    f32_t high;
};

/* LUMP mode */
class LUMPMode final{
  /* aggregate class */
  public:
    LUMPMode() = default;
    ~LUMPMode() = default;
    LUMPMode(const LUMPMode&) = default;
    LUMPMode(LUMPMode&&) = default;
    LUMPMode& operator=(const LUMPMode&) = default;
    LUMPMode& operator=(LUMPMode&&) = default;

    char name[12];
    char symbol[5];
    u8_t data_sets;
    u8_t data_type;
    u8_t figures;
    u8_t decimals;
    LUMPValue *raw;
    LUMPValue *pct;
    LUMPValue *si;
};

/* LUMP Sensor */
template <typename T>
class LUMPSensor final{
  public:
    LUMPSensor() = default;
    ~LUMPSensor() = default;
    LUMPSensor(const LUMPSensor&) = default;
    LUMPSensor(LUMPSensor&&) = default;
    LUMPSensor& operator=(const LUMPSensor&) = default;
    LUMPSensor& operator=(LUMPSensor&&) = default;
    /**
     * @brief Create a sensor
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param uart Serial object
     * @param rx_pin RX pin
     * @param tx_pin TX pin
     * @param type Sensor type (AutoID)
     * @param speed Baud rate
     * @param modes Sensor modes (LUMPMode)
     * @param mode_num Number of modes
     * @param view How many modes to show in the host (VIEW_DEFAULT: all modes)
     * @param with_xtal Whether the sensor has an external crystal
     */
    LUMPSensor(T *uart, u8_t rx_pin, u8_t tx_pin, u8_t type, u32_t speed, LUMPMode *modes, u8_t mode_num, u8_t view = VIEW_DEFAULT, bool with_xtal = true);

    /**
     * @brief
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param sensor_mode_init callback function for mode initialization
     */
    void set_sensor_mode_init_callback(void (*sensor_mode_init)());

    /**
     * @brief
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param sensor_running callback function for sensor running
     */
    void set_sensor_running_callback(void (*sensor_running)());

    /**
     * @brief Run lump sensor
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     */
    void run();

    /**
     * @brief  Transmit data to the host
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     */
    void transmit();

    /**
     * @brief Receive data from the host
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     */
    void receive();

    /**
     * @brief Get the current state of the sensor
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @return sensor_state_t Current state
     */
    sensor_state_t get_sensor_state();

    /**
     * @brief Get the current mode of the sensor
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @return u8_t Current mode
     */
    u8_t get_current_mode();

    /**
     * @brief Get the force send flag
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @return bool Force send flag
     * @retval true Force send
     * @retval false No force send
     * @note This function will reset the force send flag
     */
    bool get_nack_force_send();

    /////////////////////////// Send messages functions ///////////////////////////

    /**
     * @brief Send a DATA8 message to the host.
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param b Data to send
     */
    void send_data8(u8_t b);

    /**
     * @brief Send DATA8 messages to the host.
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param b Data to send
     * @param len Number of messages to send
     */
    void send_data8(u8_t *b, u8_t len);

    /**
     * @brief Send a DATA16 message to the host.
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param s Data to send
     */
    void send_data16(u16_t s);

    /**
     * @brief Send DATA16 messages to the host.
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param s Data to send
     * @param len Number of messages to send
     */
    void send_data16(u16_t *s, u8_t len);

    /**
     * @brief Send a DATA32 message to the host.
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param l Data to send
     */
    void send_data32(u32_t l);

    /**
     * @brief Send DATA32 messages to the host.
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param l Data to send
     * @param len Number of messages to send
     */
    void send_data32(u32_t *l, u8_t len);

    /**
     * @brief Send a DATAF message to the host.
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param f Data to send
     */
    void send_dataf(f32_t f);

    /**
     * @brief Send DATAF messages to the host.
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param f Data to send
     * @param len Number of messages to send
     */
    void send_dataf(f32_t *f, u8_t len);

  protected:
    /* UART */
    T *uart;
    u8_t rx_pin;
    u8_t tx_pin;

    /* Sensor info */
    bool with_xtal;
    u8_t type;
    u8_t mode_num;
    u8_t view;
    u32_t speed;
    LUMPMode *modes; /* An array of LUMPMode objects */

    /* Sensor mode */
    u8_t hsk_mode_index;
    LUMPMode *hsk_mode_ptr;
    u8_t current_mode{0};

    /* State machine */
    sensor_state_t sensor_state{SensorReset};
    hsk_state_t hsk_state{HskSync};
    receive_state_t receive_state;

    /* timing */
    u32_t last_ms;
    u32_t event_ms;

    /* LUMP TX */
    u8_t tx_buf[BUF_SIZE];
    u8_t rx_buf[BUF_SIZE];

    /* LUMP RX */
    u8_t rx_len{0};
    u8_t rx_index{0};
    u8_t rx_checksum{0xFF};

    /* Callback functions */
    void (*sensor_mode_init_callback)() = nullptr; /* Called when the sensor mode is changed */
    void (*sensor_running_callback)()   = nullptr; /* called when the sensor is running */

    /* MISC */
    u8_t sync_attempts;
    bool force_send{false};

    ////////////////////functions////////////////////

    /**
     * @brief Calculate checksum of a message
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param dst pointer to the message
     * @param msg_len length of the message
     * @return u8_t checksum
     */
    static u8_t msg_checksum(u8_t *dst, u8_t msg_len);

    /**
     * @brief Intergate message type, length and command
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param msg_bit message type
     * @param msg_len message length
     * @param cmd_bit command type
     * @return u8_t intergated message
     */
    static u8_t msg_encode(u8_t msg_bit, u8_t msg_len, u8_t cmd_bit);

    /**
     * @brief Utility function to send a mode's name or mode's symbol
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param ptr Pointer to the name string or symbol string
     * @param info_type Info type (INFO_NAME, INFO_SYMBOL)
     */
    void hsk_mode_name_symbol(char *ptr, u8_t info_type);

    /**
     * @brief Utility function to send a mode's value span
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param ptr Pointer to the value span(LUMPValue)
     * @param info_type Info type (INFO_RAW, INFO_PCTG, INFO_SI)
     */
    void hsk_mode_value(LUMPValue *value_ptr, u8_t info_type);

    /**
     * @brief Setup the UART pins
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     */
    void uart_pin_setup();

    /**
     * @brief Transmit a message over UART
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param msg the message to transmit
     * @param msg_len length of the message
     */
    void uart_transmit(u8_t *msg, u8_t msg_len);

    /**
     * @brief Transmit a message over UART
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param msg the message to transmit
     */
    void uart_transmit(u8_t msg);

    /**
     * @brief Utility method to read a byte synchronously
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @return u8_t The byte read
     */
    u8_t read_byte();

    /**
     * @brief Utility method to compute binary logarithm (up to 32)
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param x Value to compute the binary logarithm
     * @return u8_t x's binary logarithm
     */
    static u8_t exp2(u8_t val);

    /**
     * @brief Utility method to compute power of 2 (up to 5)
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param x Value to compute the power of 2
     * @return u8_t x power of 2
     */
    static u8_t log2(u8_t val);

    /**
     * @brief Utility method to compute next power of 2 (up to 32)
     *
     * @tparam T Serial Type (HardwareSerial, SoftwareSerial)
     * @param x Value to compute the next power of 2
     * @return u8_t x's next power of 2
     */
    static u8_t ceil_power2(u8_t val);
};

#include "lump.ipp"

#endif /* LUMP_H */