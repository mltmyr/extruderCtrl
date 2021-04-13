#ifndef COMMANDER_H__
#define COMMANDER_H__

#include <Arduino.h>

#define MSG_SET_HEATING              'H'
#define MSG_SET_EXTRUSION_SPEED      'X'
#define MSG_READ_TEMPERATURE         'T'
#define MSG_READ_EXTRUSION_SPEED     'E'
#define MSG_BLINK_DEBUG_LED          'B'
#define MSG_START_PERIODIC_MESSAGING 'Y'
#define MSG_STOP_PERIODIC_MESSAGING  'N'



#define EXTRA_BYTES_BUFFER_SIZE 4

typedef union
{
    float    arr_f[EXTRA_BYTES_BUFFER_SIZE/4];
    uint16_t arr_w[EXTRA_BYTES_BUFFER_SIZE/2];
    uint8_t  arr_b[EXTRA_BYTES_BUFFER_SIZE];
} extra_bytes_t;

typedef void (*cmd_handler_void_cb)(void);
typedef void (*cmd_handler_data_cb)(extra_bytes_t*, uint8_t); // (data, data_length)

/* 
 * Usage:
 *   1. Call one of the constructors,
 *   2. Populate handlers,
 *   3. enable communication with enable(),
 *   4. Call process_incomming() periodically.
 */
class Commander
{
public:
    Commander(HardwareSerial* serial_ptr);
    Commander();
    ~Commander();

    void enable();

    void send_msg(uint8_t* buf, int16_t len);
    void process_incomming();

    cmd_handler_data_cb set_extrusion_speed_cb;
    cmd_handler_data_cb set_heating_cb;
    cmd_handler_void_cb read_extrusion_speed_cb;
    cmd_handler_void_cb read_temperature_cb;
    cmd_handler_void_cb enable_extruder_cb;
    cmd_handler_void_cb disable_extruder_cb;
    cmd_handler_void_cb blink_debug_led_cb;
    cmd_handler_void_cb start_periodic_messaging_cb;
    cmd_handler_void_cb stop_periodic_messaging_cb;

private:
    void init_members();
    void execute_cmd();
    void process_byte();

    HardwareSerial* sr;

    boolean enabled;

    uint8_t       cmd_code;
    extra_bytes_t extra_data;

    uint8_t numBytesRcvd;
    uint8_t numBytesTot;

    int16_t sumBytesRcvd;
    int16_t timesInvalidCmdRcvd;
};

#endif /* COMMANDER_H__ */
