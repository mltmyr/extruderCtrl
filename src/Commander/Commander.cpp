#include "Commander.h"

void default_handler_void() {return;}
void default_handler_data(extra_bytes_t* data, uint8_t len) {return;}

void Commander::init_members()
{
    this->set_extrusion_speed_cb      = default_handler_data;
    this->set_heating_cb              = default_handler_data;
    this->read_extrusion_speed_cb     = default_handler_void;
    this->read_temperature_cb         = default_handler_void;
    this->blink_debug_led_cb          = default_handler_void;
    this->start_periodic_messaging_cb = default_handler_void;
    this->stop_periodic_messaging_cb  = default_handler_void;

    this->enabled = false;

    this->numBytesRcvd = 0;
    this->numBytesTot  = 1;

    this->cmd_code = 0x00;
    for (uint16_t i = 0; i < EXTRA_BYTES_BUFFER_SIZE; i++) {this->extra_data.arr_b[i] = 0x00;}

    this->sumBytesRcvd = 0;
    this->timesInvalidCmdRcvd = 0;

    return;
}

Commander::Commander(HardwareSerial* serial_ptr)
{
    this->sr = serial_ptr;

    this->init_members();
}

#define STD_SERIAL              Serial
#define STD_SERIAL_BAUD_RATE    9600
#define STD_SERIAL_CONFIG       SERIAL_8N1

Commander::Commander()
{
    this->sr = &(STD_SERIAL);
    this->sr->begin(STD_SERIAL_BAUD_RATE, STD_SERIAL_CONFIG);

    this->init_members();
}

Commander::~Commander()
{
    this->process_incomming();
    this->sr->flush();
    this->sr->end();
}

void Commander::enable()
{
    if (this->enabled == false)
    {
        this->enabled = true;
    }
    return;
}

void Commander::send_msg(uint8_t* buf, int16_t len)
{
    if (this->enabled == false)
    {
        return;
    }

    this->sr->write(buf, len);
    
    return;
}


void Commander::execute_cmd()
{
    switch (this->cmd_code)
    {
    case MSG_SET_EXTRUSION_SPEED:
        this->set_extrusion_speed_cb(&(this->extra_data), this->numBytesTot-1);
        break;

    case MSG_SET_HEATING:
        this->set_heating_cb(&(this->extra_data), this->numBytesTot-1);
        break;

    case MSG_READ_EXTRUSION_SPEED:
        this->read_extrusion_speed_cb();
        break;

    case MSG_READ_TEMPERATURE:
        this->read_temperature_cb();
        break;

    case MSG_BLINK_DEBUG_LED:
        this->blink_debug_led_cb();
        break;

    case MSG_START_PERIODIC_MESSAGING:
        this->start_periodic_messaging_cb();
        break;

    case MSG_STOP_PERIODIC_MESSAGING:
        this->stop_periodic_messaging_cb();
        break;

    default:
        /* Should not hit. Silently ignore. */
        break;
    }

    return; 
}

void Commander::process_byte()
{
    /* Process command byte and check if more bytes should be read */
    if (this->numBytesRcvd < 1)
    {
        this->cmd_code = this->sr->read();
        switch (this->cmd_code)
        {
        case MSG_SET_EXTRUSION_SPEED:
            this->numBytesTot  = 5;
            this->numBytesRcvd = 1;
            break;

        case MSG_SET_HEATING:
            this->numBytesTot  = 5;
            this->numBytesRcvd = 1;
            break;

        case MSG_READ_EXTRUSION_SPEED:
            this->numBytesRcvd = 1;
            break;

        case MSG_READ_TEMPERATURE:
            this->numBytesRcvd = 1;
            break;

        case MSG_BLINK_DEBUG_LED:
            this->numBytesRcvd = 1;
            break;

        case MSG_START_PERIODIC_MESSAGING:
            this->numBytesRcvd = 1;
            break;

        case MSG_STOP_PERIODIC_MESSAGING:
            this->numBytesRcvd = 1;
            break;

        default:
            timesInvalidCmdRcvd = timesInvalidCmdRcvd + 1;
            break;
        }
    }

    /* Read numBytesTot-1 extra bytes */
    else if (this->numBytesRcvd < this->numBytesTot)
    {
        (this->extra_data.arr_b)[this->numBytesRcvd-1] = this->sr->read();
        this->numBytesRcvd = this->numBytesRcvd + 1;
    }

    /* Execute command when all bytes have been read */
    if (this->numBytesRcvd >= this->numBytesTot)
    {
        this->execute_cmd();
        this->sumBytesRcvd = this->sumBytesRcvd + this->numBytesTot;
        this->numBytesTot  = 1;
        this->numBytesRcvd = 0;
    }

    return;
}

void Commander::process_incomming()
{
    if (this->enabled == false)
    {
        return;
    }

    while (this->sr->available() > 0)
    {
        this->process_byte();
    }

    return;
}
