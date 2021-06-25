#include <Arduino.h>

#include "src/GlobalConfig.h"
#include "src/Pins/App_Pins.h"
#include "src/DebugLED/DebugLED.h"

#if USE_THERMISTOR_TABLE
  #include "src/TableThermistor/TableThermistor.h"
  #include "src/TableThermistor/thermistor_104gt_and_ramps14_table.h"
#else
  #include "src/Thermistor/Thermistor.h"
#endif

#include "src/PID/PID.h"
#include "src/Heater/Heater.h"
#include "src/Heater_control/Heater_control.h"
#define HEATER_CARTRIDGE_RATED_POWER 30.0

#include "src/Fan/Fan.h"
#define  FAN_TURN_ON_TEMP_THRESHOLD  40
#define  FAN_TURN_OFF_TEMP_THRESHOLD 35
#define  FAN_ON  255
#define  FAN_OFF   0

#include "src/Stepper/Stepper.h"

#include "src/Commander/Commander.h"
#include "src/Periodically/Periodically.h"

#define SERIAL_MODULE Serial

Thermistor*    thrm_ptr;
Pid*           thrm_pid_ptr;
Heater*        htr_ptr;
HeaterControl* htr_ctrl_ptr;
Fan*           fan_ptr;

Commander*     cmdr_ptr;
Periodically*  temp_sender_ptr = nullptr;
Periodically*  extruder_speed_sender_ptr = nullptr;

void fanControl()
{
  if (thrm_ptr->getTemp() >= FAN_TURN_ON_TEMP_THRESHOLD)
  {
    fan_ptr->setSpeed(FAN_ON);
  }
  else if (thrm_ptr->getTemp() <= FAN_TURN_OFF_TEMP_THRESHOLD)
  {
    fan_ptr->setSpeed(FAN_OFF);
  }
  return;
}

void send_temp()
{
  uint8_t msg[5];

  msg[0] = MSG_READ_TEMPERATURE;
  float TempCelsius = thrm_ptr->getTemp();
  //memcpy(&(msg[1]), &TempCelsius, sizeof(TempCelsius));

  SERIAL_MODULE.print("T");
  SERIAL_MODULE.println(TempCelsius, DEC);
  //cmdr_ptr->send_msg((uint8_t*)msg, sizeof(msg));
  return;
}

void periodically_send_temperature(void* context, uint8_t context_length)
{
  uint8_t msg[5];

  msg[0] = MSG_READ_TEMPERATURE;
  float TempCelsius = thrm_ptr->getTemp();
  //memcpy(&(msg[1]), &TempCelsius, sizeof(TempCelsius));
  
  SERIAL_MODULE.print("T");
  SERIAL_MODULE.println(TempCelsius,DEC);
  //cmdr_ptr->send_msg((uint8_t*)msg, sizeof(msg));
  return;
}

void periodically_send_extruder_speed(void* context, uint8_t context_length)
{
  uint8_t msg[5];
  
  float stepFreq = stepper_getSteppingFrequency();
  //memcpy(&(msg[1]), &stepFreq, sizeof(stepFreq));

  SERIAL_MODULE.print("E");
  SERIAL_MODULE.println(stepFreq, DEC);
  //cmdr_ptr->send_msg((uint8_t*)msg, sizeof(msg));
  return;
}

void start_periodic_messaging()
{
  if (temp_sender_ptr != nullptr)
  {
    temp_sender_ptr->start();
  }
  if (extruder_speed_sender_ptr != nullptr)
  {
    extruder_speed_sender_ptr->start();
  }
  return;
}

void stop_periodic_messaging()
{
  if (temp_sender_ptr != nullptr)
  {
    temp_sender_ptr->stop();
  }
  if (extruder_speed_sender_ptr != nullptr)
  {
    extruder_speed_sender_ptr->stop();
  }
  return;
}

void set_heat_ref(extra_bytes_t* data, uint8_t len)
{
  if (len == 4)
  {
    //float temp_ref = (data->arr_f[0]);
    //htr_ctrl_ptr->setTempRef(temp_ref);
    //Serial2.print("TempRef: ");
    //Serial2.println(temp_ref);
    
    static boolean b = false;
    if (b == false)
    {
      htr_ctrl_ptr->setTempRef(215.0);
      b = true;
      SERIAL_MODULE.print("t");
      SERIAL_MODULE.println(215.0,DEC);
    }
    else
    {
      htr_ctrl_ptr->setTempRef(0.0);
      b = false;
      SERIAL_MODULE.print("t");
      SERIAL_MODULE.println(0.0,DEC);
    }
  }
  return;
}

void sendSteppingFreq()
{
  uint8_t msg[5];

  msg[0] = MSG_READ_EXTRUSION_SPEED;
  float stepFreq = stepper_getSteppingFrequency();
  memcpy(&(msg[1]), &stepFreq, sizeof(stepFreq));

  cmdr_ptr->send_msg((uint8_t*)msg, sizeof(msg));
  return;
}

void set_stepping_freq(extra_bytes_t* data, uint8_t len)
{
  if (len == 4)
  {
    /*float target_step_freq = (data->arr_f[0]);
    //stepper_setSteppingFrequency(target_step_freq);
    Serial.print("StepFreq: ");
    Serial.println(target_step_freq);*/
    static boolean a = false;
    if (a == false)
    {
      stepper_setSteppingFrequency(64000.0);
      a = true;
    }      
    else
    {
      stepper_setSteppingFrequency(0.0);
      a = false;
    }
  }
  return;
}

void blink_debug_led()
{
  LED_debug_blink(LED_PIN);
}

void setup()
{  
  LED_startup_blink(LED_PIN);

  /* ===[Heating control]=== */
#if USE_THERMISTOR_TABLE
  thrm_ptr = new Thermistor(TEMP_0_PIN, therm_104gt_ramps14_table, therm_104gt_ramps14_table_len, 10);
#else
  Thermistor_config_t thrm_cfg;
  thrm_cfg.R_vdiv = 4700;
  thrm_cfg.R_T1   = 100000;
  thrm_cfg.T1     = 25;
  thrm_cfg.B      = 4267;
  
  thrm_ptr = new Thermistor(TEMP_0_PIN, &thrm_cfg, 5);
#endif
  
  pid_cfg_t pid_cfg;
  pid_cfg.Kp     = 0.15;
  pid_cfg.Ki     = 0.005;
  pid_cfg.Kd     = 0;
  pid_cfg.i_sum0 = 0;
  pid_cfg.u_lim_low  = 0;
  pid_cfg.u_lim_high = 30;

  thrm_pid_ptr = new Pid(&pid_cfg);

  htr_ptr = new Heater(HEATER_0_PIN, HEATER_CARTRIDGE_RATED_POWER);

  htr_ctrl_ptr = new HeaterControl(thrm_ptr, thrm_pid_ptr, htr_ptr, 5);

  fan_ptr = new Fan(FAN_PIN);

  /* ===[Stepper motor control]=== */
  stepper_init(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN, 1000);
  stepper_enable();

  /* ===[Serial communication and command interpreter]=== */
  SERIAL_MODULE.begin(9600, SERIAL_8N1);
  
  cmdr_ptr = new Commander(&SERIAL_MODULE);

  cmdr_ptr->set_extrusion_speed_cb = set_stepping_freq;
  cmdr_ptr->read_extrusion_speed_cb = sendSteppingFreq;

  cmdr_ptr->set_heating_cb = set_heat_ref;
  cmdr_ptr->read_temperature_cb = send_temp;
  
  cmdr_ptr->blink_debug_led_cb = blink_debug_led;

  cmdr_ptr->start_periodic_messaging_cb = start_periodic_messaging;
  cmdr_ptr->stop_periodic_messaging_cb = stop_periodic_messaging;

  temp_sender_ptr           = new Periodically(periodically_send_temperature,    NULL, 0, 1);
  //extruder_speed_sender_ptr = new Periodically(periodically_send_extruder_speed, NULL, 0, 0.5);

  cmdr_ptr->enable();
  //temp_sender_ptr->start();
  //extruder_speed_sender_ptr->start();

  /* ======== */

  LED_init_done_blink(LED_PIN);
  SERIAL_MODULE.println("Setup done!");
}

boolean speedup = false;

void loop()
{ 
  thrm_ptr->PeriodicReadTemp();
  htr_ctrl_ptr->processControl();

  fanControl();

  stepper_process();
    
  cmdr_ptr->process_incomming();

  temp_sender_ptr->process();
  extruder_speed_sender_ptr->process();

  LED_process();
}
