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

#include "src/Fan/Fan.h"
#define  FAN_TURN_ON_TEMP_THRESHOLD  50
#define  FAN_TURN_OFF_TEMP_THRESHOLD 40
#define  FAN_ON  255
#define  FAN_OFF   0

#include "src/Stepper/Stepper.h"

#include "src/Commander/Commander.h"

Thermistor*    thrm_ptr;
Pid*           thrm_pid_ptr;
Heater*        htr_ptr;
HeaterControl* htr_ctrl_ptr;
Fan*           fan_ptr;

Commander*     cmdr_ptr;

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
  byte msg[5];

  msg[0] = MSG_READ_TEMPERATURE;
  float TempCelsius = thrm_ptr->getTemp();
  memcpy(&(msg[1]), &TempCelsius, sizeof(TempCelsius));

  //volatile float temp = -1.2;

  Serial2.println(TempCelsius, DEC);
  //cmdr_ptr->send_msg((byte*)msg, sizeof(msg));
  return;
}

void set_heat_ref(extra_bytes_t* data, byte len)
{
  if (len == 4)
  {
    float temp_ref = (data->arr_f[0]);
    //htr_ctrl_ptr->setTempRef(temp_ref);
    Serial2.print("TempRef: ");
    Serial2.println(temp_ref);
  }
  return;
}

void sendSteppingFreq()
{
  byte msg[5];

  msg[0] = MSG_READ_EXTRUSION_SPEED;
  float stepFreq = stepper_getSteppingFrequency();
  memcpy(&(msg[1]), &stepFreq, sizeof(stepFreq));

  cmdr_ptr->send_msg((byte*)msg, sizeof(msg));
  return;
}

void set_stepping_freq(extra_bytes_t* data, byte len)
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
      stepper_setSteppingFrequency(-64000.0);
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
  pid_cfg.Kp     = 1;
  pid_cfg.Ki     = 0.1;
  pid_cfg.Kd     = 0.1;
  pid_cfg.i_sum0 = 0;
  pid_cfg.u_lim_low  = 0;
  pid_cfg.u_lim_high = 255;

  thrm_pid_ptr = new Pid(&pid_cfg);

  htr_ptr = new Heater(HEATER_0_PIN);

  htr_ctrl_ptr = new HeaterControl(thrm_ptr, thrm_pid_ptr, htr_ptr, 5);

  fan_ptr = new Fan(FAN_PIN);

  /* ===[Stepper motor control]=== */
  stepper_init(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN, 4000);
  stepper_enable();

  /* ===[Serial communication and command interpreter]=== */
  Serial2.begin(9600, SERIAL_8N1);
  cmdr_ptr = new Commander(&Serial2);

  cmdr_ptr->set_extrusion_speed_cb = set_stepping_freq;
  cmdr_ptr->read_extrusion_speed_cb = sendSteppingFreq;

  cmdr_ptr->set_heating_cb = set_heat_ref;
  cmdr_ptr->read_temperature_cb = send_temp;
  
  cmdr_ptr->blink_debug_led_cb = blink_debug_led;

  cmdr_ptr->enable();
  
  /* ======== */

  LED_init_done_blink(LED_PIN);
  Serial2.println("Setup done!");
}

boolean speedup = false;

void loop()
{ 
  thrm_ptr->PeriodicReadTemp();
  //htr_ctrl_ptr->processControl();

  fanControl();

  stepper_process();
    
  cmdr_ptr->process_incomming();

  LED_process();
}
