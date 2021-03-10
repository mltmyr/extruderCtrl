#include "src/Pins/App_Pins.h"
#include "src/DebugLED/DebugLED.h"

#include "src/Thermistor/Thermistor.h"
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

  cmdr_ptr->send_msg((byte*)msg, sizeof(msg));
  return;
}

void set_heat_ref(extra_bytes_t* data, byte len)
{
  if (len == 4)
  {
    float temp_ref = (data->arr_f[0]);
    //htr_ctrl_ptr->setTempRef(temp_ref);
    Serial.print("TempRef: ");
    Serial.println(temp_ref);
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
    float target_step_freq = (data->arr_f[0]);
    //stepper_setSteppingFrequency(target_step_freq);
    Serial.print("StepFreq: ");
    Serial.println(target_step_freq);
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
  Thermistor_config_t thrm_cfg;
  thrm_cfg.R_vdiv = 4700;
  thrm_cfg.R_T1   = 100000;
  thrm_cfg.T1     = 25;
  thrm_cfg.B      = 4267;
  
  thrm_ptr = new Thermistor(TEMP_0_PIN, &thrm_cfg, 5);

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
  stepper_init(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN, 50);
  stepper_enable();

  /* ===[Serial communication and command interpreter]=== */
  Serial.begin(9600, SERIAL_8N1);
  cmdr_ptr = new Commander(&Serial);

  cmdr_ptr->set_extrusion_speed_cb = set_stepping_freq;
  cmdr_ptr->read_extrusion_speed_cb = sendSteppingFreq;

  cmdr_ptr->set_heating_cb = set_heat_ref;
  cmdr_ptr->read_temperature_cb = send_temp;
  
  cmdr_ptr->blink_debug_led_cb = blink_debug_led;

  cmdr_ptr->enable();
  /* ======== */

  stepper_setSteppingFrequency(-1.0);

  LED_init_done_blink(LED_PIN);
  Serial.println("Setup done!");
}

void loop()
{
  delay(10);
  
  thrm_ptr->PeriodicReadTemp();
  htr_ctrl_ptr->processControl();

  fanControl();

  stepper_process();
    
  cmdr_ptr->process_incomming();

  LED_process();
}
