/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2015 by Alexander Grau
 Copyright (c) 2013-2015 by Sven Gennat
 Copyright (c) 2014 by Maxime Carpentieri
 Copyright (c) 2014-2015 by Stefan Manteuffel
 Copyright (c) 2015 by Uwe Zimprich

 Private-use only! (you need to ask for a commercial-use)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 Private-use only! (you need to ask for a commercial-use)
 */

/*
 Ardumower Chassis Kit 1.0 - robot configuration (Ardumower electronics, Arduino Mega)
 http://wiki.ardumower.de/index.php?title=Ardumower_chassis

 Requires: Ardumower PCB v0.5  ( https://www.marotronics.de/Ardumower-Board-Prototyp )
 */

#include "Config.h"

#include "Mower.h"
#include "Pid.h"

Mower robot;

Mower::Mower()
{
  m_name = "Ardumower";

  // ------ perimeter ---------------------------------
  m_perimeterTriggerTimeout = 0;    // perimeter trigger timeout when escaping from inside (ms)
  m_perimeterOutRollTimeMax = 2000; // roll time max after perimeter out (ms)
  m_perimeterOutRollTimeMin = 750;  // roll time min after perimeter out (ms)
  m_perimeterOutRevTime = 2200;     // reverse time after perimeter out (ms)
  m_perimeterTrackRollTime = 1500;  // roll time during perimeter tracking
  m_perimeterTrackRevTime = 2200;   // reverse time during perimeter tracking
  m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_pid.setup(
      51.0, 12.5, 0.8, -100.0, 100.0, 100.0);  // perimeter PID controller
  m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_pid.setSetPoint(0);
  //perimeters.perimeter[Perimeter::RIGHT].pid.setup(51.0, 12.5, 0.8);  // perimeter PID controller
  m_trackingPerimeterTransitionTimeOut = 2000;
  m_trackingErrorTimeOut = 10000;
  m_trackingBlockInnerWheelWhilePerimeterStruggling = true;

  // ------  IMU (compass/accel/gyro) ----------------------
  m_imu.m_pid[Imu::DIR].setup(5.0, 1.0, 1.0, -100.0, 100.0, 100.0);  // direction PID controller
  m_imu.m_pid[Imu::DIR].setSetPoint(0);
  m_imu.m_pid[Imu::ROLL].setup(0.8, 21, 0, -80.0, 80.0, 80.0);    // roll PID controller
  m_imu.m_pid[Imu::ROLL].setSetPoint(0);

  // ------ model R/C ------------------------------------
  m_remoteUse = false; // use model remote control (R/C)?

  // ------  charging station ---------------------------
  m_stationRevTime = 1800;   // charge station reverse time (ms)
  m_stationRollTime = 1000;  // charge station roll time (ms)
  m_stationForwTime = 1500;  // charge station forward time (ms)
  m_stationCheckTime = 1700; // charge station reverse check time (ms)

  // ----- GPS -------------------------------------------
  m_gpsUse = false;                   // use GPS?
  m_stuckIfGpsSpeedBelow = 0.2; // if Gps speed is below given value the mower is stuck
  m_gpsSpeedIgnoreTime = 5000;    // how long gpsSpeed is ignored when robot switches into a new STATE (in ms)

  // ----- user-defined switch ---------------------------
  m_userSwitch1 = false; // user-defined switch 1 (default value)
  m_userSwitch2 = false; // user-defined switch 2 (default value)
  m_userSwitch3 = false; // user-defined switch 3 (default value)

  // ----- timer -----------------------------------------
  m_timerUse = false; // use RTC and timer?

  // -----------configuration end-------------------------------------
}

// remote control (RC) ppm signal change interrupt
ISR(PCINT0_vect)
{
  unsigned long timeMicros = micros();
  bool remoteSpeedState = digitalRead(PIN_REMOTE_SPEED);
  bool remoteSteerState = digitalRead(PIN_REMOTE_STEER);
  bool remoteMowState = digitalRead(PIN_REMOTE_MOW);
  bool remoteSwitchState = digitalRead(PIN_REMOTE_SWITCH);
  robot.setRemotePPMState(timeMicros, remoteSpeedState, remoteSteerState,
                          remoteMowState, remoteSwitchState);
}

// odometer signal change interrupt
// mower motor speed sensor interrupt
// NOTE: when choosing a higher perimeter sample rate (38 kHz) and using odometer interrupts,
// the Arduino Mega cannot handle all ADC interrupts anymore - the result will be a 'noisy'
// perimeter filter output (mag value) which disappears when disabling odometer interrupts.
// SOLUTION: allow odometer interrupt handler nesting (see odometer interrupt function)
// http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
ISR(PCINT2_vect, ISR_NOBLOCK)
{
  robot.m_odometer.read();

  // TODO: Move this elsewhere
  robot.m_cutter.m_motor.setRpmState();
}


// mower motor speed sensor interrupt
//void rpm_interrupt(){
//}

void Mower::setup()
{
  Wire.begin();
  Console.begin(BAUDRATE);
  Console.println("SETUP");
  m_rc.initSerial(PFOD_BAUDRATE);

  // LED
  pinMode(PIN_LED, OUTPUT);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, 0);

  // Battery
  // Nothing to be done here

  // ------- wheel motors -----------------------------
  m_wheels.m_rollTimeMax = 1500;      // max. roll time (ms)
  m_wheels.m_rollTimeMin = 750;       // min. roll time (ms) should be smaller than motorRollTimeMax
  m_wheels.m_reverseTime = 1200;      // max. reverse time (ms)
  m_wheels.m_forwardTimeMax = 80000;  // max. forward time (ms) / timeout
  m_wheels.m_biDirSpeedRatio1 = 0.3;  // bidir mow pattern speed ratio 1
  m_wheels.m_biDirSpeedRatio2 = 0.92; // bidir mow pattern speed ratio 2

  // left wheel motor
  m_wheels.m_wheel[Wheel::LEFT].m_motor.config(1000.0,   // Acceleration
                                         255,      // Max PWM
                                         75,       // Max Power
                                         false,    // Regulate
                                         100,      // Max RPM
                                         0.0);     // Set RPM
  m_wheels.m_wheel[Wheel::LEFT].m_motor.setScale(3.25839);
  m_wheels.m_wheel[Wheel::LEFT].m_motor.setChannel(0);
  m_wheels.m_wheel[Wheel::LEFT].m_motor.setup();
  // Normal control
  int16_t pwmMax = m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pwmMax;
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pid.setup(
      1.5, 0.29, 0.25, -pwmMax, pwmMax, pwmMax);  // Kp, Ki, Kd
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pid.setSetPoint(
      m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet);
  // Fast control
  //wheels.wheel[Wheel::LEFT].motor.pid.setup(1.76, 0.87, 0.4);  // Kp, Ki, Kd
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_powerIgnoreTime = 2000;  // time to ignore motor power (ms)
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_zeroSettleTime = 3000;   // how long (ms) to wait for motors to settle at zero speed
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_swapDir = 0;  // inverse left motor direction?
  m_wheels.m_wheel[Wheel::LEFT].m_encoder.setup(
      PIN_ODOMETER_LEFT, ODOMETER_SWAP_DIR_LEFT);

  // right wheel motor
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.config(1000.0,   // Acceleration
                                          255,      // Max PWM
                                          75,       // Max Power
                                          false,    // Regulate
                                          100,      // Max RPM
                                          0.0);     // Set RPM
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.setScale(3.25839);
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.setChannel(1);
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.setup();
  // Normal control
  pwmMax = m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_pwmMax;
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_pid.setup(
      1.5, 0.29, 0.25, -pwmMax, pwmMax, pwmMax);  // Kp, Ki, Kd
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_pid.setSetPoint(
      m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet);
  // Fast control
  //wheels.wheel[Wheel::RIGHT].motor.pid.setup(1.76, 0.87, 0.4);  // Kp, Ki, Kd
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_powerIgnoreTime = 2000;  // time to ignore motor power (ms)
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_zeroSettleTime = 3000;   // how long (ms) to wait for motors to settle at zero speed
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_swapDir = 0; // inverse right motor direction?
  m_wheels.m_wheel[Wheel::RIGHT].m_encoder.setup(
      PIN_ODOMETER_RIGHT, ODOMETER_SWAP_DIR_RIGHT);

  // mower motor
  m_cutter.m_motor.config(2000.0,     // Acceleration
                      255,        // Max PWM
                      75,         // Max Power
                      false,      // Modulation
                      0,          // Max RPM
                      3300);      // Set RPM
  m_cutter.m_motor.setScale(3.25839);
  pwmMax = m_cutter.m_motor.m_pwmMax;
  m_cutter.m_motor.m_pid.setup(
      0.005, 0.01, 0.01, 0.0, pwmMax, pwmMax);  // Kp, Ki, Kd
  m_cutter.m_motor.m_pid.setSetPoint(m_cutter.m_motor.m_rpmSet);

  // lawn sensor
  const uint8_t lawnSensorSendPins[LAWNSENSORS_NUM] =
  {
      PIN_LAWN_FRONT_SEND, PIN_LAWN_BACK_SEND
  };
  const uint8_t lawnSensorReceivePins[LAWNSENSORS_NUM] =
  {
      PIN_LAWN_FRONT_RECV, PIN_LAWN_BACK_RECV
  };
  lawnSensors_setup(lawnSensorSendPins, lawnSensorReceivePins, m_lawnSensorArray,
      &m_lawnSensors, LAWNSENSORS_NUM);

  // perimeter
  m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].setup(PIN_PERIMETER_LEFT);
  m_perimeters.m_perimeterArray_p[PERIMETER_RIGHT].setup(PIN_PERIMETER_RIGHT);

  // button
  // Nothing to be done here

  // bumpers
  // Nothing to be done here

  // drop sensor
  const uint8_t dropSensorPins[DROPSENSORS_NUM] =
  {
      PIN_DROP_LEFT, PIN_DROP_RIGHT
  };
  m_dropSensors.setup(dropSensorPins,
      DropSensor_Contact::NO, m_dropSensorArray, DROPSENSORS_NUM);

  // sonar
  m_sonars.use = true;
  sonar_setup(
      PIN_SONAR_LEFT_TRIGGER, PIN_SONAR_LEFT_ECHO,
      SONAR_DEFAULT_MAX_ECHO_TIME,
      SONAR_DEFAULT_MIN_ECHO_TIME,
      &m_sonars.sonarArray_p[SONAR_LEFT]);
  sonar_setup(
      PIN_SONAR_RIGHT_TRIGGER, PIN_SONAR_RIGHT_ECHO,
      SONAR_DEFAULT_MAX_ECHO_TIME,
      SONAR_DEFAULT_MIN_ECHO_TIME,
      &m_sonars.sonarArray_p[SONAR_RIGHT]);
  sonar_setup(
      PIN_SONAR_CENTER_TRIGGER, PIN_SONAR_CENTER_ECHO,
      SONAR_DEFAULT_MAX_ECHO_TIME,
      SONAR_DEFAULT_MIN_ECHO_TIME,
      &m_sonars.sonarArray_p[SONAR_CENTER]);
  m_sonars.sonarArray_p[SONAR_LEFT].use = false;
  m_sonars.sonarArray_p[SONAR_RIGHT].use = false;
  m_sonars.sonarArray_p[SONAR_CENTER].use = true;

  // rain
  m_rainSensor.use = false;
  rainSensor_setup(PIN_RAIN, &m_rainSensor);

  // R/C
  pinMode(PIN_REMOTE_MOW, INPUT);
  pinMode(PIN_REMOTE_STEER, INPUT);
  pinMode(PIN_REMOTE_SPEED, INPUT);
  pinMode(PIN_REMOTE_SWITCH, INPUT);

  // odometer
  m_odometer.setup(ODOMETER_TICKS_PER_REVOLUTION,
                 ODOMETER_TICKS_PER_CM,
                 ODOMETER_WHEELBASE_CM,
                 &m_wheels.m_wheel[Wheel::LEFT].m_encoder,
                 &m_wheels.m_wheel[Wheel::RIGHT].m_encoder,
                 &m_imu);

  // user switches
  pinMode(PIN_USER_SWITCH_1, OUTPUT);
  pinMode(PIN_USER_SWITCH_2, OUTPUT);
  pinMode(PIN_USER_SWITCH_3, OUTPUT);


  // PWM frequency setup
  // For obstacle detection, motor torque should be detectable - torque can be computed by motor current.
  // To get consistent current values, PWM frequency should be 3.9 Khz
  // http://wiki.ardumower.de/index.php?title=Motor_driver
  // http://sobisource.com/arduino-mega-pwm-pin-and-frequency-timer-control/
  // http://www.atmel.com/images/doc2549.pdf
  TCCR3B = (TCCR3B & 0xF8) | 0x02;  // set PWM frequency 3.9 Khz (pin2,3,5)
  TCCR1B = (TCCR1B & 0xF8) | 0x02;  // set PWM frequency 3.9 Khz (pin11,12)

  // enable interrupts
  // R/C
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT4);
  PCMSK0 |= (1 << PCINT5);
  PCMSK0 |= (1 << PCINT6);
  PCMSK0 |= (1 << PCINT1);

  // odometer
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);
  PCMSK2 |= (1 << PCINT21);
  PCMSK2 |= (1 << PCINT22);
  PCMSK2 |= (1 << PCINT23);

  // mower motor speed sensor interrupt
  //attachInterrupt(5, rpm_interrupt, CHANGE);
  PCMSK2 |= (1 << PCINT19);

  // ADC
  ADCMan.init();

  m_imu.init(PIN_BUZZER);
  m_gps.init();

  Robot::setup();
}

int Mower::readSensor(Robot::sensorE type)
{
  switch (type)
  {
// rtc--------------------------------------------------------------------------
    case SEN_RTC:
      if (!readDS1307(m_datetime))
      {
        Console.println("RTC data error!");
        //addErrorCounter(ERR_RTC_DATA);
        //setNextState(STATE_ERROR, 0);
      }
      break;
  }
  return 0;
}

void Mower::setActuator(Robot::actuatorE type, int value)
{
  switch (type)
  {
    case ACT_BUZZER:
      if (value)
      {
        tone(PIN_BUZZER, value);
      }
      else
      {
        noTone(PIN_BUZZER);
      }
      break;

    case ACT_LED:
      digitalWrite(PIN_LED, value);
      break;

    case ACT_USER_SW1:
      digitalWrite(PIN_USER_SWITCH_1, value);
      break;

    case ACT_USER_SW2:
      digitalWrite(PIN_USER_SWITCH_2, value);
      break;

    case ACT_USER_SW3:
      digitalWrite(PIN_USER_SWITCH_3, value);
      break;

    case ACT_RTC:
      if (!setDS1307(m_datetime))
      {
        Console.println("RTC comm error!");
        incErrorCounter(ERR_RTC_COMM);
        setNextState(StateMachine::STATE_ERROR);
      }
      break;

    case ACT_BATTERY_SW:
      digitalWrite(PIN_BATTERY_SWITCH, value);
      break;
  }
}

void Mower::configureBluetooth(bool quick)
{
  BluetoothConfig bt;
  bt.setParams(m_name, PFOD_BLUETOOTH_PIN_CODE, PFOD_BAUDRATE, quick);
}
