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

#include <Arduino.h>
#include "Mower.h"

Mower robot;

Mower::Mower()
{
  name = "Ardumower";

  // ------ perimeter ---------------------------------
  perimeterTriggerTimeout = 0;    // perimeter trigger timeout when escaping from inside (ms)
  perimeterOutRollTimeMax = 2000; // roll time max after perimeter out (ms)
  perimeterOutRollTimeMin = 750;  // roll time min after perimeter out (ms)
  perimeterOutRevTime = 2200;     // reverse time after perimeter out (ms)
  perimeterTrackRollTime = 1500;  // roll time during perimeter tracking
  perimeterTrackRevTime = 2200;   // reverse time during perimeter tracking
  perimeters.perimeter[Perimeter::LEFT].pid.setup(51.0, 12.5, 0.8);  // perimeter PID controller
  //perimeters.perimeter[Perimeter::RIGHT].pid.setup(51.0, 12.5, 0.8);  // perimeter PID controller
  trackingPerimeterTransitionTimeOut = 2000;
  trackingErrorTimeOut = 10000;
  trackingBlockInnerWheelWhilePerimeterStruggling = true;

  // ------  IMU (compass/accel/gyro) ----------------------
  imu.pid[Imu::DIR].setup(5.0, 1.0, 1.0);  // direction PID controller
  imu.pid[Imu::ROLL].setup(0.8, 21, 0);    // roll PID controller

  // ------ model R/C ------------------------------------
  remoteUse = false; // use model remote control (R/C)?

  // ------  charging station ---------------------------
  stationRevTime = 1800;   // charge station reverse time (ms)
  stationRollTime = 1000;  // charge station roll time (ms)
  stationForwTime = 1500;  // charge station forward time (ms)
  stationCheckTime = 1700; // charge station reverse check time (ms)

  // ----- GPS -------------------------------------------
  gpsUse = false;                   // use GPS?
  stuckIfGpsSpeedBelow = 0.2; // if Gps speed is below given value the mower is stuck
  gpsSpeedIgnoreTime = 5000;    // how long gpsSpeed is ignored when robot switches into a new STATE (in ms)

  // ----- user-defined switch ---------------------------
  userSwitch1 = false; // user-defined switch 1 (default value)
  userSwitch2 = false; // user-defined switch 2 (default value)
  userSwitch3 = false; // user-defined switch 3 (default value)

  // ----- timer -----------------------------------------
  timerUse = false; // use RTC and timer?

  // -----------configuration end-------------------------------------
}

// remote control (RC) ppm signal change interrupt
ISR(PCINT0_vect)
{
  unsigned long timeMicros = micros();
  boolean remoteSpeedState = digitalRead(PIN_REMOTE_SPEED);
  boolean remoteSteerState = digitalRead(PIN_REMOTE_STEER);
  boolean remoteMowState = digitalRead(PIN_REMOTE_MOW);
  boolean remoteSwitchState = digitalRead(PIN_REMOTE_SWITCH);
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
  robot.odometer.readAndSetState();

  // TODO: Move this elsewhere
  robot.cutter.motor.setRpmState();
}


// mower motor speed sensor interrupt
//void rpm_interrupt(){
//}

void Mower::setup()
{
  Wire.begin();
  Console.begin(BAUDRATE);
  Console.println("SETUP");
  rc.initSerial(PFOD_BAUDRATE);

  // LED
  pinMode(PIN_LED, OUTPUT);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, 0);

  // Battery
  battery_setup(PIN_BATTERY_VOLTAGE, PIN_CHARGE_VOLTAGE, PIN_CHARGE_CURRENT,
                PIN_CHARGE_RELAY, PIN_BATTERY_SWITCH, &battery);

  // ------- wheel motors -----------------------------
  wheels.rollTimeMax = 1500;      // max. roll time (ms)
  wheels.rollTimeMin = 750;       // min. roll time (ms) should be smaller than motorRollTimeMax
  wheels.reverseTime = 1200;      // max. reverse time (ms)
  wheels.forwardTimeMax = 80000;  // max. forward time (ms) / timeout
  wheels.biDirSpeedRatio1 = 0.3;  // bidir mow pattern speed ratio 1
  wheels.biDirSpeedRatio2 = 0.92; // bidir mow pattern speed ratio 2

  // left wheel motor
  wheels.wheel[Wheel::LEFT].motor.config(1000.0,   // Acceleration
                                         255,      // Max PWM
                                         75,       // Max Power
                                         false,    // Regulate
                                         100,      // Max RPM
                                         0.0);     // Set RPM
  wheels.wheel[Wheel::LEFT].motor.setScale(3.25839);
  wheels.wheel[Wheel::LEFT].motor.setChannel(0);
  wheels.wheel[Wheel::LEFT].motor.setup();
  // Normal control
  wheels.wheel[Wheel::LEFT].motor.pid.setup(1.5, 0.29, 0.25);  // Kp, Ki, Kd
  // Fast control
  //wheels.wheel[Wheel::LEFT].motor.pid.setup(1.76, 0.87, 0.4);  // Kp, Ki, Kd
  wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime = 2000;  // time to ignore motor power (ms)
  wheels.wheel[Wheel::LEFT].motor.zeroSettleTime = 3000;   // how long (ms) to wait for motors to settle at zero speed
  wheels.wheel[Wheel::LEFT].motor.swapDir = 0;  // inverse left motor direction?
  encoder_setup2pin(PIN_ODOMETER_LEFT,
                    PIN_ODOMETER_LEFT_2,
                    ODOMETER_SWAP_DIR_LEFT,
                    &wheels.wheel[Wheel::LEFT].encoder);
  // right wheel motor
  wheels.wheel[Wheel::RIGHT].motor.config(1000.0,   // Acceleration
                                          255,      // Max PWM
                                          75,       // Max Power
                                          false,    // Regulate
                                          100,      // Max RPM
                                          0.0);     // Set RPM
  wheels.wheel[Wheel::RIGHT].motor.setScale(3.25839);
  wheels.wheel[Wheel::RIGHT].motor.setChannel(1);
  wheels.wheel[Wheel::RIGHT].motor.setup();
  // Normal control
  wheels.wheel[Wheel::RIGHT].motor.pid.setup(1.5, 0.29, 0.25);  // Kp, Ki, Kd
  // Fast control
  //wheels.wheel[Wheel::RIGHT].motor.pid.setup(1.76, 0.87, 0.4);  // Kp, Ki, Kd
  wheels.wheel[Wheel::RIGHT].motor.powerIgnoreTime = 2000;  // time to ignore motor power (ms)
  wheels.wheel[Wheel::RIGHT].motor.zeroSettleTime = 3000;   // how long (ms) to wait for motors to settle at zero speed
  wheels.wheel[Wheel::RIGHT].motor.swapDir = 0; // inverse right motor direction?
  encoder_setup2pin(PIN_ODOMETER_RIGHT,
                    PIN_ODOMETER_RIGHT_2,
                    ODOMETER_SWAP_DIR_RIGHT,
                    &wheels.wheel[Wheel::RIGHT].encoder);

  // mower motor
  cutter.motor.config(2000.0,     // Acceleration
                      255,        // Max PWM
                      75,         // Max Power
                      false,      // Modulation
                      0,          // Max RPM
                      3300);      // Set RPM
  cutter.motor.setScale(3.25839);
  cutter.motor.pid.setup(0.005, 0.01, 0.01);  // Kp, Ki, Kd

  // lawn sensor
  const uint8_t lawnSensorSendPins[LAWNSENSORS_NUM] = {
      PIN_LAWN_FRONT_SEND, PIN_LAWN_BACK_SEND};
  const uint8_t lawnSensorReceivePins[LAWNSENSORS_NUM] = {
      PIN_LAWN_FRONT_RECV, PIN_LAWN_BACK_RECV};
  lawnSensors_setup(lawnSensorSendPins, lawnSensorReceivePins, lawnSensorArray,
      &lawnSensors, LAWNSENSORS_NUM);

  // perimeter
  perimeters.use = false;
  perimeters.perimeter[Perimeter::LEFT].setup(PIN_PERIMETER_LEFT);
  //perimeters.perimeter[Perimeter::RIGHT].setup(PIN_PERIMETER_RIGHT);

  // button
  button_setup(PIN_BUTTON, &button);

  // bumpers
  const uint8_t bumperPins[BUMPERS_NUM] = {PIN_BUMBER_LEFT, PIN_BUMBER_RIGHT};
  bumpers_setup(bumperPins, bumperArray, &bumpers, BUMPERS_NUM);

  // drop sensor
  const uint8_t dropSensorPins[DROPSENSORS_NUM] = {PIN_DROP_LEFT, PIN_DROP_RIGHT};
  dropSensors_setup(dropSensorPins, DROPSENSOR_NO, dropSensorArray,
                    &dropSensors, DROPSENSORS_NUM);

  // sonar
  sonars.use = true;
  sonar_setup(PIN_SONAR_LEFT_TRIGGER, PIN_SONAR_LEFT_ECHO,
              SONAR_DEFAULT_MAX_ECHO_TIME,
              SONAR_DEFAULT_MIN_ECHO_TIME,
              &sonars.sonarArray_p[LEFT]);
  sonar_setup(PIN_SONAR_RIGHT_TRIGGER, PIN_SONAR_RIGHT_ECHO,
              SONAR_DEFAULT_MAX_ECHO_TIME,
              SONAR_DEFAULT_MIN_ECHO_TIME,
              &sonars.sonarArray_p[RIGHT]);
  sonar_setup(PIN_SONAR_CENTER_TRIGGER, PIN_SONAR_CENTER_ECHO,
              SONAR_DEFAULT_MAX_ECHO_TIME,
              SONAR_DEFAULT_MIN_ECHO_TIME,
              &sonars.sonarArray_p[CENTER]);
  sonars.sonarArray_p[LEFT].use = false;
  sonars.sonarArray_p[RIGHT].use = false;
  sonars.sonarArray_p[CENTER].use = true;

  // rain
  rainSensor.use = false;
  rainSensor_setup(PIN_RAIN, &rainSensor);

  // R/C
  pinMode(PIN_REMOTE_MOW, INPUT);
  pinMode(PIN_REMOTE_STEER, INPUT);
  pinMode(PIN_REMOTE_SPEED, INPUT);
  pinMode(PIN_REMOTE_SWITCH, INPUT);

  // odometer
  odometer.setup(ODOMETER_TICKS_PER_REVOLUTION,
                 ODOMETER_TICKS_PER_CM,
                 ODOMETER_WHEELBASE_CM,
                 &wheels.wheel[Wheel::LEFT].encoder,
                 &wheels.wheel[Wheel::RIGHT].encoder,
                 &imu);

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

  imu.init(PIN_BUZZER);
  gps.init();

  Robot::setup();
}

int Mower::readSensor(Robot::sensorE type)
{
  switch (type)
  {
// rtc--------------------------------------------------------------------------
    case SEN_RTC:
      if (!readDS1307(datetime))
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
      if (!setDS1307(datetime))
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

void Mower::configureBluetooth(boolean quick)
{
  BluetoothConfig bt;
  bt.setParams(name, PFOD_BLUETOOTH_PIN_CODE, PFOD_BAUDRATE, quick);
}
