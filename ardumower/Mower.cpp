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

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// NOTE: Verify in config.h that you have enabled 'USE_MOWER' !
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#include "Config.h"
#ifdef USE_MOWER

#include <Arduino.h>
#include "Mower.h"

Mower robot;

Mower::Mower()
{
  name = "Ardumower";

  // ------ perimeter ---------------------------------
  perimeterUse = true;            // use perimeter?
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
  trackingBlockInnerWheelWhilePerimeterStruggling = 1;

  // ------  IMU (compass/accel/gyro) ----------------------
  imu.pid[Imu::DIR].setup(5.0, 1.0, 1.0);  // direction PID controller
  imu.pid[Imu::ROLL].setup(0.8, 21, 0);    // roll PID controller

  // ------ model R/C ------------------------------------
  remoteUse = false; // use model remote control (R/C)?

  // ------ battery -------------------------------------
  batMonitor = false;          // monitor battery and charge voltage?
  batGoHomeIfBelow = 11.8;     // drive home voltage (Volt)
  batSwitchOffIfBelow = 10.8;  // switch off battery if below voltage (Volt)
  batSwitchOffIfIdle = 1;      // switch off battery if idle (minutes)
  batFactor = 0.495;           // battery conversion factor  / 10 due to arduremote bug, can be removed after fixing (look in robot.cpp)
  batChgFactor = 0.495;        // battery conversion factor  / 10 due to arduremote bug, can be removed after fixing (look in robot.cpp)
  batFull = 14.7;              // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
  batChargingCurrentMax = 1.6; // maximum current your charger can deliver
  batFullCurrent = 0.3;        // current flowing when battery is fully charged
  startChargingIfBelow = 13.5; // start charging if battery Voltage is below
  chargingTimeout = 12600000;  // safety timer for charging (ms) 12600000 = 3.5hrs
  // Sensorausgabe Konsole      (chgSelection = 0)
  // Einstellungen ACS712 5A    (chgSelection = 1 / chgSenseZero ~ 511 / chgFactor = 39 / chgSense = 185.0 / chgChange = 0 oder 1 (je nach Stromrichtung) / chgNull  = 2)
  // Einstellungen INA169 board (chgSelection = 2)
  chgSelection = 2;
  chgSenseZero = 511;          // charge current sense zero point
  chgFactor = 39;              // charge current conversion factor - Empfindlichkeit nimmt mit ca. 39/V Vcc ab
  chgSense = 185.0;            // mV/A empfindlichkeit des Ladestromsensors in mV/A (Für ACS712 5A = 185)
  chgChange = 0;               // Messwertumkehr von - nach +         1 oder 0
  chgNull = 2;                 // Nullduchgang abziehen (1 oder 2)

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

  // ------ mower stats-------------------------------------------
  statsOverride = false; // if set to true mower stats are overwritten with the values below - be careful
  statsMowTimeMinutesTotal = 300;
  statsBatteryChargingCounterTotal = 11;
  statsBatteryChargingCapacityTotal = 30000;

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
  unsigned long timeMicros = micros();
  robot.odometer.read();
  robot.odometer.setState(timeMicros);

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

  // Keep battery switched ON
  pinMode(PIN_BATTERY_SWITCH, OUTPUT);
  digitalWrite(PIN_BATTERY_SWITCH, HIGH);

  // LED
  pinMode(PIN_LED, OUTPUT);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, 0);

  // Battery
  pinMode(PIN_BATTERY_VOLTAGE, INPUT);
  pinMode(PIN_CHARGE_CURRENT, INPUT);
  pinMode(PIN_CHARGE_VOLTAGE, INPUT);
  pinMode(PIN_CHARGE_RELAY, OUTPUT);
  setActuator(ACT_CHGRELAY, 0);

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
                                         false,    // Modulation
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

  // right wheel motor
  wheels.wheel[Wheel::RIGHT].motor.config(1000.0,   // Acceleration
                                          255,      // Max PWM
                                          75,       // Max Power
                                          false,    // Modulation
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

  // mower motor
  cutter.motor.config(2000.0,     // Acceleration
                      255,        // Max PWM
                      75,         // Max Power
                      false,      // Modulation
                      0,          // Max RPM
                      3300);      // Set RPM
  cutter.motor.setScale(3.25839);
  cutter.motor.pid.setup(0.005, 0.01, 0.01, -127, 127, 127);  // Kp, Ki, Kd

  // lawn sensor
  lawnSensor.setup(PIN_LAWN_FRONT_SEND, PIN_LAWN_FRONT_RECV,
                   PIN_LAWN_BACK_SEND, PIN_LAWN_BACK_RECV);

  // perimeter
  perimeters.perimeter[Perimeter::LEFT].setup(PIN_PERIMETER_LEFT);
  //perimeters.perimeter[Perimeter::RIGHT].setup(PIN_PERIMETER_RIGHT);

  // button
  button.setup(PIN_BUTTON);

  // bumpers
  bumpers.bumper[Bumpers::LEFT].setup(PIN_BUMBER_LEFT);
  bumpers.bumper[Bumpers::RIGHT].setup(PIN_BUMBER_RIGHT);

  // drop sensor
  dropSensors.dropSensor[DropSensors::LEFT].setup(PIN_DROP_LEFT, DropSensor::NO);
  dropSensors.dropSensor[DropSensors::RIGHT].setup(PIN_DROP_LEFT, DropSensor::NO);

  // sonar
  sonars.use = true;
  sonars.sonar[Sonars::LEFT].setup(PIN_SONAR_LEFT_TRIGGER, PIN_SONAR_LEFT_ECHO);
  sonars.sonar[Sonars::RIGHT].setup(PIN_SONAR_RIGHT_TRIGGER, PIN_SONAR_RIGHT_ECHO);
  sonars.sonar[Sonars::CENTER].setup(PIN_SONAR_CENTER_TRIGGER, PIN_SONAR_CENTER_ECHO);
  sonars.sonar[Sonars::LEFT].use = false;
  sonars.sonar[Sonars::RIGHT].use = false;
  sonars.sonar[Sonars::CENTER].use = true;

  // rain
  rainSensor.setup(PIN_RAIN);

  // R/C
  pinMode(PIN_REMOTE_MOW, INPUT);
  pinMode(PIN_REMOTE_STEER, INPUT);
  pinMode(PIN_REMOTE_SPEED, INPUT);
  pinMode(PIN_REMOTE_SWITCH, INPUT);

  // odometer
  const uint8_t odometerPins[] = { PIN_ODOMETER_LEFT, PIN_ODOMETER_RIGHT };
  const uint8_t odometerPins2[] = { PIN_ODOMETER_LEFT_2, PIN_ODOMETER_RIGHT_2 };
  const bool odometerSwaps[] = { ODOMETER_SWAP_DIR_LEFT, ODOMETER_SWAP_DIR_RIGHT };
  odometer.setup(ODOMETER_TICKS_PER_REVOLUTION,
                 ODOMETER_TICKS_PER_CM,
                 ODOMETER_WHEELBASE_CM,
                 odometerPins,
                 odometerPins2,
                 odometerSwaps);

  // user switches
  pinMode(PIN_USER_SWITCH_1, OUTPUT);
  pinMode(PIN_USER_SWITCH_2, OUTPUT);
  pinMode(PIN_USER_SWITCH_3, OUTPUT);

  // other
  pinMode(PIN_VOLTAGE_MEASUREMENT, INPUT);

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
  ADCMan.setCapture(PIN_CHARGE_CURRENT, 1, true); //Aktivierung des LaddeStrom Pins beim ADC-Managers
  ADCMan.setCapture(PIN_BATTERY_VOLTAGE, 1, false);
  ADCMan.setCapture(PIN_CHARGE_VOLTAGE, 1, false);
  ADCMan.setCapture(PIN_VOLTAGE_MEASUREMENT, 1, false);

  imu.init(PIN_BUZZER);
  gps.init();

  Robot::setup();
}

int Mower::readSensor(char type)
{
  switch (type)
  {
// perimeter--------------------------------------------------------------------
    case SEN_PERIM_LEFT:
      return perimeters.perimeter[Perimeter::LEFT].calcMagnitude();
      break;

      //case SEN_PERIM_RIGHT:
      //  return perimeters.perimeter[Perimeter::RIGHT].calcMagnitude();
      //  break;

// battery----------------------------------------------------------------------
    case SEN_BAT_VOLTAGE:
      ADCMan.read(PIN_VOLTAGE_MEASUREMENT);
      return ADCMan.read(PIN_BATTERY_VOLTAGE);
      break;

    case SEN_CHG_VOLTAGE:
      return ADCMan.read(PIN_CHARGE_VOLTAGE);
      break;

    case SEN_CHG_CURRENT:
      return ADCMan.read(PIN_CHARGE_CURRENT);
      break;

// lawn detector----------------------------------------------------------------
    //case SEN_LAWN_FRONT:
    //  return (measureLawnCapacity(pinLawnFrontSend, pinLawnFrontRecv));
    //  break;
    //case SEN_LAWN_BACK:
    //  return (measureLawnCapacity(PIN_LAWN_BACK_SEND, PIN_LAWN_BACK_RECV));
    //  break;

// imu--------------------------------------------------------------------------
    //case SEN_IMU:
    //  imuYaw = imu.ypr.yaw;
    //  imuPitch = imu.ypr.pitch;
    //  imuRoll = imu.ypr.roll;
    //  break;

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

void Mower::setActuator(char type, int value)
{
  switch (type)
  {
    case ACT_BUZZER:
      if (value == 0)
      {
        noTone(PIN_BUZZER);
      }
      else
      {
        tone(PIN_BUZZER, value);
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
        addErrorCounter(ERR_RTC_COMM);
        setNextState(STATE_ERROR);
      }
      break;

    case ACT_CHGRELAY:
      digitalWrite(PIN_CHARGE_RELAY, value);
      break;

      //case ACT_CHGRELAY:
    //  digitalWrite(pinChargeRelay, !value);
    //  break;

    case ACT_BATTERY_SW:
      digitalWrite(PIN_BATTERY_SWITCH, value);
      break;
  }
}

void Mower::configureBluetooth(boolean quick)
{
  BluetoothConfig bt;
  bt.setParams(name, PFOD_PIN, PFOD_BAUDRATE, quick);
}

#endif

