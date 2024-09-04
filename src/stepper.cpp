
//      ******************************************************************
//      *                                                                *
//      *                    ESP-FlexyStepper                            *
//      *                                                                *
//      *            Paul Kerspe                     4.6.2020            *
//      *       based on the concept of FlexyStepper by Stan Reifel      *
//      *                                                                *
//      ******************************************************************

// MIT License
//
// Copyright (c) 2020 Paul Kerspe
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

//
// This library is used to control one or more stepper motors.  It requires a
// stepper driver board that has a Step and Direction interface.  The motors are
// accelerated and decelerated as they travel to the final position.  This driver
// supports changing the target position, speed or rate of acceleration while a
// motion is in progress.
//
// for more details and a manual on how to use it, check the README.md on github and the provided examples
// https://github.com/pkerspe/ESP-FlexyStepper/blob/master/README.md
//
// This library is based on the works of Stan Reifel in his FlexyStepper library:
// https://github.com/Stan-Reifel/FlexyStepper
//

#include "stepper.hpp"

#include <cmath>
#include <utility>

//
// direction signal level for "step and direction"
//
#define POSITIVE_DIRECTION LOW
#define NEGATIVE_DIRECTION HIGH

#define RUN_CURRENT_PERCENT 50

// ---------------------------------------------------------------------------------
//                                  Setup functions
// ---------------------------------------------------------------------------------

//
// constructor for the stepper class
//
Stepper::Stepper(DigitalOutput &stepPin, DigitalOutput &directionPin, DigitalOutput &enablePin,
                 DigitalOutput &brakePin) :
                                            m_step_pin(stepPin), m_direction_pin(directionPin), m_enable_pin(enablePin),
                                            m_brake_pin(brakePin), stepsPerMillimeter(25.0), stepsPerRevolution(200L), directionOfMotion(0),
                                            currentPosition_InSteps(0), targetPosition_InSteps(0), nextStepPeriod_InUS(0), lastStepTime_InUS(0),
                                            currentStepPeriod_InUS(0), emergencyStopActive(false), holdEmergencyStopUntilExplicitRelease(false),
                                            directionTowardsHome(-1), lastStepDirectionBeforeLimitSwitchTrigger(0), isCurrentlyHomed(false),
                                            activeLimitSwitch(0), limitSwitchCheckPeformed(false), disallowedDirection(0)
{
  //set_speed_in_steps_per_second(16000); // at 24V
  set_speed_in_steps_per_second(9000); // at 12V
  set_acceleration_in_steps_per_second_per_second(3000.0);
  set_deceleration_in_steps_per_second_per_second(3000.0);
  _isEnableConfigured = m_enable_pin.valid();
}

/**
 * get the distance in steps to the currently set target position.
 * 0 is returned if the stepper is already at the target position.
 * The returned value is signed, depending on the direction to move to reach the target
 */
long Stepper::getDistanceToTargetSigned() { return (this->targetPosition_InSteps - this->currentPosition_InSteps); }

/**
 * perform an emergency stop, causing all movements to be canceled instantly
 * the optional parameter 'holdUntilReleased' allows to define if the emergency stop shall only affect the current
 * motion (if any) or if it should hold the emergency stop status (kind of a latching functionality) until the
 * releaseEmergencyStop() function is called explicitly. Default for holdUntilReleased is false (if parameter is
 * omitted)
 */
void Stepper::emergencyStop(bool holdUntilReleased)
{
  this->holdEmergencyStopUntilExplicitRelease = holdUntilReleased;
  this->emergencyStopActive = (!this->motionComplete() || this->holdEmergencyStopUntilExplicitRelease);
  if (this->_emergencyStopTriggeredCallback)
  {
    this->_emergencyStopTriggeredCallback();
  }
}

/**
 * releases an emergency stop that has previously been engaged using a call to emergencyStop(true)
 */
void Stepper::releaseEmergencyStop()
{
  this->emergencyStopActive = false;
  if (this->_emergencyStopReleasedCallback)
  {
    this->_emergencyStopReleasedCallback();
  }
}

/**
 *  configure the direction in which to move to reach the home position
 *  Accepts 1 or -1 as allowed values. Other values will be ignored
 */
void Stepper::set_direction_to_home(signed char directionTowardHome)
{
  if (directionTowardHome == -1 || directionTowardHome == 1)
  {
    this->directionTowardsHome = directionTowardHome;
  }
}

/**
 * Notification of an externally detected limit switch activation
 * Accepts LIMIT_SWITCH_BEGIN (-1) or LIMIT_SWITCH_END (1) as parameter values to indicate
 * whether the limit switch near the begin (direction of home position) or at the end of the movement has ben triggered.
 * It is strongly recommended to perform debouncing before calling this function to prevent issues when button is
 * released and re-triggering the limit switch function
 */
void Stepper::set_limit_switch_active(signed char limitSwitchType)
{
  if (limitSwitchType == LIMIT_SWITCH_BEGIN || limitSwitchType == LIMIT_SWITCH_END ||
      limitSwitchType == LIMIT_SWITCH_COMBINED_BEGIN_AND_END)
  {
    this->activeLimitSwitch = limitSwitchType;
    this->limitSwitchCheckPeformed = false; // set flag for newly set limit switch trigger
    if (this->_limitTriggeredCallback)
    {
      this->_limitTriggeredCallback();
      // TODO: this function is called from within a ISR in ESPStepperMotorServer thus we should try to delay
      // calling of the callback to the background task / process Steps function
    }
  }
}

/**
 * clear the limit switch flag to allow movement in both directions again
 */
void Stepper::clear_limit_switch_active() { this->activeLimitSwitch = 0; }

/**
 * get the current direction of motion of the connected stepper motor
 * returns 1 for "forward" motion
 * returns -1 for "backward" motion
 * returns 0 if the stepper has reached its destination position and is not moving anymore
 */
int Stepper::getDirectionOfMotion(void) { return this->directionOfMotion; }

/**
 * returns true if the stepper is currently in motion and moving in the direction of the home position.
 * Depends on the settings of set_direction_to_home() which defines where "home" is...a rather philosophical question
 * :-)
 */
bool Stepper::isMovingTowardsHome() { return (this->directionOfMotion == this->directionTowardsHome); }

/*
 * connect the stepper object to the IO pins
 * stepPinNumber = IO pin number for the Step signal
 * directionPinNumber = IO pin number for the direction signal
 */
void Stepper::connect_to_pins(DigitalOutput &stepPin, DigitalOutput &directionPin)
{
  m_step_pin = stepPin;
  m_direction_pin = directionPin;

  // configure the IO pins
  m_step_pin.low();

  if (m_direction_pin.valid())
  {
    m_direction_pin.low();
  }
}

/*
 * setup an IO pin to trigger an external brake for the motor.
 * This is an optional step, set to -1 to disable this function (which is default)
 * the active state parameter defines if the external brake is configured in an active high (pin goes high to enable the
 * brake) or active low (pin goes low to activate the brake) setup. active high = 1, active low = 2 Will be set to
 * active high by default or if an invalid value is given
 */
void Stepper::set_brake_pin(DigitalOutput &brakePin, byte activeState)
{
  m_brake_pin = brakePin;
  if (activeState == Stepper::ACTIVE_HIGH || activeState == Stepper::ACTIVE_LOW)
  {
    brakePinActiveState = activeState;
  }
  else
  {
    brakePinActiveState = Stepper::ACTIVE_HIGH;
  }

  if (m_brake_pin.valid())
  {
    deactivateBrake();
    _isBrakeConfigured = true;
  }
  else
  {
    _isBrakeConfigured = false;
  }
}

/*
 * setup an IO pin to enable the motors driver
 * This is an optional step, set to -1 to disable this function (which is default)
 * the active state parameter defines if the enable pin is configured in an active high (pin goes high to enable the
 * driver) or active low (pin goes low to activate the driver) setup. active high = 1, active low = 2 Will be set to
 * active high by default or if an invalid value is given
 */
void Stepper::setEnablePin(DigitalOutput &enablePin, byte activeState)
{
  m_enable_pin = enablePin;
  if (activeState == Stepper::ACTIVE_HIGH || activeState == Stepper::ACTIVE_LOW)
  {
    enablePinActiveState = activeState;
  }
  else
  {
    enablePinActiveState = Stepper::ACTIVE_LOW;
  }

  _isEnableConfigured = m_enable_pin.valid();
}

/**
 * set a delay in milliseconds between stopping the stepper motor and engaging the physical brake (trigger the eternal
 * pin configured via setBrakePin() ). Default is 0, resulting in immediate triggering of the motor brake once the motor
 * stops moving. This value does NOT affect the triggering of the brake in case of an emergency stop. In this case the
 * brake will always get triggered without delay
 */
void Stepper::set_brake_engage_delay_ms(unsigned long delay) { this->_brakeEngageDelayMs = delay; }

/**
 * set a timeout in milliseconds after which the brake shall be released once triggered and no motion is performed by
 * the stepper motor. By default the value is -1 indicating, that the brake shall never be automatically released, as
 * long as the stepper motor is not moving to a new position. Value must be larger than 1 (Even though 1ms delay does
 * probably not make any sense since physical brakes have a delay that is most likely higher than that just to engage)
 */
void Stepper::set_brake_release_delay_ms(signed long delay)
{
  if (delay < 0)
  {
    this->_brakeReleaseDelayMs = -1;
  }
  else
  {
    this->_brakeReleaseDelayMs = delay;
  }
}

/**
 * activate (engage) the motor brake (if any is configured, otherwise will do nothing)
 */
void Stepper::activateBrake()
{
  if (this->_isBrakeConfigured)
  {
    m_brake_pin.set(brakePinActiveState == Stepper::ACTIVE_HIGH);
    this->_isBrakeActive = true;
    this->_timeToEngangeBrake = LONG_MAX;
  }
}

/**
 * deactivate (release) the motor brake (if any is configured, otherwise will do nothing)
 */
void Stepper::deactivateBrake()
{
  if (this->_isBrakeConfigured)
  {
    m_brake_pin.set(brakePinActiveState != Stepper::ACTIVE_HIGH);
    this->_isBrakeActive = false;
    this->_timeToReleaseBrake = LONG_MAX;
    this->_hasMovementOccuredSinceLastBrakeRelease = false;

    // TODO: add delay here if configured as to https://github.com/pkerspe/ESP-StepperMotor-Server/issues/16
  }
}

bool Stepper::isBrakeActive() { return this->_isBrakeActive; }

/**
 * activate (engage) the driver (if any is configured, otherwise will do nothing)
 */
void Stepper::enable_driver()
{
  if (_isEnableConfigured && !_isDriverEnabled)
  {
    m_enable_pin.set(enablePinActiveState == Stepper::ACTIVE_HIGH);
    //m_driver.setup();
    //m_driver.setRunCurrent(RUN_CURRENT_PERCENT);
    //m_driver.enableCoolStep();
    //m_driver.setMicrostepsPerStep(1);
    //m_driver.setStallGuardThreshold(255);
    //m_driver.moveUsingStepDirInterface();
    _isDriverEnabled = true;
  }
}

/**
 * deactivate (release) the driver (if any is configured, otherwise will do nothing)
 */
void Stepper::disable_driver()
{
  if (_isEnableConfigured && _isDriverEnabled)
  {
    m_enable_pin.set(this->enablePinActiveState != Stepper::ACTIVE_HIGH);
    _isDriverEnabled = false;
  }
}

bool Stepper::isDriverEnabled(void) { return this->_isDriverEnabled; }

// ---------------------------------------------------------------------------------
//                     Public functions with units in millimeters
// ---------------------------------------------------------------------------------

//
// set the number of steps the motor has per millimeters
//
void Stepper::set_steps_per_millimeter(float motorStepsPerMillimeter) { stepsPerMillimeter = motorStepsPerMillimeter; }

//
// get the current position of the motor in millimeters, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in millimeters returned
//
float Stepper::getCurrentPositionInMillimeters()
{
  return ((float)get_current_position_in_steps() / stepsPerMillimeter);
}

//
// set the current position of the motor in millimeters.
// Do not confuse this function with setTargetPositionInMillimeters(), it does not directly cause a motor movement per
// se. NOTE: if you called one of the move functions before (and by that setting a target position internally) you might
// experience that the motor starts to move after calling set_current_position_in_millimeters() in the case that the
// value of currentPositionInMillimeters is different from the target position of the stepper. If this is not intended,
// you should call setTargetPositionInMillimeters() with the same value as the setCurrentPositionInMillimeters()
// function directly before or after calling set_current_position_in_millimeters
//
void Stepper::set_current_position_in_millimeters(float currentPositionInMillimeters)
{
  set_current_position_in_steps(static_cast<long>(std::round(currentPositionInMillimeters * stepsPerMillimeter)));
}

//
// set the maximum speed, units in millimeters/second, this is the maximum speed
// reached while accelerating
//  Enter:  speedInMillimetersPerSecond = speed to accelerate up to, units in
//            millimeters/second
//
void Stepper::set_speed_in_millimeters_per_second(float speedInMillimetersPerSecond)
{
  set_speed_in_steps_per_second(speedInMillimetersPerSecond * stepsPerMillimeter);
}

//
// set the rate of acceleration, units in millimeters/second/second
//  Enter:  accelerationInMillimetersPerSecondPerSecond = rate of acceleration,
//          units in millimeters/second/second
//
void Stepper::set_acceleration_in_millimeters_per_second_per_second(float accelerationInMillimetersPerSecondPerSecond)
{
  set_acceleration_in_steps_per_second_per_second(accelerationInMillimetersPerSecondPerSecond * stepsPerMillimeter);
}

//
// set the rate of deceleration, units in millimeters/second/second
//  Enter:  decelerationInMillimetersPerSecondPerSecond = rate of deceleration,
//          units in millimeters/second/second
//
void Stepper::set_deceleration_in_millimeters_per_second_per_second(float decelerationInMillimetersPerSecondPerSecond)
{
  set_deceleration_in_steps_per_second_per_second(decelerationInMillimetersPerSecondPerSecond * stepsPerMillimeter);
}

//
// setup a move relative to the current position, units are in millimeters, no
// motion occurs until processMove() is called
//  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the
//          current position in millimeters
//
void Stepper::setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters)
{
  setTargetPositionRelativeInSteps((long)round(distanceToMoveInMillimeters * stepsPerMillimeter));
}

//
// setup a move, units are in millimeters, no motion occurs until processMove()
// is called
//  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to
//          move to in units of millimeters
//
void Stepper::setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters)
{
  set_target_position_in_steps((long)round(absolutePositionToMoveToInMillimeters * stepsPerMillimeter));
}

float Stepper::getTargetPositionInMillimeters() { return get_target_position_in_steps() / stepsPerMillimeter; }

//
// Get the current velocity of the motor in millimeters/second.  This functions is
// updated while it accelerates up and down in speed.  This is not the desired
// speed, but the speed the motor should be moving at the time the function is
// called.  This is a signed value and is negative when the motor is moving
// backwards.  Note: This speed will be incorrect if the desired velocity is set
// faster than this library can generate steps, or if the load on the motor is too
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float Stepper::get_current_velocity_in_millimeters_per_second()
{
  return (get_current_velocity_in_steps_per_second() / stepsPerMillimeter);
}

/*
access the acceleration/deceleration parameters set by user
*/

float Stepper::get_configured_acceleration_in_steps_per_second_per_second()
{
  return acceleration_InStepsPerSecondPerSecond;
}

float Stepper::get_configured_acceleration_in_revolutions_per_second_per_second()
{
  return acceleration_InStepsPerSecondPerSecond / stepsPerRevolution;
}

float Stepper::get_configured_acceleration_in_millimeters_per_second_per_second()
{
  return acceleration_InStepsPerSecondPerSecond / stepsPerMillimeter;
}

float Stepper::get_configured_deceleration_in_steps_per_second_per_second()
{
  return deceleration_InStepsPerSecondPerSecond;
}

float Stepper::get_configured_deceleration_in_revolutions_per_second_per_second()
{
  return deceleration_InStepsPerSecondPerSecond / stepsPerRevolution;
}

float Stepper::get_configured_deceleration_in_millimeters_per_second_per_second()
{
  return deceleration_InStepsPerSecondPerSecond / stepsPerMillimeter;
}

// ---------------------------------------------------------------------------------
//                     Public functions with units in revolutions
// ---------------------------------------------------------------------------------

//
// set the number of steps the motor has per revolution
//
void Stepper::set_steps_per_revolution(float motorStepPerRevolution) { stepsPerRevolution = motorStepPerRevolution; }

//
// get the current position of the motor in revolutions, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in revolutions returned
//
float Stepper::getCurrentPositionInRevolutions()
{
  return ((float)get_current_position_in_steps() / stepsPerRevolution);
}

//
// set the current position of the motor in revolutions, this does not move the
// Do not confuse this function with setTargetPositionInRevolutions(), it does not directly cause a motor movement per
// se. NOTE: if you called one of the move functions before (and by that setting a target position internally) you might
// experience that the motor starts to move after calling set_current_position_in_revolutions() in the case that the
// value of currentPositionInRevolutions is different from the target position of the stepper. If this is not intended,
// you should call setTargetPositionInRevolutions() with the same value as the setCurrentPositionInRevolutions()
// function directly before or after calling set_current_position_in_revolutions

void Stepper::set_current_position_in_revolutions(float currentPositionInRevolutions)
{
  set_current_position_in_steps((long)round(currentPositionInRevolutions * stepsPerRevolution));
}

//
// set the maximum speed, units in revolutions/second, this is the maximum speed
// reached while accelerating
//  Enter:  speedInRevolutionsPerSecond = speed to accelerate up to, units in
//            revolutions/second
//
void Stepper::set_speed_in_revolutions_per_second(float speedInRevolutionsPerSecond)
{
  set_speed_in_steps_per_second(speedInRevolutionsPerSecond * stepsPerRevolution);
}

//
// set the rate of acceleration, units in revolutions/second/second
//  Enter:  accelerationInRevolutionsPerSecondPerSecond = rate of acceleration,
//          units in revolutions/second/second
//
void Stepper::set_acceleration_in_revolutions_per_second_per_second(float accelerationInRevolutionsPerSecondPerSecond)
{
  set_acceleration_in_steps_per_second_per_second(accelerationInRevolutionsPerSecondPerSecond * stepsPerRevolution);
}

//
// set the rate of deceleration, units in revolutions/second/second
//  Enter:  decelerationInRevolutionsPerSecondPerSecond = rate of deceleration,
//          units in revolutions/second/second
//
void Stepper::set_deceleration_in_revolutions_per_second_per_second(float decelerationInRevolutionsPerSecondPerSecond)
{
  set_deceleration_in_steps_per_second_per_second(decelerationInRevolutionsPerSecondPerSecond * stepsPerRevolution);
}

//
// setup a move relative to the current position, units are in revolutions, no
// motion occurs until processMove() is called
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the
//            currentposition in revolutions
//
void Stepper::setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions)
{
  setTargetPositionRelativeInSteps((long)round(distanceToMoveInRevolutions * stepsPerRevolution));
}

//
// setup a move, units are in revolutions, no motion occurs until processMove()
// is called
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to
//          move to in units of revolutions
//
void Stepper::setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions)
{
  set_target_position_in_steps((long)round(absolutePositionToMoveToInRevolutions * stepsPerRevolution));
}

float Stepper::getTargetPositionInRevolutions() { return get_target_position_in_steps() / stepsPerRevolution; }

//
// Get the current velocity of the motor in revolutions/second.  This functions is
// updated while it accelerates up and down in speed.  This is not the desired
// speed, but the speed the motor should be moving at the time the function is
// called.  This is a signed value and is negative when the motor is moving
// backwards.  Note: This speed will be incorrect if the desired velocity is set
// faster than this library can generate steps, or if the load on the motor is too
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float Stepper::get_current_velocity_in_revolutions_per_second()
{
  return (get_current_velocity_in_steps_per_second() / stepsPerRevolution);
}

// ---------------------------------------------------------------------------------
//                        Public functions with units in steps
// ---------------------------------------------------------------------------------

//
// set the current position of the motor in steps, this does not move the motor
// currentPositionInSteps = the new position value of the motor in steps to be set internally for the current position
// Do not confuse this function with setTargetPositionInMillimeters(), it does not directly cause a motor movement per
// se. Notes: This function should only be called when the motor is stopped If you called one of the move functions
// before (and by that setting a target position internally) you might experience that the motor starts to move after
// calling set_current_position_in_steps() in the case that the value of currentPositionInSteps is different from the
// target position of the stepper. If this is not intended, you should call set_target_position_in_steps() with the same
// value as the setCurrentPositionInSteps() function directly before or after calling set_current_position_in_steps
//
void Stepper::set_current_position_in_steps(long currentPositionInSteps)
{
  currentPosition_InSteps = currentPositionInSteps;
}

//
// get the current position of the motor in steps, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in steps returned
//
long Stepper::get_current_position_in_steps() { return (currentPosition_InSteps); }

//
// set the maximum speed, units in steps/second, this is the maximum speed reached
// while accelerating
//  Enter:  speedInStepsPerSecond = speed to accelerate up to, units in steps/second
//
void Stepper::set_speed_in_steps_per_second(float speedInStepsPerSecond)
{
  desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
  desiredPeriod_InUSPerStep = 1000000.0 / desiredSpeed_InStepsPerSecond;
}

//
// set the rate of acceleration, units in steps/second/second
//  Enter:  accelerationInStepsPerSecondPerSecond = rate of acceleration, units in
//          steps/second/second
//
void Stepper::set_acceleration_in_steps_per_second_per_second(float accelerationInStepsPerSecondPerSecond)
{
  acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
  acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E12;

  periodOfSlowestStep_InUS = 1000000.0 / sqrt(2.0 * acceleration_InStepsPerSecondPerSecond);
  minimumPeriodForAStoppedMotion = periodOfSlowestStep_InUS / 2.8;
}

//
// set the rate of deceleration, units in steps/second/second
//  Enter:  decelerationInStepsPerSecondPerSecond = rate of deceleration, units in
//          steps/second/second
//
void Stepper::set_deceleration_in_steps_per_second_per_second(float decelerationInStepsPerSecondPerSecond)
{
  deceleration_InStepsPerSecondPerSecond = decelerationInStepsPerSecondPerSecond;
  deceleration_InStepsPerUSPerUS = deceleration_InStepsPerSecondPerSecond / 1E12;
}

/**
 * set the current position as the home position (Step count = 0)
 */
void Stepper::setCurrentPositionAsHomeAndStop()
{
  this->isOnWayToHome = false;
  this->currentStepPeriod_InUS = 0.0;
  this->nextStepPeriod_InUS = 0.0;
  this->directionOfMotion = 0;
  this->currentPosition_InSteps = 0;
  this->targetPosition_InSteps = 0;
  this->isCurrentlyHomed = true;
}

/**
 * register a callback function to be called whenever a movement to home has been completed (does not trigger when
 * movement passes by the home position)
 */
void Stepper::registerHomeReachedCallback(callbackFunction newFunction) { this->_homeReachedCallback = newFunction; }

/**
 * register a callback function to be called whenever a
 */
void Stepper::registerLimitReachedCallback(callbackFunction limitSwitchTriggerdCallbackFunction)
{
  this->_limitTriggeredCallback = limitSwitchTriggerdCallbackFunction;
}

/**
 * register a callback function to be called whenever a target position has been reached
 */
void Stepper::registerTargetPositionReachedCallback(positionCallbackFunction targetPositionReachedCallbackFunction)
{
  this->_targetPositionReachedCallback = targetPositionReachedCallbackFunction;
}

/**
 * register a callback function to be called whenever a emergency stop is triggered
 */
void Stepper::registerEmergencyStopTriggeredCallback(callbackFunction emergencyStopTriggerdCallbackFunction)
{
  this->_emergencyStopTriggeredCallback = emergencyStopTriggerdCallbackFunction;
}

/**
 * register a callback function to be called whenever the emergency stop switch is released
 */
void Stepper::registerEmergencyStopReleasedCallback(callbackFunction emergencyStopReleasedCallbackFunction)
{
  this->_emergencyStopReleasedCallback = emergencyStopReleasedCallbackFunction;
}

/**
 * start jogging (continuous movement without a fixed target position)
 * uses the currently set speed and acceleration settings
 * to stop the motion call the stop_jogging function.
 * Will also stop when the external limit switch has been triggered using set_limit_switch_active() or when the
 * emergencyStop function is triggered Warning: This function requires either a limit switch to be configured or manual
 * trigger of the stop_jogging/setTargetPositionToStop or emergencyStop function, otherwise the motor will never stop
 * jogging (which could of course also be an intended behavior)
 */
void Stepper::start_jogging(signed char direction) { this->set_target_position_in_steps(direction * 2000000000); }

/**
 * Stop jogging, basically an alias function for setTargetPositionToStop()
 */
void Stepper::stop_jogging() { this->setTargetPositionToStop(); }

//
// setup a move relative to the current position, units are in steps, no motion
// occurs until processMove() is called
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current
//            position in steps
//
void Stepper::setTargetPositionRelativeInSteps(long distanceToMoveInSteps)
{
  set_target_position_in_steps(currentPosition_InSteps + distanceToMoveInSteps);
}

//
// setup a move, units are in steps, no motion occurs until processMove() is called
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to
//            in units of steps
//
void Stepper::set_target_position_in_steps(long absolutePositionToMoveToInSteps)
{
  // abort potentially running homing movement
  this->isOnWayToHome = false;
  this->isOnWayToLimit = false;
  targetPosition_InSteps = absolutePositionToMoveToInSteps;
  this->firstProcessingAfterTargetReached = true;
}

long Stepper::get_target_position_in_steps() { return targetPosition_InSteps; }

//
// setup a "Stop" to begin the process of decelerating from the current velocity
// to zero, decelerating requires calls to processMove() until the move is complete
// Note: This function can be used to stop a motion initiated in units of steps
// or revolutions
//
void Stepper::setTargetPositionToStop()
{
  // abort potentially running homing movement
  this->isOnWayToHome = false;
  this->isOnWayToLimit = false;

  if (directionOfMotion == 0)
  {
    return;
  }

  long decelerationDistance_InSteps;

  //
  // move the target position so that the motor will begin deceleration now
  //
  decelerationDistance_InSteps =
      (long)round(5E11 / (deceleration_InStepsPerSecondPerSecond * currentStepPeriod_InUS * currentStepPeriod_InUS));

  if (directionOfMotion > 0)
    set_target_position_in_steps(currentPosition_InSteps + decelerationDistance_InSteps);
  else
    set_target_position_in_steps(currentPosition_InSteps - decelerationDistance_InSteps);
}

//
// if it is time, move one step
//  Exit:  true returned if movement complete, false returned not a final target position yet
//
bool Stepper::processMovement(micros_t now)
{
  signed long distanceToTarget_Signed = 0;

  if (emergencyStopActive)
  {
    // abort potentially running homing movement
    this->isOnWayToHome = false;
    this->isOnWayToLimit = false;

    currentStepPeriod_InUS = 0.0;
    nextStepPeriod_InUS = 0.0;
    directionOfMotion = 0;
    targetPosition_InSteps = currentPosition_InSteps;

    // activate brake (if configured) directly due to emergency stop if not already active
    if (this->_isBrakeConfigured && !this->_isBrakeActive)
    {
      this->activateBrake();
    }

    if (!this->holdEmergencyStopUntilExplicitRelease)
    {
      emergencyStopActive = false;
    }
    return (true);
  }

  // check if delayed brake shall be engaged / released
  if (this->_timeToEngangeBrake != LONG_MAX && this->_timeToEngangeBrake <= millis())
  {
    this->activateBrake();
  }
  else if (this->_timeToReleaseBrake != LONG_MAX && this->_timeToReleaseBrake <= millis())
  {
    this->deactivateBrake();
  }

  // check if limit switch flag is active
  if (this->activeLimitSwitch != 0)
  {
    distanceToTarget_Signed = targetPosition_InSteps - currentPosition_InSteps;

    if (!this->limitSwitchCheckPeformed)
    {
      this->limitSwitchCheckPeformed = true;

      // a limit switch is active, so movement is only allowed in one direction (away from the switch)
      if (this->activeLimitSwitch == this->LIMIT_SWITCH_BEGIN)
      {
        this->disallowedDirection = this->directionTowardsHome;
      }
      else if (this->activeLimitSwitch == this->LIMIT_SWITCH_END)
      {
        this->disallowedDirection = this->directionTowardsHome * -1;
      }
      else if (this->activeLimitSwitch == this->LIMIT_SWITCH_COMBINED_BEGIN_AND_END)
      {
        // limit switches are paired together, so we need to try to figure out by checking which one it is, by
        // using the last used step direction
        if (distanceToTarget_Signed > 0)
        {
          this->lastStepDirectionBeforeLimitSwitchTrigger = 1;
          this->disallowedDirection = 1;
        }
        else if (distanceToTarget_Signed < 0)
        {
          this->lastStepDirectionBeforeLimitSwitchTrigger = -1;
          this->disallowedDirection = -1;
        }
      }

      // movement has been triggered by goToLimitAndSetAsHome() function. so once the limit switch has been
      // triggered we have reached the limit and need to set it as home
      if (this->isOnWayToHome)
      {
        this->setCurrentPositionAsHomeAndStop(); // clear isOnWayToHome flag and stop motion

        if (this->_homeReachedCallback != NULL)
        {
          this->_homeReachedCallback();
        }
        // activate brake (or schedule activation) since we reached the final position
        if (this->_isBrakeConfigured && !this->_isBrakeActive)
        {
          this->triggerBrakeIfNeededOrSetTimeout();
        }
        return true;
      }
    }

    // check if further movement is allowed
    if ((this->disallowedDirection == 1 && distanceToTarget_Signed > 0) ||
        (this->disallowedDirection == -1 && distanceToTarget_Signed < 0))
    {
      // limit switch is active and movement in request direction is not allowed
      currentStepPeriod_InUS = 0.0;
      nextStepPeriod_InUS = 0.0;
      directionOfMotion = 0;
      targetPosition_InSteps = currentPosition_InSteps;
      // activate brake (or schedule activation) since limit is active for requested direction
      if (this->_isBrakeConfigured && !this->_isBrakeActive)
      {
        this->triggerBrakeIfNeededOrSetTimeout();
      }
      return true;
    }
  }

  micros_t currentTime_InUS;
  micros_t periodSinceLastStep_InUS;

  //
  // check if currently stopped
  //
  if (directionOfMotion == 0)
  {
    distanceToTarget_Signed = targetPosition_InSteps - currentPosition_InSteps;
    // check if target position in a positive direction
    if (distanceToTarget_Signed > 0)
    {
      directionOfMotion = 1;
      m_direction_pin.set(POSITIVE_DIRECTION);
      nextStepPeriod_InUS = periodOfSlowestStep_InUS;
      lastStepTime_InUS = now;
      lastStepDirectionBeforeLimitSwitchTrigger = directionOfMotion;
      return (false);
    }

    // check if target position in a negative direction
    else if (distanceToTarget_Signed < 0)
    {
      directionOfMotion = -1;
      m_direction_pin.set(NEGATIVE_DIRECTION);
      nextStepPeriod_InUS = periodOfSlowestStep_InUS;
      lastStepTime_InUS = now;
      lastStepDirectionBeforeLimitSwitchTrigger = directionOfMotion;
      return (false);
    }
    else
    {
      this->lastStepDirectionBeforeLimitSwitchTrigger = 0;

      if (this->firstProcessingAfterTargetReached)
      {
        firstProcessingAfterTargetReached = false;
        if (this->_targetPositionReachedCallback)
        {
          this->_targetPositionReachedCallback(currentPosition_InSteps);
        }
      }

      // activate brake since motor is stopped
      if (this->_isBrakeConfigured && !this->_isBrakeActive && this->_hasMovementOccuredSinceLastBrakeRelease)
      {
        this->triggerBrakeIfNeededOrSetTimeout();
      }
      return (true);
    }
  }

  // determine how much time has elapsed since the last step (Note 1: this method
  // works even if the time has wrapped. Note 2: all variables must be unsigned)
  currentTime_InUS = now;
  periodSinceLastStep_InUS = currentTime_InUS - lastStepTime_InUS;
  // if it is not time for the next step, return
  if (periodSinceLastStep_InUS < (micros_t)nextStepPeriod_InUS)
    return (false);

  // we have to move, so deactivate brake (if configured at all) immediately
  if (this->_isBrakeConfigured && this->_isBrakeActive)
  {
    this->deactivateBrake();
  }

  // execute the step on the rising edge
  m_step_pin.set(HIGH);

  // update the current position and speed
  currentPosition_InSteps += directionOfMotion;
  currentStepPeriod_InUS = nextStepPeriod_InUS;

  // remember the time that this step occured
  lastStepTime_InUS = currentTime_InUS;

  // figure out how long before the next step
  DeterminePeriodOfNextStep();

  this->_hasMovementOccuredSinceLastBrakeRelease = true;
  // return the step line low
  m_step_pin.set(LOW);

  // check if the move has reached its final target position, return true if all
  // done
  if (currentPosition_InSteps == targetPosition_InSteps)
  {
    // at final position, make sure the motor is not going too fast
    if (nextStepPeriod_InUS >= minimumPeriodForAStoppedMotion)
    {
      currentStepPeriod_InUS = 0.0;
      nextStepPeriod_InUS = 0.0;
      directionOfMotion = 0;
      this->lastStepDirectionBeforeLimitSwitchTrigger = 0;

      if (this->firstProcessingAfterTargetReached)
      {
        firstProcessingAfterTargetReached = false;
        if (this->_targetPositionReachedCallback)
        {
          this->_targetPositionReachedCallback(currentPosition_InSteps);
        }
        // activate brake since we reached the final position
        if (this->_isBrakeConfigured && !this->_isBrakeActive)
        {
          this->triggerBrakeIfNeededOrSetTimeout();
        }
      }
      return (true);
    }
  }
  return (false);
}

/**
 * internal helper to determine if brake shall be activated (if configured at all) or if a delay needs to be set
 */
void Stepper::triggerBrakeIfNeededOrSetTimeout()
{
  // check if break is already set or a timeout has already been set
  if (this->_isBrakeConfigured && !this->_isBrakeActive && this->_timeToEngangeBrake == LONG_MAX)
  {
    if (this->_brakeReleaseDelayMs > 0 && this->_hasMovementOccuredSinceLastBrakeRelease)
    {
      this->_timeToReleaseBrake = millis() + this->_brakeReleaseDelayMs;
    }

    if (this->_brakeEngageDelayMs == 0)
    {
      this->activateBrake();
    }
    else
    {
      this->_timeToEngangeBrake = millis() + this->_brakeEngageDelayMs;
    }
  }
}

// Get the current velocity of the motor in steps/second.  This functions is
// updated while it accelerates up and down in speed.  This is not the desired
// speed, but the speed the motor should be moving at the time the function is
// called.  This is a signed value and is negative when the motor is moving
// backwards.  Note: This speed will be incorrect if the desired velocity is set
// faster than this library can generate steps, or if the load on the motor is too
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float Stepper::get_current_velocity_in_steps_per_second()
{
  if (currentStepPeriod_InUS == 0.0)
    return (0);
  else
  {
    if (directionOfMotion > 0)
      return (1000000.0 / currentStepPeriod_InUS);
    else
      return (-1000000.0 / currentStepPeriod_InUS);
  }
}

//
// check if the motor has competed its move to the target position
//  Exit:  true returned if the stepper is at the target position
//
bool Stepper::motionComplete()
{
  if ((directionOfMotion == 0) && (currentPosition_InSteps == targetPosition_InSteps))
    return (true);
  else
    return (false);
}

//
// determine the period for the next step, either speed up a little, slow down a
// little or go the same speed
//
void Stepper::DeterminePeriodOfNextStep()
{
  long distanceToTarget_Signed;
  long distanceToTarget_Unsigned;
  long decelerationDistance_InSteps;
  float currentStepPeriodSquared;
  bool speedUpFlag = false;
  bool slowDownFlag = false;
  bool targetInPositiveDirectionFlag = false;
  bool targetInNegativeDirectionFlag = false;

  //
  // determine the distance to the target position
  //
  distanceToTarget_Signed = targetPosition_InSteps - currentPosition_InSteps;
  if (distanceToTarget_Signed >= 0L)
  {
    distanceToTarget_Unsigned = distanceToTarget_Signed;
    targetInPositiveDirectionFlag = true;
  }
  else
  {
    distanceToTarget_Unsigned = -distanceToTarget_Signed;
    targetInNegativeDirectionFlag = true;
  }

  //
  // determine the number of steps needed to go from the current speed down to a
  // velocity of 0, Steps = Velocity^2 / (2 * Deceleration)
  //
  currentStepPeriodSquared = currentStepPeriod_InUS * currentStepPeriod_InUS;
  decelerationDistance_InSteps =
      (long)round(5E11 / (deceleration_InStepsPerSecondPerSecond * currentStepPeriodSquared));

  //
  // check if: Moving in a positive direction & Moving toward the target
  //    (directionOfMotion == 1) && (distanceToTarget_Signed > 0)
  //
  if ((directionOfMotion == 1) && (targetInPositiveDirectionFlag))
  {
    //
    // check if need to start slowing down as we reach the target, or if we
    // need to slow down because we are going too fast
    //
    if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) ||
        (nextStepPeriod_InUS < desiredPeriod_InUSPerStep))
      slowDownFlag = true;
    else
      speedUpFlag = true;
  }

  //
  // check if: Moving in a positive direction & Moving away from the target
  //    (directionOfMotion == 1) && (distanceToTarget_Signed < 0)
  //
  else if ((directionOfMotion == 1) && (targetInNegativeDirectionFlag))
  {
    //
    // need to slow down, then reverse direction
    //
    if (currentStepPeriod_InUS < periodOfSlowestStep_InUS)
    {
      slowDownFlag = true;
    }
    else
    {
      directionOfMotion = -1;
      m_direction_pin.set(NEGATIVE_DIRECTION);
    }
  }

  //
  // check if: Moving in a negative direction & Moving toward the target
  //    (directionOfMotion == -1) && (distanceToTarget_Signed < 0)
  //
  else if ((directionOfMotion == -1) && (targetInNegativeDirectionFlag))
  {
    //
    // check if need to start slowing down as we reach the target, or if we
    // need to slow down because we are going too fast
    //
    if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) ||
        (nextStepPeriod_InUS < desiredPeriod_InUSPerStep))
      slowDownFlag = true;
    else
      speedUpFlag = true;
  }

  //
  // check if: Moving in a negative direction & Moving away from the target
  //    (directionOfMotion == -1) && (distanceToTarget_Signed > 0)
  //
  else if ((directionOfMotion == -1) && (targetInPositiveDirectionFlag))
  {
    //
    // need to slow down, then reverse direction
    //
    if (currentStepPeriod_InUS < periodOfSlowestStep_InUS)
    {
      slowDownFlag = true;
    }
    else
    {
      directionOfMotion = 1;
      m_direction_pin.set(POSITIVE_DIRECTION);
    }
  }

  //
  // check if accelerating
  //
  if (speedUpFlag)
  {
    //
    // StepPeriod = StepPeriod(1 - a * StepPeriod^2)
    //
    nextStepPeriod_InUS =
        currentStepPeriod_InUS - acceleration_InStepsPerUSPerUS * currentStepPeriodSquared * currentStepPeriod_InUS;

    if (nextStepPeriod_InUS < desiredPeriod_InUSPerStep)
      nextStepPeriod_InUS = desiredPeriod_InUSPerStep;
  }

  //
  // check if decelerating
  //
  if (slowDownFlag)
  {
    //
    // StepPeriod = StepPeriod(1 + a * StepPeriod^2)
    //
    nextStepPeriod_InUS =
        currentStepPeriod_InUS + deceleration_InStepsPerUSPerUS * currentStepPeriodSquared * currentStepPeriod_InUS;

    if (nextStepPeriod_InUS > periodOfSlowestStep_InUS)
      nextStepPeriod_InUS = periodOfSlowestStep_InUS;
  }
}

// -------------------------------------- End --------------------------------------
