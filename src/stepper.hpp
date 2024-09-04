
//      ******************************************************************
//      *                                                                *
//      *                    Header file for ESP-FlexyStepper            *
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

// This library is based on the works of Stan Reifel in his FlexyStepper library:
// https://github.com/Stan-Reifel/FlexyStepper

//      ******************************************************************
//      *                                                                *
//      *       Stepper                                                  *
//      *       based on ESP-FlexyStepper by Paul Kerspe                 *
//      *                                                                *
//      ******************************************************************

#ifndef _Stepper_h
#define _Stepper_h

#include <Arduino.h>
#include <stdlib.h>
#include <limits.h>
#include "io/digital_output.hpp"

//typedef unsigned long micros_t;
typedef uint32_t micros_t;
typedef void (*callbackFunction)(void);
typedef void (*positionCallbackFunction)(long);

class Stepper
{
public:
  explicit Stepper(
      DigitalOutput& stepPin = DIGITAL_OUTPUT_NONE,
      DigitalOutput& directionPin = DIGITAL_OUTPUT_NONE,
      DigitalOutput& enablePin = DIGITAL_OUTPUT_NONE,
      DigitalOutput& brakePin = DIGITAL_OUTPUT_NONE);
  virtual ~Stepper() = default;

  // IO setup and helper / debugging functions
  void connect_to_pins(DigitalOutput& stepPin, DigitalOutput& directionPin);
  void set_brake_pin(DigitalOutput& brakePin, byte activeState = Stepper::ACTIVE_HIGH);
  void setEnablePin(DigitalOutput& enablePin, byte activeState = Stepper::ACTIVE_LOW);
  void clear_limit_switch_active(void);
  bool motionComplete();
  int getDirectionOfMotion(void);
  bool isMovingTowardsHome(void);
  void emergencyStop(bool holdUntilReleased = false);
  void releaseEmergencyStop(void);
  void activateBrake(void);
  void deactivateBrake(void);
  bool isBrakeActive(void);
  virtual void enable_driver(void);
  virtual void disable_driver(void);
  bool isDriverEnabled(void);

  // the central function to calculate the next movment step signal
  virtual bool processMovement(micros_t now);

  // register function for callbacks
  void registerHomeReachedCallback(callbackFunction homeReachedCallbackFunction);
  void registerLimitReachedCallback(callbackFunction limitSwitchTriggerdCallbackFunction);
  void registerTargetPositionReachedCallback(positionCallbackFunction targetPositionReachedCallbackFunction);
  void registerEmergencyStopTriggeredCallback(callbackFunction emergencyStopTriggerdCallbackFunction);
  void registerEmergencyStopReleasedCallback(callbackFunction emergencyStopReleasedCallbackFunction);

  // configuration functions
  void set_steps_per_millimeter(float motorStepPerMillimeter);
  void set_steps_per_revolution(float motorStepPerRevolution);
  void set_speed_in_steps_per_second(float speedInStepsPerSecond);
  void set_speed_in_millimeters_per_second(float speedInMillimetersPerSecond);
  void set_speed_in_revolutions_per_second(float speedInRevolutionsPerSecond);
  void set_acceleration_in_millimeters_per_second_per_second(float accelerationInMillimetersPerSecondPerSecond);
  void set_acceleration_in_revolutions_per_second_per_second(float accelerationInRevolutionsPerSecondPerSecond);
  void set_deceleration_in_millimeters_per_second_per_second(float decelerationInMillimetersPerSecondPerSecond);
  void set_deceleration_in_revolutions_per_second_per_second(float decelerationInRevolutionsPerSecondPerSecond);
  void set_acceleration_in_steps_per_second_per_second(float accelerationInStepsPerSecondPerSecond);
  void set_deceleration_in_steps_per_second_per_second(float decelerationInStepsPerSecondPerSecond);
  void set_direction_to_home(signed char directionTowardHome);
  void set_limit_switch_active(signed char limitSwitchType);

  void set_brake_engage_delay_ms(unsigned long);
  void set_brake_release_delay_ms(signed long);

  float get_current_velocity_in_steps_per_second();
  float get_current_velocity_in_revolutions_per_second();
  float get_current_velocity_in_millimeters_per_second(void);

  float get_configured_acceleration_in_steps_per_second_per_second();
  float get_configured_acceleration_in_revolutions_per_second_per_second();
  float get_configured_acceleration_in_millimeters_per_second_per_second();

  float get_configured_deceleration_in_steps_per_second_per_second();
  float get_configured_deceleration_in_revolutions_per_second_per_second();
  float get_configured_deceleration_in_millimeters_per_second_per_second();

  // positioning functions
  void set_current_position_in_steps(long currentPositionInSteps);
  void set_current_position_in_millimeters(float currentPositionInMillimeters);
  void set_current_position_in_revolutions(float currentPositionInRevolutions);

  long get_current_position_in_steps();
  float getCurrentPositionInRevolutions();
  float getCurrentPositionInMillimeters();

  void start_jogging(signed char direction);
  void stop_jogging();

  void setCurrentPositionAsHomeAndStop(void);
  void setTargetPositionToStop();
  long getDistanceToTargetSigned(void);

  void set_target_position_in_steps(long absolutePositionToMoveToInSteps);
  void setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
  void setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
  void setTargetPositionRelativeInSteps(long distanceToMoveInSteps);
  void setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters);
  void setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions);

  long get_target_position_in_steps();
  float getTargetPositionInMillimeters();
  float getTargetPositionInRevolutions();

  static const signed char LIMIT_SWITCH_BEGIN = -1;
  static const signed char LIMIT_SWITCH_END = 1;
  static const signed char LIMIT_SWITCH_COMBINED_BEGIN_AND_END = 2;
  static const byte ACTIVE_HIGH = 1;
  static const byte ACTIVE_LOW = 2;

private:
  callbackFunction _homeReachedCallback = NULL;
  callbackFunction _limitTriggeredCallback = NULL;
  callbackFunction _emergencyStopTriggeredCallback = NULL;
  callbackFunction _emergencyStopReleasedCallback = NULL;
  positionCallbackFunction _targetPositionReachedCallback = NULL;
  callbackFunction _callbackFunctionForGoToLimit = NULL;

  void DeterminePeriodOfNextStep();
  void triggerBrakeIfNeededOrSetTimeout(void);

  DigitalOutput& m_step_pin;
  DigitalOutput& m_direction_pin;
  DigitalOutput& m_enable_pin;
  DigitalOutput& m_brake_pin;
  volatile byte brakePinActiveState = ACTIVE_HIGH;
  volatile byte enablePinActiveState = ACTIVE_HIGH;
  volatile unsigned long _brakeEngageDelayMs = 0;
  volatile signed long _brakeReleaseDelayMs = -1;
  volatile unsigned long _timeToEngangeBrake = LONG_MAX;
  volatile unsigned long _timeToReleaseBrake = LONG_MAX;
  volatile bool _isBrakeConfigured = false;
  volatile bool _isEnableConfigured = false;
  volatile bool _hasMovementOccuredSinceLastBrakeRelease = true;

  volatile bool _isBrakeActive = false;
  volatile bool _isDriverEnabled = false;
  volatile float stepsPerMillimeter;
  volatile float stepsPerRevolution;
  volatile int directionOfMotion;
  volatile signed long currentPosition_InSteps;
  volatile signed long targetPosition_InSteps;
  volatile float desiredSpeed_InStepsPerSecond{};
  volatile float desiredPeriod_InUSPerStep{};
  volatile float acceleration_InStepsPerSecondPerSecond{};
  volatile float acceleration_InStepsPerUSPerUS{};
  volatile float deceleration_InStepsPerSecondPerSecond{};
  volatile float deceleration_InStepsPerUSPerUS{};
  volatile float periodOfSlowestStep_InUS{};
  volatile float minimumPeriodForAStoppedMotion{};
  volatile float nextStepPeriod_InUS;
  volatile micros_t lastStepTime_InUS;
  volatile float currentStepPeriod_InUS;
  volatile bool emergencyStopActive;
  volatile bool holdEmergencyStopUntilExplicitRelease;
  volatile signed char directionTowardsHome;
  volatile signed char lastStepDirectionBeforeLimitSwitchTrigger;
  // true if the current stepper position equals the homing position
  volatile bool isCurrentlyHomed;
  volatile bool isOnWayToHome = false;
  volatile bool isOnWayToLimit = false;
  volatile bool firstProcessingAfterTargetReached = false;
  // The type ID of the limit switch type that is active. possible values are LIMIT_SWITCH_BEGIN (-1) or LIMIT_SWITCH_END (1) or LIMIT_SWITCH_COMBINED_BEGIN_AND_END (2) or 0 if no limit switch is active
  volatile signed char activeLimitSwitch;
  volatile bool limitSwitchCheckPeformed;
  // 0 if the the stepper is allowed to move in both directions (e.g. no limit or homing switch triggered), otherwise indicated which direction is currently not allowed for further movement
  volatile signed char disallowedDirection;
};

// ------------------------------------ End ---------------------------------
#endif // _Stepper_h
