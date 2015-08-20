/*
    This file is part of naoqisim.

    naoqisim is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    naoqisim is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with naoqisim.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Motor.hpp"
#include "Singletons.hpp"
#include "util.hpp"
#include <webots/robot.h>
#include <webots/motor.h>
#include <iostream>
#include <cassert>

#ifdef _MSC_VER
#define isnan _isnan
#endif

using namespace Sim;
using namespace std;
using namespace util;

Motor::Motor(const AngleActuator *actuator, int step) :Device(actuator->name()) {
  
  mActuator = actuator;

  mTag = wb_robot_get_device(name().c_str());
  if (! mTag) {
    cerr << "Webots Motor not found for Sim::AngleActuator: " << name() << "\n";
    mMaxPosition = 0.0;
    mMinPosition = 0.0;
  } else {
    mMaxPosition = wb_motor_get_max_position(mTag);
    mMinPosition = wb_motor_get_min_position(mTag);
    wb_motor_enable_position(mTag, step);
  }
  
  mSensor = Singletons::model()->angleSensor(name());
  if (mSensor)
    Singletons::hal()->sendAngleSensorValue(mSensor, 0.0);
  else
    cerr << "Sim::AngleSensor not found for motor: " << name() << "\n";
}

Motor::~Motor() {
}

void Motor::update() {
    
  if (! mTag) return;

  // effective position feedback
  if (mSensor) {
    double feedback = wb_motor_get_position(mTag);
    if (! Singletons::hal()->sendAngleSensorValue(mSensor, (float)feedback))
      cerr << "Sim::HALInterface::sendAngleSensorValue() failed.\n";
  }

  // set target position
  double target = Singletons::hal()->fetchAngleActuatorValue(mActuator);
  if (isnan(target)) target = mActuator->startValue();
  wb_motor_set_position(mTag, clamp(target, mMinPosition, mMaxPosition));
}
