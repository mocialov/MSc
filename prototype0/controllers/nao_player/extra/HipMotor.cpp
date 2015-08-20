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

#include "HipMotor.hpp"
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

HipMotor::HipMotor(int step) : Device("HipYawPitch") {

  mActuator = Singletons::model()->angleActuator("LHipYawPitch");
  mLeftTag = wb_robot_get_device("LHipYawPitch");
  mRightTag = wb_robot_get_device("RHipYawPitch");
  
  if (mLeftTag) {
    mMaxLeftPosition = wb_motor_get_max_position(mLeftTag);
    mMinLeftPosition = wb_motor_get_min_position(mLeftTag);
    wb_motor_enable_position(mLeftTag, step);
  } else {
    mMaxLeftPosition = 0.0;
    mMinLeftPosition = 0.0;
  }
  
  if (mRightTag) {
    mMaxRightPosition = wb_motor_get_max_position(mRightTag);
    mMinRightPosition = wb_motor_get_min_position(mRightTag);
  } else {
    mMaxRightPosition = 0.0;
    mMinRightPosition = 0.0;
  }

  mSensor = Singletons::model()->angleSensor("LHipYawPitch");
  if (mSensor)
    Singletons::hal()->sendAngleSensorValue(mSensor, 0.0);
  else
    cerr << "Sim::AngleSensor not found for motor: LHipYawPitch\n";
}

HipMotor::~HipMotor() {
}

void HipMotor::update() {

  // effective position feedback
  if (mSensor) {
    double feedback = wb_motor_get_position(mLeftTag);
    if (! Singletons::hal()->sendAngleSensorValue(mSensor, (float)feedback))
      cerr << "Sim::HALInterface::sendAngleSensorValue() failed.\n";
  }

  // set target position
  double target = Singletons::hal()->fetchAngleActuatorValue(mActuator);
  if (isnan(target))
    target = mActuator->startValue();
  
  if (mLeftTag)
    wb_motor_set_position(mLeftTag, clamp(target, mMinLeftPosition, mMaxLeftPosition));
  
  if (mRightTag)
    wb_motor_set_position(mRightTag, clamp(target, mMinRightPosition, mMaxRightPosition));
}
