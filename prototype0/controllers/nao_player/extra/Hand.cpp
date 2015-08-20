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

#include "Hand.hpp"
#include "Singletons.hpp"
#include <webots/robot.h>
#include <webots/motor.h>
#include <iostream>
#include <stdio.h>

#ifdef _MSC_VER
#define isnan _isnan
#endif

using namespace std;
using namespace Sim;

Hand::Hand(const CoupledActuator *actuator, int step) : Device(actuator->name()) {

  mActuator = actuator;
  
  const char *deviceFormat = NULL;
  if (name() == "LHand")
    deviceFormat = "LPhalanx%d";
  else if (name() == "RHand")
    deviceFormat = "RPhalanx%d";
  else
    cerr << "Cannot create Hand for unknown Sim::CoupledSensor: " << name() << "\n";

  // get phalanx motors: the real Nao has 1 motor for each hand
  // but in Webots we have 8 phalanx motors for each hand
  for (int i = 0; i < PHALANX_MAX; i++) {
    char name[32];
    sprintf(name, deviceFormat, i + 1);
    mTags[i] = wb_robot_get_device(name);
  }
  
  mSensor = Singletons::model()->coupledSensor(name());
  if (! mSensor)
    cerr << "Sim::CoupledSensor not found for CoupledActuator: " << name() << "\n";
}

Hand::~Hand() {
}

void Hand::update() {

  // set target position
  double target = Singletons::hal()->fetchCoupledActuatorValue(mActuator);
  if (isnan(target)) target = mActuator->startValue();
  
  // we must activate the 8 phalanx motors
  for (int i = 0; i < PHALANX_MAX; i++)
    if (mTags[i])
      wb_motor_set_position(mTags[i], target);
  
  // effective position feedback
  if (mSensor) {
    if (! Singletons::hal()->sendCoupledSensorValue(mSensor, (float)target))
      cerr << "Sim::HALInterface::sendCoupledSensorValue() failed.\n";
  }
}
