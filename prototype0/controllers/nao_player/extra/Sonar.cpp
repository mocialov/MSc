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

#include "Sonar.hpp"
#include "Singletons.hpp"
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <iostream>

using namespace Sim;
using namespace std;

Sonar::Sonar(const SonarSensor *sensor, int step) : Device(sensor->name()) {
  
  mSensor = sensor;

  mTag = wb_robot_get_device(name().c_str());
  if (! mTag)
    cerr << "Webots DistanceSensor not found for Sim::SonarSensor: " << name() << "\n";
  else
    wb_distance_sensor_enable(mTag, step);
}

Sonar::~Sonar() {
}

void Sonar::update() { 
    
  if (! mTag) return;
  
  if (mSensor) {
    float value = (float)wb_distance_sensor_get_value(mTag);
    
    // send 10 echoe values, only the 1st one is used by the HAL
    if (! Singletons::hal()->sendSonarSensorValues(mSensor, vector<float>(10, value)))
      cerr << "Sim::HALInterface::sendSonarSensorValues() failed.\n";
  }
}
