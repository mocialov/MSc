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

#ifndef MOTOR_HPP
#define MOTOR_HPP

// File:        Motor.hpp
// Description: Class for controlling each motor of the Nao robot (with th exception of the hip motor)
// Author:      Yvan Bourquin - www.cyberbotics.com

#include "Device.hpp"
#include <webots/types.h>

namespace Sim {
  class AngleActuator;
  class AngleSensor;
}

class Motor : public Device {
public:
  // constructor & destructor
  Motor(const Sim::AngleActuator *actuator, int step);
  virtual ~Motor();
  
  // reimplemented functions
  virtual void update();
  
private:
  std::string mName;
  WbDeviceTag mTag;
  const Sim::AngleActuator *mActuator;
  const Sim::AngleSensor *mSensor;
  
  double mMaxPosition;
  double mMinPosition;
};

#endif
