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

#ifndef HIP_MOTOR_HPP
#define HIP_MOTOR_HPP

// File:        HipMotor.hpp
// Description: Class for handling the special case of the hip HipYawPitch motors of the Nao robot
//              The LHipYawPitch and RHipYawPitch are in fact a single motor in the real Nao, but
//              they are represented as two distinct Motors (motors) in the Webots models
// Author:      Yvan Bourquin - www.cyberbotics.com

#include "Device.hpp"
#include <webots/types.h>

namespace Sim {
  class AngleActuator;
  class AngleSensor;
}

class HipMotor : public Device {
public:
  // constructor & destructor
  HipMotor(int step);
  virtual ~HipMotor();
  
  // reimplemented functions
  virtual void update();
  
private:
  WbDeviceTag mLeftTag, mRightTag;
  const Sim::AngleActuator *mActuator;
  const Sim::AngleSensor *mSensor;
  
  double mMaxLeftPosition;
  double mMinLeftPosition;
  double mMaxRightPosition;
  double mMinRightPosition;
};

#endif
