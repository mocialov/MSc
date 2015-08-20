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

#ifndef INERTIAL_UNIT_HPP
#define INERTIAL_UNIT_HPP

// File:        InertialUnit.hpp
// Description: Simulate the Nao's inertial unit using Webots Accelerometer, Gyro, and InertialUnit
// Author:      Yvan Bourquin - www.cyberbotics.com
// Mofidied:
//   - 25 Apr 2013: Fabien Rohrer
//                  - Added a low pass filter smoothing signal

#include "Device.hpp"
#include <webots/types.h>

namespace Sim {
  class InertialSensor;
}

class LowPassFilter;

class InertialUnit : public Device {
public:
  // constructor & destructor
  InertialUnit(const Sim::InertialSensor *sensor, int step);
  virtual ~InertialUnit();

  // reimplemented functions
  virtual void update();

private:
  enum {
    ANGLE_X = 0, ANGLE_Y,
    ACC_X, ACC_Y, ACC_Z,
    GYR_X, GYR_Y
  };

  WbDeviceTag mAccelerometer, mGyroscope, mInertialUnit;
  const Sim::InertialSensor *mSensor;
  LowPassFilter *mFilters[7];
};

#endif
