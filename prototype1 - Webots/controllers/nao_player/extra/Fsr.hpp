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

#ifndef FSR_HPP
#define FSR_HPP

// File:        Fsr.hpp
// Description: Simulated Force Sensitive Resistor (FSR) of the Nao robot 
// Author:      Yvan Bourquin - www.cyberbotics.com
// Mofidied:
//   - 25 Apr 2013: Fabien Rohrer
//                  - Added a low pass filter smoothing signal

#include "Device.hpp"
#include <webots/types.h>
#include <vector>

namespace Sim {
  class FSRSensor;
}

class LowPassFilter;

class Fsr : public Device {
public:
  // constructor and destructor
  Fsr(std::vector<const Sim::FSRSensor*> sensors, int step, std::string name);
  virtual ~Fsr();
  
  // reimplemented functions
  virtual void update();
  
private:
  void inferValues();
  WbDeviceTag mTag;
  std::vector<const Sim::FSRSensor*> mFSRSensors;
  LowPassFilter *mFilter[4];
};

#endif
