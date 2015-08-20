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

#ifndef HAND_HPP
#define HAND_HPP

// File:        Hand.hpp
// Description: Class for controlling a Nao hand
// Author:      Yvan Bourquin - www.cyberbotics.com

#include "Device.hpp"
#include <webots/types.h>

namespace Sim {
  class CoupledActuator;
  class CoupledSensor;
}

class Hand : public Device {
public:
  // constructor & destructor
  Hand(const Sim::CoupledActuator *actuator, int step);
  virtual ~Hand();
  
  // reimplemented functions
  virtual void update();
  
private:
  enum { PHALANX_MAX = 8 };
  WbDeviceTag mTags[PHALANX_MAX];
  const Sim::CoupledActuator *mActuator;
  const Sim::CoupledSensor *mSensor;
};

#endif
