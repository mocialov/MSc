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

#ifndef BUMPER_HPP
#define BUMPER_HPP

// File:        Bumper.hpp
// Description: Foot bumper
// Author:      Yvan Bourquin - www.cyberbotics.com

#include "Device.hpp"
#include <webots/types.h>

namespace Sim {
  class BumperSensor;
}

class Bumper : public Device {
public:
  // constructor and destructor
  Bumper(const Sim::BumperSensor *sensor, int step);
  virtual ~Bumper();
  
  // reimplemented functions
  virtual void update();
  
private:
  WbDeviceTag mTag;
  const Sim::BumperSensor *mSensor;
};

#endif
