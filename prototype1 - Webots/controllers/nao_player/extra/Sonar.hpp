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

#ifndef SONAR_HPP
#define SONAR_HPP

// File:        Sonar.hpp
// Description: Class for handling a single ultra sound sonar of the Nao robot
// Author:      Yvan Bourquin - www.cyberbotics.com

#include "Device.hpp"
#include <webots/types.h>

namespace Sim {
  class SonarSensor;
}

class Sonar : public Device {
public:
  // constructor & destructor
  Sonar(const Sim::SonarSensor *sensor, int step);
  virtual ~Sonar();
  
  // reimplemented functions
  virtual void update();
  
private:
  std::string mName;
  WbDeviceTag mTag;
  const Sim::SonarSensor *mSensor;
};

#endif
