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

#ifndef DEVICE_HPP
#define DEVICE_HPP

// File:        Device.hpp
// Description: Abstract base class for simulated Nao devices
// Author:      Yvan Bourquin - www.cyberbotics.com

#include <string>

class Device {
public:
  // constructor
  Device(const std::string &name) { mName = name; }
  
  // destructor
  virtual ~Device() {}
  
  // device name as in HAL
  const std::string &name() const { return mName; }
  
  // update the device: Webots -> HAL and HAL -> Webots
  virtual void update() = 0;
  
private:
  std::string mName;
};

#endif
