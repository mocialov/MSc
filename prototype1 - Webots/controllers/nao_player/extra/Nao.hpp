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

#ifndef NAO_HPP
#define NAO_HPP

// File:        Nao.hpp
// Description: Class that represents a single Nao robot and its devices
// Author:      Yvan Bourquin - www.cyberbotics.com

class Device;

#include <vector>

class Nao {
public:
  // create nao robot
  Nao(int timeStep, bool useCamera);
  virtual ~Nao();  
  
  // run synchronization between simulated robot and HAL
  // this function returns when Webots simulation is terminated by the user
  void run();

private:
  int mTimeStep;
  std::vector<Device*> mDevices;
  
  void update();
  void checkRealTime();
};

#endif
