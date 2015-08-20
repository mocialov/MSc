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

#ifndef SINGLETONS_HPP
#define SINGLETONS_HPP

// File:        Singletons.hpp
// Description: static environment for naoqisim
//              the Model contains a description of the Nao model
//              the HALInterface is used to communicate with the sensors and actuators
// Author:      Yvan Bourquin - www.cyberbotics.com

#include <alnaosim/alnaosim.h>
#include <alrobotmodel/alrobotmodel.h>

#include <string>

class Process;

class Singletons {
public:
  // initialize and shutdown simulation environment
  static bool initialize(const std::string &model, int naoqiPort);
  static void shutdown();

  // robot model
  static Sim::Model *model() { return mModel; }
  
  // hardware abstraction layer
  static Sim::HALInterface *hal() { return mHal; }

private:
  static Sim::Model *mModel;
  static Sim::HALInterface *mHal;
  static Process *mHalProcess;
  static Process *mNaoQiProcess;
};

#endif
