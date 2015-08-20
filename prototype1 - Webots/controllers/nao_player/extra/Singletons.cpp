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

#include "Singletons.hpp"
#include "Process.hpp"
#include <stdlib.h>
#include <iostream>
#include <sstream>

using namespace Sim;
using namespace std;

#ifdef WIN32
#define SEP "\\"
#define putenv _putenv
#else
#define SEP "/"
#endif

Model *Singletons::mModel = NULL;
HALInterface *Singletons::mHal = NULL;
Process *Singletons::mHalProcess = NULL;
Process *Singletons::mNaoQiProcess = NULL;

bool Singletons::initialize(const string &model, int naoqiPort) {

  const string WEBOTS_NAOSIM_DIR(getenv("WEBOTS_NAOSIM_DIR"));
  const string pathToModel = WEBOTS_NAOSIM_DIR + SEP + "share" + SEP + "alrobotmodel" + SEP + "models" + SEP + model + ".xml";

  string WEBOTS_NAOQI_DIR;
  const char* var = getenv("WEBOTS_NAOQI_DIR");
  if (var != NULL)
    WEBOTS_NAOQI_DIR = var;
  else {
    size_t pos = WEBOTS_NAOSIM_DIR.find_last_of(SEP);
    WEBOTS_NAOQI_DIR = WEBOTS_NAOSIM_DIR.substr(0, pos) + SEP + "naoqi-runtime";
    const string WEBOTS_NAOQI_DIR_ENV = "WEBOTS_NAOQI_DIR=" + WEBOTS_NAOQI_DIR;
    putenv((char*)WEBOTS_NAOQI_DIR_ENV.c_str());
  }

  stringstream out;
  out << naoqiPort;
  const string portStr = out.str();

  try {
    mModel = new Model(pathToModel);
  }
  catch (const exception &e) {
    cerr << "exception raised: " << e.what() << "\n";
    cerr << "while calling Model::Model(" << pathToModel << ")\n";
    return false;
  }

  try {
    mHal = new HALInterface(mModel, naoqiPort);
  }
  catch (const exception &e) {
    cerr << "exception raised: " << e.what() << "\n";
    cerr << "while calling HALInterface::HALInterface(" << mModel << ", " << naoqiPort << ")\n";
    return false;
  }

  cout << "===== starting hal =====\n" << flush;

  // we need to convert model ("NAO_H25_V40", "NAO_T14_V32" or "ROMEO") to robotType ("Nao" or "Romeo") in a generic way
  string robotType;
  robotType += model[0];
  int l=model.length();
  for(int i=1;i<l;i++)
   if (isalpha(model[i])) robotType+=tolower(model[i]); else break;
  
  //string commandLine = WEBOTS_NAOQI_DIR + SEP + "bin" + SEP + "nao_simulation_hal " + portStr; // naoqi-runtime version 1.12
  string commandLine = WEBOTS_NAOQI_DIR + SEP + "bin" + SEP + "hal";
  string options = "-s hal-ipc" + portStr +
                   " -p HAL/Robot/Type:string=" + robotType +
                   " -p HAL/Simulation:int=1 -p HAL/SimShmId:int=" + portStr; // naoqi-runtime version 1.14
  cout << commandLine << " " << options << "\n";
  mHalProcess = new Process(commandLine,options);
  
  // sleep 2s
  // 1 second is too low in some environments: http://www.cyberbotics.com/forum?message=2644
#ifdef WIN32
  Sleep(2000);
#else
  for (int k = 0; k < 2; ++k)
    usleep(1000000); // Note: usleep argument should be 1000000 at maximum
#endif

  cout << "===== starting naoqi-bin =====\n" << flush;

  commandLine = WEBOTS_NAOQI_DIR + SEP + "bin" + SEP + "naoqi-bin";
  options = "-p " + portStr;
  cout << commandLine << " " << options << "\n";
  mNaoQiProcess = new Process(commandLine,options);
  
  // sleep 1s
#ifdef WIN32
  Sleep(1000);
#else
  usleep(1000000);
#endif

  return true;
}

void Singletons::shutdown() {
  cout << "===== shutting down ! =====\n" << flush;

  delete mNaoQiProcess;
  delete mHalProcess;
  delete mHal;
  delete mModel;
}
