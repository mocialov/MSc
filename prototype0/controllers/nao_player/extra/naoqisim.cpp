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

// File:        naoqisim.cpp
// Description: Interface between Webots and Nao simulation SDK 
// Author:      Yvan Bourquin - www.cyberbotics.com

#include <webots/robot.h>
#include <webots/emitter.h>
#include <iostream>
#include <stdlib.h>
#include <cstring>
#include "Nao.hpp"
#include "Singletons.hpp"

#ifdef WIN32
#include <windows.h>
#endif

using namespace std;

static void printUsage() {
  cerr << "Please specify the NAOQI_PORT_NUMBER in the 'controllerArgs' field of the Nao robot.\n";
  cerr << "Usage: controllerArgs \"[-nocam] NAOQI_PORT_NUMBER\"\n";
  cerr << "Note that each Nao robot should use a different port number.\n";
  cerr << "Options: -nocam, disable the simulated camera\n";
}

#ifdef WIN32
// naoqisim is linked with the Windows SubSystem (/SUBSYSTEM:WINDOWS) in order to avoid the console pop-up
// hence WinMain() is used instead of main() on Windows
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd) {
  int argc = __argc;
  char **argv = __argv;
#else
int main(int argc, char *argv[]) {
#endif

  if (argc < 2 || argc > 3) {
    printUsage();
    return EXIT_FAILURE;
  }
  
  int naoqiPort = atoi(argv[argc - 1]);
  if (naoqiPort == 0) {
    cerr << "Error: invalid NAOQI_PORT_NUMBER specified in 'controllerArgs'\n";
    printUsage();
    return EXIT_FAILURE;
  }

  bool useCameras = true;
  if (argc == 3) {
    if (strcmp(argv[1], "-nocam") == 0)
      useCameras = false;
    else {
      cerr << "invalid argument: " << argv[1] << "\n";
      printUsage();
      return EXIT_FAILURE;
    }
  }
  
  // initialize webots
  wb_robot_init();
  
  // emitter to signal when naoQi has started
  WbDeviceTag emit = wb_robot_get_device("emitter");
  
  // get WorldInfo.basicTimeStep
  int timeStep = (int)wb_robot_get_basic_time_step();
  
  cout << "===== naoqisim controller started =====\n" << flush;
  cout << "Press the real-time button [>] to start the simulation, then you can connect Choregraphe.\n" << flush;
  
  // wait for simulation to start or revert
  if (wb_robot_step(timeStep) == -1) {
    wb_robot_cleanup();
    return EXIT_SUCCESS;
  }
  
  // the robot model is hidden in the Robot.name field in the .proto file
  string robotModel(wb_robot_get_name());

  // initialize HAL
  if (! Singletons::initialize(robotModel, naoqiPort))
    return EXIT_FAILURE;

  // create/run/destroy Nao
  Nao *nao = new Nao(timeStep, useCameras);
  
  // signals that naoqi is ready
  wb_emitter_send(emit, "ready", strlen("ready") + 1); 
  nao->run();
  
  delete nao;

  // cleanup
  Singletons::shutdown();
  wb_robot_cleanup();
  
  return EXIT_SUCCESS;
}
