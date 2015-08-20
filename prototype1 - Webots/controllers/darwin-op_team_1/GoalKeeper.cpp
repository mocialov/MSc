#include "GoalKeeper.hpp"
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>
#include <iostream>
#include <cmath>
#include <cassert>

using namespace webots;

static double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}

GoalKeeper::GoalKeeper(int playerID, int teamID) : Player(playerID, teamID) {
}

GoalKeeper::~GoalKeeper() {
}

bool GoalKeeper::getBallCenter(double &x, double &y) {
  static int width  = camera->getWidth();
  static int height = camera->getHeight();
  
  const unsigned char *im = camera->getImage();
  int x0 = -1;
  int y0 = -1;
  for (int j=0; j<height; j++) {
    for (int i=0; i<width; i++) {
      int r = Camera::imageGetRed  (im, width, i, j);
      int g = Camera::imageGetGreen(im, width, i, j);
      int b = Camera::imageGetBlue (im, width, i, j);
      
      cout << r << " | " << g << " | " << b << endl;
      
      if (r>200 && g>110 && g<205 && b<40) {
        x0 = i;
        y0 = j;
      }
    }
  }
  
  if (x0==-1 && y0==-1) {
    x = 0.0;
    y = 0.0;
    return false;
  } else {
    x = 2.0 * x0 / width  - 1.0;
    y = 2.0 * y0 / height - 1.0;
    return true;
  }
}

void GoalKeeper::run() {
  step(SIMULATION_STEP);
  
  gaitManager->start();
  

      //motionManager->playPage(57); // hello

    motionManager->playPage(1); // init position

motionManager->playPage(9);
  while (true) {
    double x, y;
    //bool ballInFieldOfView = getBallCenter(x, y);

      double neckPosition = clamp(-x, motorMinPositions[18], motorMaxPositions[18]);

      double headPosition = clamp(-y, motorMinPositions[19], motorMaxPositions[19]);

      gaitManager->setXAmplitude(0.5);
      gaitManager->setAAmplitude(0.5);
      gaitManager->step(SIMULATION_STEP);
      

      
  servos[18]->setPosition(neckPosition);
      servos[19]->setPosition(headPosition);
      
      motionManager->playPage(9);
      
    sleepSteps(5);
    runStep();
  }
}
