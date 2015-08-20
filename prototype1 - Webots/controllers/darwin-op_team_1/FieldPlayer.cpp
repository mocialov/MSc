#include "FieldPlayer.hpp"
#include "../nao_soccer_supervisor/RoboCupGameControlData.h"
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>
#include <cmath>

using namespace webots;

FieldPlayer::FieldPlayer(int playerID, int teamID) : Player(playerID, teamID) {
}

FieldPlayer::~FieldPlayer() {
}

// simple and not robust ball detection algorithm
// finding the last pixel which seems like the ball color
// - return: indicates if the algorithm found the ball
// - args: return the position of the ball [-1.0, 1.0]
bool FieldPlayer::getBallCenter(double &x, double &y) {
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

void FieldPlayer::run() {
  step(SIMULATION_STEP);
  
  // play the motion preparing the robot to walk
  gaitManager->start();
  
  // main loop
  double px = 0.0;
  double py = 0.0;
  int fup = 0;
  int fdown = 0;
  const double acc_tolerance = 80.0;
  const double acc_step = 100;
  
  while (true) {
    if (gameControlData->state != STATE_PLAYING) {
      motionManager->playPage(57); // hello
      motionManager->playPage(1); // init position
      sleepSteps(5);
      continue;
    }
    
    double x, y;
    bool ballInFieldOfView = getBallCenter(x, y);
    const double *acc = accelerometer->getValues();
    
    // count how many steps the accelerometer
    // says that the robot is down
    if (acc[1] < 512.0 - acc_tolerance)
      fup++;
    else
      fup = 0;
    
    if (acc[1] > 512.0 + acc_tolerance)
      fdown++;
    else
      fdown = 0;
    
    // the robot face is down
    if (fup > acc_step) {
      motionManager->playPage(1); // init position
      motionManager->playPage(10); // f_up
      motionManager->playPage(1); // init position    
      fup = 0;
    }
    // the back face is down
    else if (fdown > acc_step) {
      motionManager->playPage(1); // init position
      motionManager->playPage(11); // b_up
      motionManager->playPage(1); // init position
      fdown = 0;
    }
    // if the ball is in the field of view,
    // go in the direction of the ball and kick it
    else if (ballInFieldOfView) {
      // compute the direction of the head
      // the head move at maximum by 0.01 [rad] at each time step
      x  = 0.01*x + px;
      y  = 0.01*y + py;
      px = x;
      py = y;

      // go forwards and turn according to the head rotation
      gaitManager->setXAmplitude(1.0);
      gaitManager->setAAmplitude(-x);
      gaitManager->step(SIMULATION_STEP);
      servos[18]->setPosition(-x);
      servos[19]->setPosition(-y);
      
      // if the ball is close enough
      // kick the ball with the right foot
      if (y > 1.25) {
        if (x<0.0)
          motionManager->playPage(13); // left kick
        else
          motionManager->playPage(12); // right kick
        sleepSteps(5);
        px = 0.0;
        py = 0.0;
      }
    
    // the ball is not in the field of view,
    // search it by turning round and moving vertically the head 
    } else {
      // turn round
      gaitManager->setXAmplitude(0.0);
      gaitManager->setAAmplitude(1.0);
      gaitManager->step(SIMULATION_STEP);
      
      // move the head vertically
      servos[19]->setPosition(-0.7*sin(2.0*getTime()) + 0.7);
    }
    
    // step
    runStep();
  }
}
