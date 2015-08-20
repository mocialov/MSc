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

#include "Camera.hpp"
#include "Singletons.hpp"
#include "Image.hpp"
#include <webots/robot.h>
#include <webots/camera.h>
#include <iostream>

using namespace Sim;
using namespace std;


Camera::Camera(const CameraSensor *sensor, int timeStep) : Device(sensor->name()) {
  
  mSensor = sensor;

  cout << "simulator-sdk max camera resolution: " << name() << ": " << sensor->width() << "x" << sensor->height() << "\n";
  
  mTag = wb_robot_get_device(name().c_str());
  if (! mTag) {
    cerr << "Webots Camera not found for Sim::CameraSensor: " << name() << "\n";
    return;
  }

  // enable Webots camera
  wb_camera_enable(mTag, 4 * timeStep); // refreshing the camera images at each steps is costly
  
  // width and height of Webots camera images
  mWidth = wb_camera_get_width(mTag);
  mHeight = wb_camera_get_height(mTag);
  cout << "Webots camera resolution: " << name() << ": " << mWidth << "x" << mHeight << "\n";
}

Camera::~Camera() {
}

void Camera::update() {
  
  if (! mTag)
    return;

  // find out the width and height expected by the HAL
  int bufferSize = 0;
  int width = 0;
  int height = 0;
  Singletons::hal()->cameraBufferSize(mSensor, &bufferSize, &width, &height);
  if (bufferSize == 0 || width == 0 || height == 0)
    return;
  
  const unsigned char *imageData = wb_camera_get_image(mTag);

  // create image with Webots camera dimensions
  Image image(mWidth, mHeight, imageData);

  // rescale Webots image to HAL image dimensions change format to RGB
  image.convert(width, height);

  // update HAL
  if (! Singletons::hal()->sendCameraSensorValue(mSensor, image.data()))
    cerr << "Sim::HALInterface::sendCameraSensorValue() failed.\n";
}
