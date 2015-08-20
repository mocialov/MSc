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

#ifndef IMAGE_HPP
#define IMAGE_HPP

// File:        Image.hpp
// Description: Class for storing and resampling pixmap images
// Author:      Yvan Bourquin - www.cyberbotics.com

class Image {
public:
  // create/destroy image
  Image(int width, int height, const unsigned char *data); // data is a pointer to a BGRA image buffer 
  virtual ~Image();
  
  // RGB image data in bytes as an array of [width][height][3]
  unsigned char *data() const { return mData; }
  
  // // change image format from BGRA to RGB and dimensions to the ones specified
  void convert(int newWidth, int newHeight);
  
private:
  int mWidth, mHeight;
  unsigned char *mData;
  bool mFreeData;
};

#endif
