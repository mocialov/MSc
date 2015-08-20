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

#ifndef LOW_PASS_FILTER_HPP
#define LOW_PASS_FILTER_HPP

// File:        LowPassFilter.hpp
// Description: Simulate an electronical low pass filter
// Source:      http://en.wikipedia.org/wiki/Lowpass_filter
// Author:      Fabien Rohrer - www.cyberbotics.com

class LowPassFilter {
public:
  LowPassFilter(double timeInterval, double RCTimeConstant);

  void appendRawValue(double value);
  double filteredValue() const { return mCurrentFilteredValue; }
  
private:
  double mCurrentFilteredValue;
  double mCurrentRawValue;
  double mPreviousFilteredValue;

  double mAlpha;
  double mOneMinusAlpha;
  int mSize;
};

#endif
