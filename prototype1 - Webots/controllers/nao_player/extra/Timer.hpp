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


#ifndef TIMER_HPP
#define TIMER_HPP

// File:        Timer.hpp
// Description: Class helping to measure delta time in milliseconds
// Author:      Fabien Rohrer - www.cyberbotics.com

class Timer {
public:
  // constructor & destructor
  Timer();
  virtual ~Timer() {}
  
  double delta(); // second
  void reset();

private:
  double time() const; // second

  double beforeTime;
};

#endif
