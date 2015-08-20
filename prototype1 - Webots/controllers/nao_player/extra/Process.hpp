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

#ifndef PROCESS_HPP
#define PROCESS_HPP

// File:        Process.hpp
// Description: Class used for starting external processes
// Author:      Yvan Bourquin - www.cyberbotics.com

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <string>

class Process {
public:
  // create and start process
  Process(const std::string &commandLine, const std::string &options);
  
  // kill process
  ~Process();
    
private:
#ifdef WIN32
  PROCESS_INFORMATION mPi;
#else
  pid_t mPid;
#endif
};

#endif
