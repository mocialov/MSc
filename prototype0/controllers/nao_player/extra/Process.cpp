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

#include "Process.hpp"
#include <iostream>
#include <stdlib.h>

using namespace std;

#ifdef WIN32

Process::Process(const string &commandLine,const string &options) {
  STARTUPINFO si;
  GetStartupInfo(&si);

  // hide console on startup
  si.cb = sizeof(STARTUPINFO);
  si.dwFlags = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
  si.wShowWindow = SW_HIDE;
  si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
  si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
  string fullCommand = commandLine + " " + options;
  int status = CreateProcess(NULL, (LPSTR)fullCommand.c_str(), NULL, NULL, TRUE, 0, NULL, NULL, &si, &mPi);
  if (status == 0) {
    cerr << "CreateProcess() failed for: " << commandLine << " (error " << GetLastError() << ")\n";
    exit(EXIT_FAILURE);
  }
}

Process::~Process() {
  // this will only work if the primary thread of the process has created a message queue
  PostThreadMessage(mPi.dwThreadId, WM_CLOSE, 0, 0);
  WaitForSingleObject(mPi.hProcess, 500);  // allow 500 milliseconds to terminate

  // check exit code
  DWORD dwExitCode = 0;
  GetExitCodeProcess(mPi.hProcess, &dwExitCode);
  if (dwExitCode == STILL_ACTIVE) {
    // process did not terminate -> force it
    TerminateProcess(mPi.hProcess, 0);  // zero is the exit code                                               
  }

  CloseHandle(mPi.hThread);
  CloseHandle(mPi.hProcess);
}

#else

#include <signal.h>
#include <sstream>
#include <sys/wait.h>

Process::Process(const string &commandLine,const string &options) {
  mPid = ::fork();
  if (mPid < 0)  // error
    cerr << "fork() failed for: " << commandLine << "\n";
  else if (mPid == 0) {  // child ...
    // convert std::string command line to argv
    char *argv[32];
    argv[0] = new char[commandLine.size() + 1];
    copy(commandLine.begin(), commandLine.end(), argv[0]);
    argv[0][commandLine.size()]='\0';
    int argc = 1;
    istringstream iss(options);
    string token;
    while (iss >> token) {
      char *arg = new char[token.size() + 1];
      copy(token.begin(), token.end(), arg);
      arg[token.size()] = '\0';
      argv[argc++] = arg;
    }
    
    // argv must be NULL terminated for exec
    argv[argc++] = NULL;
    
    // execute program (should never return)
    // no need to cleanup argv, because the process memory is replaced
    ::execv(argv[0], argv);

    // we should not reach this point
    cerr << "execv() failed for: " << argv[0] << " " << options << "\n";
    exit(EXIT_FAILURE);
  }
}

Process::~Process() {
  ::kill(mPid, SIGTERM);
  // allow 400 milliseconds to terminate
  for (int i=0; i < 4; i++) {
    usleep(100000);
    if (::waitpid(mPid, NULL, WNOHANG) != 0)
      return;
  }
  ::kill(mPid, SIGKILL);
}

#endif

