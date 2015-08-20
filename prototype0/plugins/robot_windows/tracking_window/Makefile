# Webots Makefile system 
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include


CXX_SOURCES = $(wildcard *.cpp)

QT = core gui widgets
USE_C_API = true

QT_UTILS = $(WEBOTS_HOME)/resources/projects/default/libraries/qt_utils
#QT_UTILS = $(WEBOTS_HOME)/resources/projects/robots/nao/aldebaran/simulator-sdk/include
INCLUDE = -I"$(QT_UTILS)"
LIBRARIES = -L"$(QT_UTILS)" -lqt_utils

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
