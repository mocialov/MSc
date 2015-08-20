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
INCLUDE += -I/usr/local/Cellar/opencv/2.4.10.1/include


LIBRARIES = -L"$(QT_UTILS)" -lqt_utils
LIBRARIES += -L/usr/local/Cellar/opencv/2.4.10.1/lib -lopencv_objdetect -lopencv_core -lopencv_highgui -lopencv_video -lopencv_imgproc -L/usr/lib -lc++ -lc++abi -lm -lc -I/usr/include -lopencv_photo

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include