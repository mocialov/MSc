
#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <gui/MainWindow.hpp>

class QGraphicsItem;

#define NROBOTS 5

typedef struct
{
    int TAG;
    float x;
    float y;
    float z;
} Point;

class Tracker : public webotsQtUtils::MainWindow {
public:
  Tracker();
  virtual ~Tracker();
 
  virtual QSize sizeHint() const;

  void readSensors();
  void writeActuators();

private:
  QGraphicsItem *mRobotRepresentation[NROBOTS];
  
public slots:
  void updateGraphics();
};

extern "C" {
  void wbw_robot_window_custom_function(Point points[5][5][667]);
}

#endif
