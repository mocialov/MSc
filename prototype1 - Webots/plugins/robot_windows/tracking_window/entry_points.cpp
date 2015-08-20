#include "entry_points.hpp"
#include "Tracker.hpp"

#include <core/MainApplication.hpp>

using namespace webotsQtUtils;

static MainApplication *gApplication = NULL;
static Tracker *gTracker = NULL;

bool wbw_init() {
  gApplication = new MainApplication;
  if (gApplication->isInitialized())
    gTracker = new Tracker;
  return gApplication->isInitialized();
}

void wbw_cleanup() {
  if (gTracker) {
    delete gTracker;
    gTracker = NULL;
  }
  if (gApplication) {
    delete gApplication;
    gApplication = NULL;
  }
}

void wbw_pre_update_gui() {
  if (gApplication && gApplication->isInitialized())
    gApplication->preUpdateGui();
}

void wbw_update_gui() {
  if (gApplication && gApplication->isInitialized())
    gApplication->updateGui();
}

void wbw_read_sensors() {
  if (gTracker && gTracker->isVisible())
    gTracker->readSensors();
}

void wbw_write_actuators() {
  if (gTracker && gTracker->isVisible())
   gTracker->writeActuators();
}

void wbw_show() {
  if (gTracker)
    gTracker->showWindow();
}
