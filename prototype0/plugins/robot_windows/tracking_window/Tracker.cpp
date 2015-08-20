#include "Tracker.hpp"

#include <QtWidgets/QGraphicsItem>
#include <QtWidgets/QGraphicsEllipseItem>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include <QtCore/QTimer>
#include <core/MainApplication.hpp>

#include <webots/supervisor.h>

#include <iostream>

using namespace webotsQtUtils;

static const int DIMENSION = 800;
static const int MARGIN = 30;
static const int X = 0;
static const int Z = 2;

int temp=0;

QGraphicsScene *scene;
QGraphicsView *view;
QWidget* viewport;

Tracker::Tracker() : MainWindow() {
  scene = new QGraphicsScene(this);

  view = new QGraphicsView(scene);
  viewport = view->viewport();
  
  setCentralWidget(view);
}

void Tracker::updateGraphics(){
printf("update\n");
QList<QGraphicsItem*> all = scene->items();
for (int i = 0; i < all.size(); i++)
{
    QGraphicsItem *gi = all[i];
    if(gi->parentItem()==NULL) {
        delete gi;
    }
}

scene->clear();
viewport->update();
}

Tracker::~Tracker() {
}

QSize Tracker::sizeHint() const {
  static QSize size(DIMENSION, DIMENSION);
  return size;
}

void Tracker::readSensors() {
  //for (int i = 0; i < NROBOTS; i++) {
  //  QString robotName = QString("ROBOT%1").arg(i+1);
  //  WbNodeRef r = wb_supervisor_node_get_from_def(robotName.toLatin1().data());
  //  WbFieldRef f = wb_supervisor_node_get_field(r, "translation");
  //  const double *position = wb_supervisor_field_get_sf_vec3f(f);
  //  mRobotRepresentation[i]->setPos(
  //    position[X] * (DIMENSION - 2*MARGIN),
  //    position[Z] * (DIMENSION - 2*MARGIN)
  //  );
  //}
}

void Tracker::writeActuators() {
}


//shows additional window with all the clustered points real-time
void wbw_robot_window_custom_function(Point points[5][5][667]){
    QPen pen(Qt::blue, 1);
    int color_index = -1;

    int i,m,n;
    float previousX, previousY = -1;
    
    scene->clear();
    viewport->update();        
    qApp->processEvents();     
    
    for(i=0; i<5; i++){
      for(m=0; m<5; m++){
        previousX = -1;
        previousY = -1;
        QBrush *brush = new QBrush();
        brush->setStyle(Qt::SolidPattern);
        brush->setColor(QColor(rand() % 255 + 1,rand() % 255 + 1,rand() % 255 + 1));
        
        for(n=0;n<667;n++){
        if((((Point)points[i][m][n]).y == ((Point)points[i][m][n]).y) && (((Point)points[i][m][n]).x == ((Point)points[i][m][n]).x))
          if((((Point)points[i][m][n]).y < (float)5.6) && (((Point)points[i][m][n]).y > (float)0.0)){

            double rad = 3;
            scene->addEllipse(((Point)points[i][m][n]).x*10.0-rad, ((Point)points[i][m][n]).y*10.0-rad, rad*2.0, rad*2.0, QPen(), *brush);
            
            if((previousX != -1) && (previousY != -1))
              scene->addLine(previousX*10.0, previousY*10.0, ((Point)points[i][m][n]).x*10.0, ((Point)points[i][m][n]).y*10.0, pen);
            
            previousX = ((Point)points[i][m][n]).x;
            previousY = ((Point)points[i][m][n]).y;
          }
        }
      }
    }
    
    char fn [100];
    
    //print to file
    snprintf (fn, sizeof fn, "file_no_%02d.png", temp);

    QImage img(scene->sceneRect().width(), scene->sceneRect().height(), QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::transparent);
 
    QPainter painter(&img);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    scene->render(&painter);
    painter.end();
 
    img.save(fn, "PNG");
   

    temp++;

    memset(&points, 0, 5*5*667*sizeof(Point));
}