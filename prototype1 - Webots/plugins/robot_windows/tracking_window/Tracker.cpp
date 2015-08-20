#include "Tracker.hpp"

#include <QtWidgets/QGraphicsItem>
#include <QtWidgets/QGraphicsEllipseItem>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include <QtCore/QTimer>
#include <core/MainApplication.hpp>

#include <webots/supervisor.h>

#include <iostream>

#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<algorithm>

using namespace webotsQtUtils;
using namespace std;
using namespace cv;

static const int DIMENSION = 800;
static const int MARGIN = 30;
static const int X = 0;
static const int Z = 2;

QGraphicsScene *scene;
QGraphicsView *view;
QWidget* viewport;


CascadeClassifier face_cascade;
CascadeClassifier upper_body_cascade;
	    //setup image files used in the capture process
	    Mat grayscaleFrame;
	    
////

  	Mat back;
	Mat fore;
	vector<pair<cv::Point,double> > palm_centers;

	BackgroundSubtractorMOG2 bg;


	int backgroundFrame=500;
	
typedef struct Limbs
{
   cv::Point  start;
   cv::Point  end;
   RotatedRect limb_bounding_rectangle;
   cv::Point break_point;
} Limb;
	
	
	void initFaces(){
	    //use the haarcascade_frontalface_alt.xml library
	    face_cascade.load("/usr/local/Cellar/opencv/2.4.10.1/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml");
		upper_body_cascade.load("/usr/local/Cellar/opencv/2.4.10.1/share/OpenCV/haarcascades/haarcascade_mcs_upperbody.xml");
}


Tracker::Tracker() : MainWindow() {
  scene = new QGraphicsScene(this);

  // outline


  //// robot representation
  //for (int i = 0; i < NROBOTS; i++)
  //  mRobotRepresentation[i] = static_cast<QGraphicsItem *> (scene->addEllipse(0, 0, 15, 15));

  view = new QGraphicsView(scene);
  viewport = view->viewport();
  
  setCentralWidget(view);
  
  //QTimer *timer = new QTimer(this);
  
  //timer->setInterval(1000);
  //timer->start();
    
  //connect(timer, SIGNAL(timeout()), scene, SLOT(clear())); 
  //connect(timer, SIGNAL(timeout()), viewport, SLOT(update())); 
  
	bg.set("nmixtures",3);
	bg.set("detectShadows",false);





	initFaces();
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

int printf_ByteArray(const unsigned char *data, size_t len) {
  size_t i;
  int result = 0;
  for (i = 0; i < len; i++) {
    int y;
    int ch = data[i];
    static const char escapec[] = "\a\b\t\n\v\f\n\'\"\?\\";
    char *p = strchr(escapec, ch);
    if (p && ch) {
      static const char escapev[] = "abtnvfn\'\"\?\\";
      y = printf("\\%c", escapev[p - escapec]);
    } else if (isprint(ch)) {
      y = printf("%c", ch);
    } else {
      // If at end of array, assume _next_ potential character is a '0'.
      int nch = i >= (len - 1) ? '0' : data[i + 1];
      if (ch < 8 && (nch < '0' || nch > '7')) {
        y = printf("\\%o", ch);
      } else if (!isxdigit(nch)) {
        y = printf("\\x%X", ch);
      } else {
        y = printf("\\o%03o", ch);
      }
    }
    if (y == EOF)
      return EOF;
    result += y;
  }
  return result;
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

//This function returns the square of the euclidean distance between 2 points.
double dist(cv::Point x,cv::Point y)
{
	return (x.x-y.x)*(x.x-y.x)+(x.y-y.y)*(x.y-y.y);
}


//This function returns the radius and the center of the circle given 3 points
//If a circle cannot be formed , it returns a zero radius circle centered at (0,0)
pair<cv::Point,double> circleFromPoints(cv::Point p1, cv::Point p2, cv::Point p3)
{
	double offset = pow(p2.x,2) +pow(p2.y,2);
	double bc =   ( pow(p1.x,2) + pow(p1.y,2) - offset )/2.0;
	double cd =   (offset - pow(p3.x, 2) - pow(p3.y, 2))/2.0;
	double det =  (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x)* (p1.y - p2.y); 
	double TOL = 0.0000001;
	if (abs(det) < TOL) { cout<<"POINTS TOO CLOSE"<<endl;return make_pair(cv::Point(0,0),0); }

	double idet = 1/det;
	double centerx =  (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
	double centery =  (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
	double radius = sqrt( pow(p2.x - centerx,2) + pow(p2.y-centery,2));

	return make_pair(cv::Point(centerx,centery),radius);
}

vector<Rect> detectFaces(Mat frame){
	        //convert captured image to gray scale and equalize
	        cvtColor(frame, grayscaleFrame, CV_BGR2GRAY);
	        equalizeHist(grayscaleFrame, grayscaleFrame);

	        //create a vector array to store the face found
	        std::vector<Rect> faces;

	        //find faces and store them in the vector array
	        face_cascade.detectMultiScale(grayscaleFrame, faces, 1.1, 3, CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_SCALE_IMAGE, Size(10,10));
	
			return faces;
}

vector<Rect> detectUpperBodies(Mat frame){
	        //convert captured image to gray scale and equalize
	        cvtColor(frame, grayscaleFrame, CV_BGR2GRAY);
	        equalizeHist(grayscaleFrame, grayscaleFrame);

	        //create a vector array to store the face found
	        std::vector<Rect> upperBodies;

	        //find faces and store them in the vector array
	        upper_body_cascade.detectMultiScale(grayscaleFrame, upperBodies, 1.05, 3, CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_SCALE_IMAGE, Size(10,10));
	
			return upperBodies;
}

CvRect rect_intersect(CvRect a, CvRect b) 
{ 
    CvRect r; 
    r.x = (a.x > b.x) ? a.x : b.x;
    r.y = (a.y > b.y) ? a.y : b.y;
    r.width = (a.x + a.width < b.x + b.width) ? 
        a.x + a.width - r.x : b.x + b.width - r.x; 
    r.height = (a.y + a.height < b.y + b.height) ? 
        a.y + a.height - r.y : b.y + b.height - r.y; 
    if(r.width <= 0 || r.height <= 0) 
        r = cvRect(0, 0, 0, 0); 

    return r; 
}

void reduceIllumination(Mat frame){
	///illumination solution
		cv::cvtColor(frame, frame, CV_BGR2YUV);
		std::vector<cv::Mat> channels;
		cv::split(frame, channels);
		cv::equalizeHist(channels[0], channels[0]);
		cv::merge(channels, frame);
		cv::cvtColor(frame, frame, CV_YUV2BGR);
}

   QImage  cvMatToQImage( const cv::Mat &inMat )
   {
      switch ( inMat.type() )
      {
         // 8-bit, 4 channel
         case CV_8UC4:
         {
            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB32 );

            return image;
         }

         // 8-bit, 3 channel
         case CV_8UC3:
         {
            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB888 );

            return image.rgbSwapped();
         }

         // 8-bit, 1 channel
         case CV_8UC1:
         {
            static QVector<QRgb>  sColorTable;

            // only create our color table once
            if ( sColorTable.isEmpty() )
            {
               for ( int i = 0; i < 256; ++i )
                  sColorTable.push_back( qRgb( i, i, i ) );
            }

            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_Indexed8 );

            image.setColorTable( sColorTable );

            return image;
         }

         default:
            //qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
            break;
      }

      return QImage();
   }

void wbw_robot_window_custom_function(ImageData *image_data){

	vector<Limb> potential_limbs;

scene->clear();

Mat frame = cv::Mat(120, 160, CV_8UC4, (unsigned char *)image_data->image_values);
cvtColor(frame,frame,CV_BGRA2BGR);   //from CV_8UC4 to CV_8UC3 (fails with _4)
//cv::imwrite( "image.png", frame );

resize(frame,frame,cv::Size(320, 240));//resize image

		vector<vector<cv::Point> > contours;

	
		//reduceIllumination(frame);
		
		vector<Rect> approx_faces = detectFaces(frame);
		vector<Rect> approx_upperBodies = detectUpperBodies(frame);

		//Update the current background model and get the foreground
		if(backgroundFrame>0)
		{bg.operator ()(frame,fore);backgroundFrame--;}
		else
		{bg.operator()(frame,fore,0);}

		//Get background image to display it
		bg.getBackgroundImage(back);


		//Enhance edges in the foreground by applying erosion and dilation
		erode(fore,fore,Mat());
		dilate(fore,fore,Mat());
		
		//imshow("Frame",fore);
		
		//Find the contours in the foreground
		findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_TC89_KCOS);
		vector<vector<cv::Point> > newcontours;
		
		//find biggest face
		cv::Point biggestUpperBodyCenter(-1, -1);
		Rect biggestUpperBody;
		Rect biggestFace;
		cv::Point biggestFaceCenter(-1, -1);
		float biggestFaceArea = 0.0;
		cv::Point possibleElbowStart(-1,-1);
		cv::Point possibleElbowEnd(-1,-1);
		
		if(approx_faces.size() > 0){
		biggestFace = approx_faces[0];
	    for (int i = 1; i < approx_faces.size(); i++) {
		  if((biggestFace.width * biggestFace.height) < (approx_faces[i].width * approx_faces[i].height))
			biggestFace = approx_faces[i];
	    }
	
		biggestFaceArea = biggestFace.width * biggestFace.height;
		biggestFaceCenter.x = biggestFace.x + biggestFace.width/2.0;
		biggestFaceCenter.y = biggestFace.y + biggestFace.height/2.0;
	
		cv::Point pt1(biggestFace.x + biggestFace.width, biggestFace.y + biggestFace.height);
		            cv::Point pt2(biggestFace.x, biggestFace.y);
	rectangle(frame, pt1, pt2, cvScalar(0, 255, 0, 0), 1, 8, 0);


		//find biggest upper body
		if(approx_upperBodies.size() > 0){
		biggestUpperBody = approx_upperBodies[0];
	    for (int i = 1; i < approx_upperBodies.size(); i++) {
		  if((biggestUpperBody.width * biggestUpperBody.height) < (approx_upperBodies[i].width * approx_upperBodies[i].height))
			biggestUpperBody = approx_upperBodies[i];
	    }	
	    
	    biggestUpperBodyCenter.x = biggestUpperBody.x + biggestUpperBody.width/2.0;
	    biggestUpperBodyCenter.y = biggestFace.y+biggestFace.height;
	//intersection between face and upper body
	//Rect head_upper_body_intersection = rect_intersect(biggestUpperBody, biggestFace);
		cv::Point pt3(biggestUpperBody.x + biggestUpperBody.width, biggestUpperBody.y + biggestUpperBody.height);
		            cv::Point pt4(biggestUpperBody.x, biggestFace.y+biggestFace.height);
	rectangle(frame, pt3, pt4, cvScalar(0, 255, 0, 0), 1, 8, 0);
}

		for(int i=0;i<contours.size();i++){
			RotatedRect rect_test=minAreaRect(Mat(contours[i]));
				Rect intersectionRectangle = rect_intersect(biggestFace, rect_test.boundingRect());
				if((intersectionRectangle.width * intersectionRectangle.height) > biggestFaceArea * 0.5){
					//contours.erase(contours.begin()+i);
					//printf("fingers on the face\n"); 
					//newcontours.push_back(contours[i]);
				}else{
					newcontours.push_back(contours[i]);
				}
		}
	}else{
		newcontours = contours;
	}
	
		for(int i=0;i<newcontours.size();i++)
			//Ignore all small insignificant areas
			if(contourArea(newcontours[i])>= (biggestFaceArea * 0.5) && (biggestFaceArea > 0.0)/*std::max(biggestFaceArea/100.0, 5000.0)*/)		    
			{
          			Limb limb;
          			//default
          			limb.start.x = -1;
          			limb.start.y = -1;
          			limb.end.x = -1;
          			limb.end.y = -1;
          			limb.break_point.x = -1;
          			limb.break_point.y = -1; 
				//Draw contour
				vector<vector<cv::Point> > tcontours;
				tcontours.push_back(newcontours[i]);
				drawContours(frame,tcontours,-1,cv::Scalar(0,0,255),2);

				//Detect Hull in current contour
				vector<vector<cv::Point> > hulls(1);
				vector<vector<int> > hullsI(1);
				convexHull(Mat(tcontours[0]),hulls[0],false);
				convexHull(Mat(tcontours[0]),hullsI[0],false);
				drawContours(frame,hulls,-1,cv::Scalar(0,255,0),2);

				//Find minimum area rectangle to enclose hand
				RotatedRect rect=minAreaRect(Mat(tcontours[0]));
				Point2f vertices[4];
				rect.points(vertices);
				
				//is a limb?
          			//RotatedRect potential_limb_contour = minAreaRect(Mat(newcontours[i]));    
          			biggestUpperBody.y = biggestFace.y+biggestFace.height;
          			biggestUpperBody.height = biggestUpperBody.height-biggestFace.height;
          			
          			Rect potential_limb_intersections = rect_intersect(rect.boundingRect(), biggestUpperBody);
          			if(potential_limb_intersections.width * potential_limb_intersections.height > rect.boundingRect().width * rect.boundingRect().height * 0.1){
                    			for(int m=0;m<4;m++)
                              		  if(potential_limb_intersections.contains((vertices[m] + vertices[(m+1)%4])*.5)/* && potential_limb_intersections.contains(vertices[(m+1)%4])*/)
                              		  {
                              		    //Limb limb;
                              		    limb.start = (vertices[m] + vertices[(m+1)%4])*.5;
                              		    limb.end = (vertices[(m+2)%4] + vertices[(m+3)%4])*.5;
                              		    limb.limb_bounding_rectangle = rect;
                              		    //potential_limbs.push_back(limb);
                              		  }
          			}

				//Find Convex Defects
				vector<Vec4i> defects;
				if(hullsI[0].size()>0)
				{
					Point2f rect_points[4]; rect.points( rect_points );
					for( int j = 0; j < 4; j++ )
						line( frame, rect_points[j], rect_points[(j+1)%4], Scalar(255,0,0), 1, 8 );
					cv::Point rough_palm_center;
					convexityDefects(tcontours[0], hullsI[0], defects);
					if(defects.size()>=3)
					{
						vector<cv::Point> palm_points;
						for(int j=0;j<defects.size();j++)
						{
							int startidx=defects[j][0]; cv::Point ptStart( tcontours[0][startidx] );
							int endidx=defects[j][1]; cv::Point ptEnd( tcontours[0][endidx] );
							int faridx=defects[j][2]; cv::Point ptFar( tcontours[0][faridx] );
							//Sum up all the hull and defect points to compute average
							rough_palm_center+=ptFar+ptStart+ptEnd;
							palm_points.push_back(ptFar);
							palm_points.push_back(ptStart);
							palm_points.push_back(ptEnd);
						}

						//Get palm center by 1st getting the average of all defect points, this is the rough palm center,
						//Then U chose the closest 3 points ang get the circle radius and center formed from them which is the palm center.
						rough_palm_center.x/=defects.size()*3;
						rough_palm_center.y/=defects.size()*3;
						cv::Point closest_pt=palm_points[0];
						vector<pair<double,int> > distvec;
						for(int i=0;i<palm_points.size();i++)
							distvec.push_back(make_pair(dist(rough_palm_center,palm_points[i]),i));
						sort(distvec.begin(),distvec.end());

						//Keep choosing 3 points till you find a circle with a valid radius
						//As there is a high chance that the closes points might be in a linear line or too close that it forms a very large circle
						pair<cv::Point,double> soln_circle;
						for(int i=0;i+2<distvec.size();i++)
						{
							cv::Point p1=palm_points[distvec[i+0].second];
							cv::Point p2=palm_points[distvec[i+1].second];
							cv::Point p3=palm_points[distvec[i+2].second];
							soln_circle=circleFromPoints(p1,p2,p3);//Final palm center,radius
							if(soln_circle.second!=0)
								break;
						}

						//Find avg palm centers for the last few frames to stabilize its centers, also find the avg radius
						palm_centers.push_back(soln_circle);
						if(palm_centers.size()>10)
							palm_centers.erase(palm_centers.begin());
						
						cv::Point palm_center;
						double radius=0;
						for(int i=0;i<palm_centers.size();i++)
						{
							palm_center+=palm_centers[i].first;
							radius+=palm_centers[i].second;
						}
						palm_center.x/=palm_centers.size();
						palm_center.y/=palm_centers.size();
						radius/=palm_centers.size();

						//Draw the palm center and the palm circle
						//The size of the palm gives the depth of the hand
						//circle(frame,palm_center,5,Scalar(144,144,255),3);
						//circle(frame,palm_center,radius,Scalar(144,144,255),2);

						//Detect fingers by finding points that form an almost isosceles triangle with certain thesholds
						int no_of_fingers=0;
						for(int j=0;j<defects.size();j++)
						{
							int startidx=defects[j][0]; cv::Point ptStart( tcontours[0][startidx] );
							int endidx=defects[j][1]; cv::Point ptEnd( tcontours[0][endidx] );
							int faridx=defects[j][2]; cv::Point ptFar( tcontours[0][faridx] );
							//X o--------------------------o Y
							double Xdist=sqrt(dist(palm_center,ptFar));
							double Ydist=sqrt(dist(palm_center,ptStart));
							double length=sqrt(dist(ptFar,ptStart));

							double retLength=sqrt(dist(ptEnd,ptFar));
							//Play with these thresholds to improve performance
							if(length<=3*radius&&Ydist>=0.4*radius&&length>=10&&retLength>=10&&max(length,retLength)/min(length,retLength)>=0.8)
								if(min(Xdist,Ydist)/max(Xdist,Ydist)<=0.8)
								{
									if((Xdist>=0.1*radius&&Xdist<=1.3*radius&&Xdist<Ydist)||(Ydist>=0.1*radius&&Ydist<=1.3*radius&&Xdist>Ydist)){
										line( frame, ptEnd, ptFar, Scalar(0,255,0), 1 );
										no_of_fingers++;
										
										if(dist(ptEnd, ptFar) > dist(possibleElbowStart, possibleElbowEnd) && rect.boundingRect().x+rect.boundingRect().width - ptFar.x >= rect.boundingRect().width*0.6){
											possibleElbowStart = ptEnd;
											possibleElbowEnd = ptFar;
										}
									}
								}


						}
						
						//no_of_fingers=min(5,no_of_fingers);
						//cout<<"NO OF FINGERS: "<<no_of_fingers<<endl;

						
					}
				}
				
				limb.break_point = possibleElbowStart;
				potential_limbs.push_back(limb);

			}
		//if(backgroundFrame>0)
		//	putText(frame, "Recording Background", cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
		//imshow("Frame",frame);
		//imshow("Background",back);
		//if(waitKey(10) >= 0) break;
		

		QImage result = cvMatToQImage(frame);
		QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(result));
		item->setPos(-300, -300);
                   scene->addItem(item);
                   
                   cv::Point initialPoint(0, -300);
                   double rad = 5;
                   //face
                   scene->addEllipse( initialPoint.x+biggestFaceCenter.x-rad, initialPoint.y+biggestFaceCenter.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern) );

                   if((biggestUpperBodyCenter.x != -1) || (biggestUpperBodyCenter.y != -1)){                     
                     //neck
                     scene->addEllipse( initialPoint.x+biggestUpperBodyCenter.x-rad, initialPoint.y+biggestUpperBodyCenter.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern) );
                     scene->addLine(initialPoint.x+biggestFaceCenter.x, initialPoint.y+biggestFaceCenter.y,  initialPoint.x+biggestUpperBodyCenter.x, initialPoint.y+biggestUpperBodyCenter.y, QPen());                   
                   
                   
                     //shoulders
                     scene->addEllipse( initialPoint.x+biggestUpperBodyCenter.x-biggestUpperBody.width/2.0-rad, initialPoint.y+biggestUpperBodyCenter.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern) );
                     scene->addLine(initialPoint.x+biggestUpperBodyCenter.x, initialPoint.y+biggestUpperBodyCenter.y, initialPoint.x+biggestUpperBodyCenter.x-biggestUpperBody.width/2.0, initialPoint.y+biggestUpperBodyCenter.y, QPen());
                     scene->addEllipse( initialPoint.x+biggestUpperBodyCenter.x+biggestUpperBody.width/2.0-rad, initialPoint.y+biggestUpperBodyCenter.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern)  );
                     scene->addLine(initialPoint.x+biggestUpperBodyCenter.x, initialPoint.y+biggestUpperBodyCenter.y, initialPoint.x+biggestUpperBodyCenter.x+biggestUpperBody.width/2.0, initialPoint.y+biggestUpperBodyCenter.y, QPen());
                   }
                   
                   for(int p=0; p<potential_limbs.size(); p++){
                     if(potential_limbs[p].break_point.x != -1 && potential_limbs[p].break_point.y != -1 && potential_limbs[p].start.x != -1 && potential_limbs[p].start.y != -1 && potential_limbs[p].end.x != -1 && potential_limbs[p].end.y != -1){
                       scene->addEllipse( initialPoint.x+potential_limbs[p].break_point.x-rad, initialPoint.y+potential_limbs[p].break_point.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern) );
                       scene->addLine(initialPoint.x+initialPoint.x+potential_limbs[p].start.x, initialPoint.y+initialPoint.x+potential_limbs[p].start.y, initialPoint.x+initialPoint.x+potential_limbs[p].break_point.x, initialPoint.y+initialPoint.x+potential_limbs[p].break_point.y, QPen());                     
                       scene->addEllipse( initialPoint.x+potential_limbs[p].end.x-rad, initialPoint.y+potential_limbs[p].end.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern) );
                       scene->addLine(initialPoint.x+potential_limbs[p].break_point.x, initialPoint.y+initialPoint.x+potential_limbs[p].break_point.y, initialPoint.x+initialPoint.x+potential_limbs[p].end.x, initialPoint.y+initialPoint.x+potential_limbs[p].end.y, QPen());                     
                     }else{
                       if(potential_limbs[p].start.x != -1 && potential_limbs[p].start.y != -1 && potential_limbs[p].end.x != -1 && potential_limbs[p].end.y != -1){
                         scene->addEllipse( initialPoint.x+potential_limbs[p].end.x-rad, initialPoint.y+potential_limbs[p].end.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern) );
                         scene->addLine(initialPoint.x+initialPoint.x+potential_limbs[p].start.x, initialPoint.y+initialPoint.x+potential_limbs[p].start.y, initialPoint.x+initialPoint.x+potential_limbs[p].end.x, initialPoint.y+initialPoint.x+potential_limbs[p].end.y, QPen());
                       }
                     }
                   }
}