////gcc stable_version_gesture_recognition.cpp -o hand -I/usr/local/Cellar/opencv/2.4.10.1/include -L/usr/local/Cellar/opencv/2.4.10.1/lib -lopencv_objdetect -lopencv_core -lopencv_highgui -lopencv_video -lopencv_imgproc -L/usr/lib -lc++ -lc++abi -lm -lc -I/usr/include -lopencv_photo -lopencv_contrib -I/Library/Frameworks/Python.framework/Versions/2.7/include/ -lpython2.7 -L/Library/Frameworks/Python.framework/Versions/2.7/lib/python2.7/config -std=c++11

#include "graph.h"
#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<algorithm>
#include <numeric>
#include <time.h>

using namespace std;
using namespace cv;

//create the cascade classifier object used for the face detection
CascadeClassifier face_cascade;
CascadeClassifier upper_body_cascade;

//setup image files used in the capture process
Mat grayscaleFrame;
	
// structure to hold detected limbs
typedef struct Limbs
{
	cv::Point  start;
	cv::Point  end;
	RotatedRect limb_bounding_rectangle;
	cv::Point break_point;
	vector<Point> details;
} Limb;
		

// Returns the square of the euclidean distance between 2 points.
double dist(Point x,Point y)
{
	return (x.x-y.x)*(x.x-y.x)+(x.y-y.y)*(x.y-y.y);
}

// Returns whether a point lies on the contour
int lies_on_contour(vector<vector<Point> > contours, Point point)
{
	for(size_t n = 0; n < contours.size(); ++n)
	{
	    auto i = find(contours[n].begin(), contours[n].end(), point);
	    if(contours[n].end() != i)
			return 1;
	}	
	
	return 0;
}


//This function returns the radius and the center of the circle given 3 points
//If a circle cannot be formed , it returns a zero radius circle centered at (0,0)
pair<Point,double> circleFromPoints(Point p1, Point p2, Point p3)
{
	double offset = pow(p2.x,2) +pow(p2.y,2);
	double bc =   ( pow(p1.x,2) + pow(p1.y,2) - offset )/2.0;
	double cd =   (offset - pow(p3.x, 2) - pow(p3.y, 2))/2.0;
	double det =  (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x)* (p1.y - p2.y); 
	double TOL = 0.0000001;
	if (abs(det) < TOL) { cout<<"POINTS TOO CLOSE"<<endl;return make_pair(Point(0,0),0); }

	double idet = 1/det;
	double centerx =  (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
	double centery =  (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
	double radius = sqrt( pow(p2.x - centerx,2) + pow(p2.y-centery,2));

	return make_pair(Point(centerx,centery),radius);
}

// Load face classification data
void initFaces(){
	face_cascade.load("/path_to_open_cv/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml");
	upper_body_cascade.load("/path_to_open_cv/share/OpenCV/haarcascades/haarcascade_mcs_upperbody.xml");
}

//function detects faces on a frame and returns a vector with rectangles that correspond to the detected faces
vector<Rect> detectFaces(Mat frame){
	//convert captured image to gray scale and equalize
	cvtColor(frame, grayscaleFrame, CV_BGR2GRAY);
	equalizeHist(grayscaleFrame, grayscaleFrame);

	//create a vector array to store the face found
	std::vector<Rect> faces;

	//find faces and store them in the vector array
	face_cascade.detectMultiScale(grayscaleFrame, faces, 1.05, 3, CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_SCALE_IMAGE, Size(10,10));
	
	return faces;
}

// function detects upper bodies on a frame and recurns a vector with rectangles that correspond to the detected upper bodies
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

// function takes in two rectangles (a and b) and returns a rectangle from the intersection between rectangles a and b
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

// function reduces illumination by normalising the intensity values of the frame brightness (Y - lumma)
void reduceIllumination(Mat frame){
	cv::cvtColor(frame, frame, CV_BGR2YUV);
	std::vector<cv::Mat> channels;
	cv::split(frame, channels);
	cv::equalizeHist(channels[0], channels[0]);
	cv::merge(channels, frame);
	cv::cvtColor(frame, frame, CV_YUV2BGR);
}

// Adds an edge to an undirected graph
void addEdge(struct Graph* graph, int src, cv::Point src_point, int dest, cv::Point dest_point, float weight)
{
    // Add an edge from src to dest.  A new node is added to the adjacency
    // list of src.  The node is added at the begining
    struct AdjListNode* newNode = newAdjListNode(dest_point, dest, weight);
    newNode->next = graph->array[src].head;
    graph->array[src].head = newNode;
 
    // Since graph is undirected, add an edge from dest to src also
    newNode = newAdjListNode(src_point, src, weight);
    newNode->next = graph->array[dest].head;
    graph->array[dest].head = newNode;

    //keeps number of vertices added to the graph
	graph->vertices_added++;
}

/* This procedure captures stream/pre-recorded video from caera/file, extracts features and creates a graph for every frame
   Then graphs are concattenated together to produce a string of feature graphs.
   Function returns a number of frames in a pre-recorded video
*/
int start_capture(int mode, vector<Graph> &string_feature_graph, int limit, float &diagonal_size, int duration){
	
	Mat frame;
	Mat back;
	Mat fore;
	vector<pair<Point,double> > palm_centers;
	
	VideoCapture cap;
	
	// 2 filenames, one for original video, another for query video. The algorithm will try to find query video in original video
	string filename = "gesture_original.mov";
	string filename2 = "gesture_query.mov";
	
	//Three modes: 0 - start pre-recorded video original; 1 - start camera stream; 2- start pre-recorded video query
	if(mode == 0){
		cap = VideoCapture(filename);
		cap.set(CV_CAP_PROP_FPS, 3);
	}
	else if(mode == 1){
		cap = VideoCapture(0);
		cap.set(CV_CAP_PROP_FPS, 3);
	}
	else if(mode == 2){
		cap = VideoCapture(filename2);
		cap.set(CV_CAP_PROP_FPS, 3);
	}
	
	if( !cap.isOpened() )
		return -1;

	//Supporting class that does background substraction
	BackgroundSubtractorMOG2 bg;
	bg.set("nmixtures",3);
	bg.set("detectShadows",false);


	//initialise windows to show results
	namedWindow("Frame");
	namedWindow("Background");
	
	//interval for background update (needed in case if the background changes)
	int backgroundFrame=500;
	
	//calculate the diagonal size of the frame (needed for as a factor when calculating distance between features in different frames)
	double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	diagonal_size = sqrt(width*width + height*height);
	
	int frames=0;
	
	//for a number of frames or until the end of the video
		for(int r =0;r<mode==1 ? duration : INFINITY;r++)
		{		
			//gesture feature hierarchy (0 - no features found)
			int hierarchy = 0;

			//vector that holds potential limbs
			vector<Limb> potential_limbs;
			
			//contours of the detected movement
			vector<vector<Point> > contours;

			//Get the frame from either a pre-recorded video or a camera stream
			cap >> frame;
			
			//pre-recorded video ended - return number of frames
			if(frame.empty() || frames==limit)
	            return frames;

			frames++;	
			
			//reduceIllumination(frame);

			//two vectors that hold detected faces and upper bodies
			vector<Rect> approx_faces = detectFaces(frame);
			vector<Rect> approx_upperBodies = detectUpperBodies(frame);

			//Update the current background model and extract the foreground
			if(backgroundFrame>0)
			{bg.operator ()(frame,fore);backgroundFrame--;}
			else
			{bg.operator()(frame,fore,0);}

			//Get background image to display it
			bg.getBackgroundImage(back);


			//Enhance edges in the foreground by applying erosion and dilation
			erode(fore,fore,Mat());
			dilate(fore,fore,Mat());

			//Find the contours in the foreground
			findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_TC89_KCOS);
			vector<vector<Point> > newcontours;

				//find biggest face of all faces detected (only one person is supported in this version)
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

					//find biggest upper body of all upper bodies deteted (only one person is supported in this version)
					if(approx_upperBodies.size() > 0){
						//increase features hierarchy level
						hierarchy = 1;

					biggestUpperBody = approx_upperBodies[0];
				    for (int i = 1; i < approx_upperBodies.size(); i++) {
					  if((biggestUpperBody.width * biggestUpperBody.height) < (approx_upperBodies[i].width * approx_upperBodies[i].height))
						biggestUpperBody = approx_upperBodies[i];
				    }	

				    biggestUpperBodyCenter.x = biggestUpperBody.x + biggestUpperBody.width/2.0;
				    biggestUpperBodyCenter.y = biggestFace.y+biggestFace.height;
	
					cv::Point pt3(biggestUpperBody.x + biggestUpperBody.width, biggestUpperBody.y + biggestUpperBody.height);
					cv::Point pt4(biggestUpperBody.x, biggestFace.y+biggestFace.height);
			}
			
			// get rid of the contours of the foreground that are on the top of the biggest detected face (restrict current implementation)
			for(int i=0;i<contours.size();i++){
				RotatedRect rect_test=minAreaRect(Mat(contours[i]));
					Rect intersectionRectangle = rect_intersect(biggestFace, rect_test.boundingRect());
					if((intersectionRectangle.width * intersectionRectangle.height) > biggestFaceArea * 0.5){
						//...
					}else{
						newcontours.push_back(contours[i]);
					}
			}
		}else{
			newcontours = contours;
		}

			for(int i=0;i<newcontours.size();i++)
				//Ignore all small insignificant areas (currently use 30% of the biggest area face)
				if(contourArea(newcontours[i])>=(biggestFaceArea * 0.3) && (biggestFaceArea > 0.0))		    
				{
					possibleElbowStart.x = -1;
					possibleElbowStart.y = -1;
					possibleElbowEnd.x = -1;
					possibleElbowEnd.y = -1;

					vector<Point2f> limb_details_temp;

					Limb limb;
	      			//default limb
	      			limb.start.x = -1;
	      			limb.start.y = -1;
	      			limb.end.x = -1;
	      			limb.end.y = -1;
	      			limb.break_point.x = -1;
	      			limb.break_point.y = -1;
					//Draw contour
					vector<vector<Point> > tcontours;
					tcontours.push_back(newcontours[i]);
					//drawContours(frame,tcontours,-1,cv::Scalar(0,0,255),2);

					//Detect Hull in current contour
					vector<vector<Point> > hulls(1);
					vector<vector<int> > hullsI(1);
					convexHull(Mat(tcontours[0]),hulls[0],false);
					convexHull(Mat(tcontours[0]),hullsI[0],false);
					//drawContours(frame,hulls,-1,cv::Scalar(0,255,0),2);

					//Find minimum area rectangle to enclose hand
					RotatedRect rect=minAreaRect(Mat(tcontours[0]));
					Point2f vertices[4];
					rect.points(vertices);

					//is a limb?
	          			//RotatedRect potential_limb_contour = minAreaRect(Mat(newcontours[i]));  					
	          			biggestUpperBody.y = biggestFace.y+biggestFace.height;
	          			biggestUpperBody.height = biggestUpperBody.height-biggestFace.height;

						Rect biggestUpperBodyTemp(biggestUpperBody.tl(), biggestUpperBody.size());  //use copy for expansion

						//extends biggest upper body in width (to increase confidence in the intersection between hand and the upper body)
						cv::Size deltaSize( biggestUpperBodyTemp.width *.5, biggestUpperBodyTemp.height);
						cv::Point offset( deltaSize.width/2, deltaSize.height/2);
						biggestUpperBodyTemp += deltaSize;
						biggestUpperBodyTemp -= offset;

	          			Rect potential_limb_intersections = rect_intersect(rect.boundingRect(), biggestUpperBodyTemp);

						//extend intersection rectangle
						cv::Size deltaSize2( potential_limb_intersections.width * .1, potential_limb_intersections.height * .1 );
						cv::Point offset2( deltaSize2.width/2, deltaSize2.height/2);
						potential_limb_intersections += deltaSize2;
						potential_limb_intersections -= offset2;

	          			if(potential_limb_intersections.width * potential_limb_intersections.height > rect.boundingRect().width * rect.boundingRect().height * 0.1){
							//increase detected features hierarchy level
							hierarchy = 2;
	                    			for(int m=0;m<4;m++)
	                              		  if(dist(limb.start, limb.end) < dist((vertices[m] + vertices[(m+1)%4])*.5, (vertices[(m+2)%4] + vertices[(m+3)%4])*.5) && potential_limb_intersections.contains((vertices[m] + vertices[(m+1)%4])*.5) /* && potential_limb_intersections.contains(vertices[(m+1)%4])*/)
	                              		  {
	                              		    limb.start = (vertices[m] + vertices[(m+1)%4])*.5;
	                              		    limb.end = (vertices[(m+2)%4] + vertices[(m+3)%4])*.5;
	                              		    limb.limb_bounding_rectangle = rect;
	                              		  }
	          			}

					//Find Convex Defects
					vector<Vec4i> defects;
					if(hullsI[0].size()>0)
					{
						Point2f rect_points[4]; rect.points( rect_points );
						//for( int j = 0; j < 4; j++ )
						//	line( frame, rect_points[j], rect_points[(j+1)%4], Scalar(255,0,0), 1, 8 );
						Point rough_palm_center;
						convexityDefects(tcontours[0], hullsI[0], defects);
						if(defects.size()>=3)
						{
							vector<Point> palm_points;
							for(int j=0;j<defects.size();j++)
							{
								int startidx=defects[j][0]; Point ptStart( tcontours[0][startidx] );
								int endidx=defects[j][1]; Point ptEnd( tcontours[0][endidx] );
								int faridx=defects[j][2]; Point ptFar( tcontours[0][faridx] );

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
							Point closest_pt=palm_points[0];
							vector<pair<double,int> > distvec;
							for(int i=0;i<palm_points.size();i++)
								distvec.push_back(make_pair(dist(rough_palm_center,palm_points[i]),i));
							sort(distvec.begin(),distvec.end());

							//Keep choosing 3 points till you find a circle with a valid radius
							//As there is a high chance that the closes points might be in a linear line or too close that it forms a very large circle
							pair<Point,double> soln_circle;
							for(int i=0;i+2<distvec.size();i++)
							{
								Point p1=palm_points[distvec[i+0].second];
								Point p2=palm_points[distvec[i+1].second];
								Point p3=palm_points[distvec[i+2].second];
								soln_circle=circleFromPoints(p1,p2,p3);//Final palm center,radius
								if(soln_circle.second!=0)
									break;
							}

							//Find avg palm centers for the last few frames to stabilize its centers, also find the avg radius
							palm_centers.push_back(soln_circle);
							if(palm_centers.size()>10)
								palm_centers.erase(palm_centers.begin());

							Point palm_center;
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
								int startidx=defects[j][0]; Point ptStart( tcontours[0][startidx] );
								int endidx=defects[j][1]; Point ptEnd( tcontours[0][endidx] );
								int faridx=defects[j][2]; Point ptFar( tcontours[0][faridx] );
								//X o--------------------------o Y
								double Xdist=sqrt(dist(palm_center,ptFar));
								double Ydist=sqrt(dist(palm_center,ptStart));
								double length=sqrt(dist(ptFar,ptStart));

								//circle(frame,ptStart,5,Scalar(0,0,255),3);

								double retLength=sqrt(dist(ptEnd,ptFar));
								//Play with these thresholds to improve performance
								if(length<=3*radius&&Ydist>=0.4*radius&&length>=10&&retLength>=10&&max(length,retLength)/min(length,retLength)>=0.8)
									if(min(Xdist,Ydist)/max(Xdist,Ydist)<=0.8)
									{
										if((Xdist>=0.1*radius&&Xdist<=1.3*radius&&Xdist<Ydist)||(Ydist>=0.1*radius&&Ydist<=1.3*radius&&Xdist>Ydist)){
											if(dist(ptEnd, limb.end) <= dist(limb.start, limb.end) * .1){
												hierarchy = 3;
											  limb_details_temp.push_back(Point2f(ptEnd.x, ptEnd.y));
											}

											//line( frame, ptEnd, ptFar, Scalar(0,255,0), 1 );
											no_of_fingers++;


											if(dist(ptStart, limb.start) >= std::min(rect.boundingRect().width, rect.boundingRect().height)*.2 && dist(ptStart, limb.end) >= std::min(rect.boundingRect().width, rect.boundingRect().height)*.2 && lies_on_contour(newcontours, ptStart) && dist(ptStart, ptEnd) > dist(possibleElbowStart, possibleElbowEnd)){
												possibleElbowStart = ptStart;
												possibleElbowEnd = ptEnd;
											}
										}

									}


							}


							//circle(frame,possibleElbowStart,5,Scalar(0,255,0),3);
							//circle(frame,possibleElbowEnd,5,Scalar(255,0,0),3);

							no_of_fingers=min(5,no_of_fingers);
							//cout<<"NO OF FINGERS: "<<no_of_fingers<<endl;


						}

					}

					//since detected hierarchy 3 data can be very dense, it can be clustered to identify only clusters instead of actual data
					if(limb_details_temp.size() > 0){
						Mat labels;
						int cluster_number = std::min(5, (int)limb_details_temp.size());
						TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0 );
						Mat centers;
						kmeans(limb_details_temp, cluster_number, labels, criteria, 1, KMEANS_PP_CENTERS, centers);

						for(int h=0;h<centers.rows;h++)
							limb.details.push_back(Point(centers.at<float>(h,0), centers.at<float>(h,1)));

				    }

					limb.break_point = possibleElbowStart;
					potential_limbs.push_back(limb);

				}

				//frame to be used to present a skeletal data from identified end-points
				Mat skeletonFrame = Mat(frame.rows*.5,frame.cols*.5, CV_8UC3, cv::Scalar(255,255,255));
				double rad = 5;
				double scale_factor = .5;
				
		        //face
		        //scene->addEllipse( initialPoint.x+biggestFaceCenter.x-rad, initialPoint.y+biggestFaceCenter.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern) );
				Point faceCenter(biggestFaceCenter.x*scale_factor, biggestFaceCenter.y*scale_factor);
				circle(skeletonFrame,faceCenter,rad,Scalar(0,0,0),2);

		        if((biggestUpperBodyCenter.x != -1) || (biggestUpperBodyCenter.y != -1)){                     
		          //neck
		          //scene->addEllipse( initialPoint.x+biggestUpperBodyCenter.x-rad, initialPoint.y+biggestUpperBodyCenter.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern) );
		          //scene->addLine(initialPoint.x+biggestFaceCenter.x, initialPoint.y+biggestFaceCenter.y,  initialPoint.x+biggestUpperBodyCenter.x, initialPoint.y+biggestUpperBodyCenter.y, QPen());                   
				  Point neckCenter(biggestUpperBodyCenter.x*scale_factor, biggestUpperBodyCenter.y*scale_factor);
				  circle(skeletonFrame,neckCenter,rad,Scalar(0,0,0),2);
				  line( skeletonFrame, faceCenter, neckCenter, Scalar(0,0,0), 1, 8 );

		          //shoulders
		          //scene->addEllipse( initialPoint.x+biggestUpperBodyCenter.x-biggestUpperBody.width/2.0-rad, initialPoint.y+biggestUpperBodyCenter.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern) );
		          //scene->addLine(initialPoint.x+biggestUpperBodyCenter.x, initialPoint.y+biggestUpperBodyCenter.y, initialPoint.x+biggestUpperBodyCenter.x-biggestUpperBody.width/2.0, initialPoint.y+biggestUpperBodyCenter.y, QPen());
		          //scene->addEllipse( initialPoint.x+biggestUpperBodyCenter.x+biggestUpperBody.width/2.0-rad, initialPoint.y+biggestUpperBodyCenter.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern)  );
		          //scene->addLine(initialPoint.x+biggestUpperBodyCenter.x, initialPoint.y+biggestUpperBodyCenter.y, initialPoint.x+biggestUpperBodyCenter.x+biggestUpperBody.width/2.0, initialPoint.y+biggestUpperBodyCenter.y, QPen());

				  Point shoulder1Center((biggestUpperBodyCenter.x-biggestUpperBody.width/2.0)*scale_factor, biggestUpperBodyCenter.y*scale_factor);
				  circle(skeletonFrame,shoulder1Center,rad,Scalar(0,0,0),2);
				  Point shoulder2Center((biggestUpperBodyCenter.x+biggestUpperBody.width/2.0)*scale_factor, biggestUpperBodyCenter.y*scale_factor);
				  circle(skeletonFrame,shoulder2Center,rad,Scalar(0,0,0),2);

				  //waist
				  Point waistCenter((biggestUpperBody.tl().x + biggestUpperBody.width/2.0)*scale_factor, (biggestUpperBody.tl().y+biggestUpperBody.height)*scale_factor);
				  circle(skeletonFrame,waistCenter,rad,Scalar(0,0,0),2);
		        }

		        for(int p=0; p<potential_limbs.size(); p++){
		          //scene->addEllipse( initialPoint.x+potential_limbs[p].end.x-rad, initialPoint.y+potential_limbs[p].end.y-rad, rad*2.0, rad*2.0, QPen(), QBrush(Qt::SolidPattern) );
		          //scene->addLine(initialPoint.x+initialPoint.x+potential_limbs[p].start.x, initialPoint.y+initialPoint.x+potential_limbs[p].start.y, initialPoint.x+initialPoint.x+potential_limbs[p].end.x, initialPoint.y+initialPoint.x+potential_limbs[p].end.y, QPen());

				  if(potential_limbs[p].break_point.x != -1 && potential_limbs[p].break_point.y != -1 && potential_limbs[p].start.x != -1 && potential_limbs[p].start.y != -1 && potential_limbs[p].end.x != -1 && potential_limbs[p].end.y != -1){
					Point limbEnd(potential_limbs[p].end.x*scale_factor, potential_limbs[p].end.y*scale_factor);
				  	circle(skeletonFrame,limbEnd,rad,Scalar(0,0,255),2);
					Point limbMiddle(potential_limbs[p].break_point.x*scale_factor, potential_limbs[p].break_point.y*scale_factor);
				  	circle(skeletonFrame,limbMiddle,rad,Scalar(0,0,0),2);
				  	Point limbStart(potential_limbs[p].start.x*scale_factor, potential_limbs[p].start.y*scale_factor);
				  	line( skeletonFrame, limbStart, limbMiddle, Scalar(0,0,255), 1, 8 );
					line( skeletonFrame, limbMiddle, limbEnd, Scalar(0,0,0), 1, 8 );
				  }else{
					  if(potential_limbs[p].start.x != -1 && potential_limbs[p].start.y != -1 && potential_limbs[p].end.x != -1 && potential_limbs[p].end.y != -1){
				  		Point limbEnd(potential_limbs[p].end.x*scale_factor, potential_limbs[p].end.y*scale_factor);
				  		circle(skeletonFrame,limbEnd,rad,Scalar(0,0,0),2);
				  		Point limbStart(potential_limbs[p].start.x*scale_factor, potential_limbs[p].start.y*scale_factor);
				  		line( skeletonFrame, limbStart, limbEnd, Scalar(0,0,0), 1, 8 );
				      }
			  	  }

				  //details
				  for(int l=0; l<potential_limbs[p].details.size(); l++){
					if(dist(potential_limbs[p].details[l], potential_limbs[p].break_point) != 0.0){
						Point limb_detal(potential_limbs[p].details[l].x*scale_factor, potential_limbs[p].details[l].y*scale_factor);
						circle(skeletonFrame,limb_detal,rad,Scalar(0,0,0),2);
					}
				  }
		        }

					/*	
					Below is the construction of the feature graphs (FG)
					*/
					
					//0-face
					//1-hand1
					//2-hand2
					//3-shoulder1
					//4-shoulder2
					//5-elbow1
					//6-elbow2
					
		    		// create the graph given in above fugure
	        		int V = 7;
	        		struct Graph* graph = createGraph(V);

					//features
					int item = 1;

					//hierarchy
					std::stringstream s00;
					s00 << "hierarchy: " << hierarchy;
					putText(skeletonFrame, s00.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

					//face-hand
					std::stringstream s0;
					s0 << "face-hand-1: ";
					if(potential_limbs.size() > 0){
						s0 << (int)round(sqrt(dist(faceCenter, potential_limbs[0].end)));
						addEdge(graph, 0, faceCenter, 1, potential_limbs[0].end, sqrt(dist(faceCenter, potential_limbs[0].end))); //face-hand1
					}
					else{
						s0 << "-";
						//addEdge(graph, 0, Point(0,0), 1, Point(0,0), 0); //face-hand1
					}
					putText(skeletonFrame, s0.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
					std::stringstream s;
					s << "face-hand-2: ";
					if(potential_limbs.size() > 1){
						s << (int)round(sqrt(dist(faceCenter, potential_limbs[1].end)));
						addEdge(graph, 0, faceCenter, 2, potential_limbs[1].end, sqrt(dist(faceCenter, potential_limbs[1].end))); //face-hand2
					}else{
						s << "-";
						//addEdge(graph, 0, Point(0,0), 2, Point(0,0), 0); //face-hand2
					}
					putText(skeletonFrame, s.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);


					//hand-hand
					std::stringstream s1;
					s1 << "hand-hand: ";
					if(potential_limbs.size() == 2){
						s1 << (int)round(sqrt(dist(potential_limbs[1].end, potential_limbs[0].end)));
						addEdge(graph, 1, potential_limbs[1].end, 2, potential_limbs[0].end, sqrt(dist(potential_limbs[1].end, potential_limbs[0].end))); //hand1-hand2 (same as hand2-hand1)
					}else{
						s1 << "-";
						//addEdge(graph, 1, Point(0,0), 2, Point(0,0), 0); //hand1-hand2 (same as hand2-hand1)
					}
					putText(skeletonFrame, s1.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

					//shoulder-shoulder
					Point shoulder1((biggestUpperBody.x + biggestUpperBody.width/2.0-biggestUpperBody.width/2.0)*scale_factor, (biggestFace.y+biggestFace.height)*scale_factor);
					Point shoulder2((biggestUpperBody.x + biggestUpperBody.width/2.0+biggestUpperBody.width/2.0)*scale_factor, (biggestFace.y+biggestFace.height)*scale_factor);
					std::stringstream s2;
					s2 << "shoulder-shoulder: " << (int)round(sqrt(dist(shoulder1, shoulder2)));
					putText(skeletonFrame, s2.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
					addEdge(graph, 3, shoulder1, 4, shoulder2, sqrt(dist(shoulder1, shoulder2))); //shoulder1-shoulder2 (same as shoulder2-shoudler1)

					//elbow-elbow
					std::stringstream s3;
					s3 << "elbow-elbow: ";
					if(potential_limbs.size() == 2){
						s3 << (int)round(sqrt(dist(potential_limbs[0].break_point, potential_limbs[1].break_point)));
						addEdge(graph, 5, potential_limbs[0].break_point, 6, potential_limbs[1].break_point, sqrt(dist(potential_limbs[0].break_point, potential_limbs[1].break_point))); //elbow1-elbow2 (same as elbow2-elbow1)
					}else{
						s3 << "-";
						//addEdge(graph, 5, Point(0,0), 6, Point(0,0), 0); //elbow1-elbow2 (same as elbow2-elbow1)
					}
					putText(skeletonFrame, s3.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);


					imshow("Frame2",skeletonFrame);



	        		// print the adjacency list representation of the above graph
	        		//printGraph(graph);
					string_feature_graph.push_back(*graph);

					//if(backgroundFrame>0)
					//	putText(frame, "Recording Background", cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
					putText(frame, "Preview", cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
					imshow("Frame",frame);
					//imshow("Background",back);
					if(waitKey(10) >= 0) break;


		}
		
		return -1;
}

/** * Determines the angle of a straight line drawn between point one and two.
 The number returned, which is a float in degrees, 
 tells us how much we have to rotate a horizontal line clockwise for it to match the line between the two points. * 
 If you prefer to deal with angles using radians instead of degrees, 
 just change the last line to: "return atan2(yDiff, xDiff);" */ 
float GetAngleOfLineBetweenTwoPoints(cv::Point p1, cv::Point p2) 
{ 
	float xDiff = p2.x - p1.x;
	float yDiff = p2.y - p1.y; 
	
	return atan2(yDiff, xDiff) * (180 / M_PI); 
}

//finds eigenvectors of an affinity matrix
vector<vector<float> > calcEigen(IplImage* src)
{
	vector<vector<float> > eigenvectors;
		
	if(src->height != src->width)
	{
		printf("\nDimensions must be same for eigen!!");
		return eigenvectors;
	}
	
	int j,i;
	float **p;
	CvScalar se;

    //creating the 2D array - this part will work for unequal dimensions also
	p = (float**)malloc(src->height*sizeof(float*));
	for(i=0; i<src->height; i++)
		p[i] = (float*)malloc(src->width*sizeof(float));


	float *a;  //1D array reqd to convert IplImage to CvMat (grrrr)
	a = (float*)malloc( (src->height) * (src->width) * sizeof(float) );
	long int k=0;
	
	//image pixels into 2D array
	for(i = 0; i < src->height; ++i)
	{
		for(j = 0; j <src->width; ++j)
		{
			se = cvGet2D(src, i, j);
			p[i][j] = (float)se.val[0];
		}

	}

	//2D array into 1D array
	for(i = 0; i < src->height; ++i)
		for(j = 0; j <src->width; ++j)
		{
			a[k++] = p[i][j];
		}
 
	//1D datapoints into CvMat
	CvMat mat = cvMat(src->height,src->height,CV_32FC1, a); 
	/*the resulting CvMat is used for many computations*/
		for(i = 0; i < src->height; ++i)
			for(j = 0; j <src->width; ++j)
			{
				float t = (float)cvmGet(&mat,i,j); // Get M(i,j)
				printf("\t%f",t);
				//a[k++] = p[i][j];
			}

		/*Eigenvalue specific code begins*/
        CvScalar scal;
		CvMat* evec  = cvCreateMat(src->height,src->height,CV_32FC1); //eigenvectors
		CvMat* eval  = cvCreateMat(1,src->height,CV_32FC1);  //eigenvalues (1xN)
		cvZero(evec);
		cvZero(eval);
		cvEigenVV(&mat, evec, eval, 0);	
				
			//only 1st eigenvector is chosen (associated with greates eigenvalue) -> Principal eigenvector
			for( int j = 0; j < 1/*eval->cols*/; j++ )
			{
			// 	/*access the obtained eigenvalues*/
			//  scal = cvGet2D( eval, 0, j );
			// 	printf( "\n%f\n", scal.val[0]);
			// 	
				vector<float> eigenvector;
			 	for(int i=0;i < evec->rows;i++){   
			 		//printf("cluster %d, frame %d, feature %d similarity: %f \n", j, i/7, i%7, cvmGet(evec,j,i)); //Fetching each component of Eigenvector i    
					eigenvector.push_back(cvmGet(evec,j,i));
			   	}
				eigenvectors.push_back(eigenvector);
			}
 
		//cvReleaseMat(&evec);
		//cvReleaseMat(&eval);
		
		return eigenvectors;
}

// function constructs affinity matrix for two string of feature graphs
Mat match_strings(vector<Graph> test, vector<Graph> query, float max_length){
	Mat M = Mat(test.size()*7,test.size()*7, CV_32F, cvScalar(0.));
	
	//iterate both strings of feature graphs
	for(std::vector<Graph>::iterator test_it = test.begin(); test_it != test.end(); ++test_it) {
		for(std::vector<Graph>::iterator query_it = query.begin(); query_it != query.end(); ++query_it) {
			if((&(*test_it))->vertices_added == (&(*query_it))->vertices_added){
		    	int v;
				
				//iterate both feature graphs adjacency lists
		    	for (v = 0; v < (&(*test_it))->V; ++v)
		    	{
					int v2;
				    for (v2 = 0; v2 < (&(*query_it))->V; ++v2)
				    {
		    	    	struct AdjListNode* pCrawl = (&(*test_it))->array[v].head;
		    	    	while (pCrawl)
		    	    	{
					        struct AdjListNode* pCrawl2 = (&(*query_it))->array[v2].head;
					        while (pCrawl2)
					        {
								// diagonal - similarity between feature end-point distances
								if((pCrawl->dest) == (pCrawl2->dest)){
									float dn = sqrt(dist(pCrawl->position, pCrawl2->position))/max_length;
									
									// maximum allowed deviation is 1% of the frame diagonal
									if(dn <= max_length*0.01/max_length){
										if(max_length > 0.0)
											M.at<float>(std::distance(test.begin(), test_it)*7+pCrawl->dest, std::distance(query.begin(), query_it)*7+pCrawl2->dest) = max_length*0.01/max_length - dn;
										else
											M.at<float>(std::distance(test.begin(), test_it)*7+pCrawl->dest, std::distance(query.begin(), query_it)*7+pCrawl2->dest) = 0.0;
									}else{
										M.at<float>(std::distance(test.begin(), test_it)*7+pCrawl->dest, std::distance(query.begin(), query_it)*7+pCrawl2->dest) = 0.0;
									}
								}else{
									float de1 = 0.0;
									float de2 = 0.0;
									
									for(std::vector<Graph>::iterator test_it_dest = test.begin(); test_it_dest != test.end(); ++test_it_dest) {
										int v_dest;
									    for (v_dest = 0; v_dest < (&(*test_it_dest))->V; ++v_dest)
									    {
									        struct AdjListNode* pCrawl_test_dest = (&(*test_it_dest))->array[v_dest].head;
									        while (pCrawl_test_dest)
									        {
												if(pCrawl_test_dest->dest == pCrawl->dest)
													de1 = GetAngleOfLineBetweenTwoPoints(pCrawl->position, pCrawl_test_dest->position) / 360.0;
												pCrawl_test_dest = pCrawl_test_dest->next;
											}
										}
									}
									
									for(std::vector<Graph>::iterator query_it_dest = query.begin(); query_it_dest != query.end(); ++query_it_dest) {
										int v2_dest;
									    for (v2_dest = 0; v2_dest < (&(*query_it_dest))->V; ++v2_dest)
									    {
									        struct AdjListNode* pCrawl_query_dest = (&(*query_it_dest))->array[v2_dest].head;
									        while (pCrawl_query_dest)
									        {
												if(pCrawl_query_dest->dest == pCrawl2->dest)
													de2 = GetAngleOfLineBetweenTwoPoints(pCrawl2->position, pCrawl_query_dest->position) / 360.0;
												pCrawl_query_dest = pCrawl_query_dest->next; 
											}
										}
									}
									
									
									float de_diff = abs(de1-de2);
									
									// 3,6% maximum deviation of inclination between line edge connecting two feature end-points 
									if(de_diff/360.0 <= 360.0*0.01/360.0){
										M.at<float>(std::distance(test.begin(), test_it)*7+pCrawl->dest, std::distance(query.begin(), query_it)*7+pCrawl2->dest) = 360.0*0.01/360.0 - de_diff/360.0;
										M.at<float>(std::distance(query.begin(), query_it)*7+pCrawl2->dest, std::distance(test.begin(), test_it)*7+pCrawl->dest) = 360.0*0.01/360.0 - de_diff/360.0; //symmetry
									}else{
										M.at<float>(std::distance(test.begin(), test_it)*7+pCrawl->dest, std::distance(query.begin(),query_it)*7+pCrawl2->dest) = 0.0;
										M.at<float>(std::distance(query.begin(),query_it)*7+pCrawl2->dest, std::distance(test.begin(),test_it)*7+pCrawl->dest) = 0.0;
									}
								}
					
					            pCrawl2 = pCrawl2->next;
					        }
							pCrawl = pCrawl->next;
					    }
		    	    }
		    	}
			}
		}
	}
	
	return M;
}

// function that performs dynamic time warping and outputs cost matrix
Mat dtw(const std::vector<float>& t1, const std::vector<float>& t2, Mat distance, int i) {
    int m = t1.size();
    int n = t2.size();

    // create cost matrix
    Mat cost = Mat(m, n, CV_32FC1, cvScalar(0.));

	cost.at<float>(0,0) = distance.at<float>(0,0);

    // calculate first row
    for(int i2 = 1; i2 < m; i2++)
        	cost.at<float>(i2,0) = cost.at<float>(i2-1,0) + distance.at<float>(i2,0);
    // calculate first column
    for(int j = 1; j < n; j++)
        	cost.at<float>(0,j) = /*cost.at<float>(0,j-1) +*/ distance.at<float>(0,j);
    // fill matrix
    for(int i2 = 1; i2 < m; i2++)
        for(int j = 1; j < n; j++)
            	cost.at<float>(i2,j) = std::min(cost.at<float>(i2,j-1), std::min(cost.at<float>(i2-1,j), cost.at<float>(i2-1,j-1))) 
                	+ distance.at<float>(i2,j);

	return cost;
}

//function binarises eigenvector
vector<float> binirise_eigenvector(vector<float> vector){
	for(int j=0;j<vector.size();j++){
		
		//find largest value
		float largest = vector[0];
		int largest_index = 0;
		for(int i=1; i<vector.size();i++){
			if(largest < vector[i] && vector[i] != 1.0){
				largest = vector[i];
				largest_index = i;
			}
		}
		
		if(largest != 0.0 && largest != 1.0){
			//set largest to 1
			vector[largest_index] = 1.0;
			
			//set all similar occurences to 0
			for(int i=0; i<vector.size();i++){
				if(largest == vector[i]){
					vector[i] = 0.0;
				}
			}
		}
	}
	
	return vector;
}

int main(int argc, char *argv[])
{
	//BOF original video
	int mode = 0;

	//two vectors to hold string of feature graphs
	vector<Graph> string_feature_graph_demo;
	vector<Graph> string_feature_graph_test;

	//initialise face recognitions
	initFaces();
	
	//default frame diagonal size
	float diagonal_size = 0.0;

	//capture
	int frames = start_capture(mode, string_feature_graph_demo, -1, diagonal_size, 80);
	
	//EOF original video
	
	//BOF query video
	mode = 2;
	
	//capture
	start_capture(mode, string_feature_graph_test, frames, diagonal_size, 80);
	
	//EOF query video
	
	//timer	
	clock_t start = clock(), diff;
	
	
	//align by filling with empty graphs
	int size_a = string_feature_graph_demo.size();
	int size_b = string_feature_graph_test.size();
	
	if(size_a != size_b){
		if(size_a < size_b){
			for(int i=0;i<size_b-size_a;i++){
				int V=7;
				struct Graph* graph = createGraph(V);
				string_feature_graph_demo.push_back(*graph);
			}
		}else{
			for(int i=0;i<size_a-size_b;i++){
				int V=7;
				struct Graph* graph = createGraph(V);
				string_feature_graph_test.push_back(*graph);
			}
		}
	}
	
	//construct affinity matrices
	Mat M = match_strings(string_feature_graph_demo, string_feature_graph_test, diagonal_size);
	Mat M2 = match_strings(string_feature_graph_test, string_feature_graph_test, diagonal_size);
	
	//make copies for binaries
	Mat M_copy(M.size(),M.type());
	M.copyTo(M_copy);
	
	Mat M2_copy(M2.size(),M2.type());
	M2.copyTo(M2_copy);
	
	//getting only 1 principal eigenvector for both M and M2
	vector<vector<float> > eigenM = calcEigen(new IplImage(M_copy));
	vector<vector<float> > eigenM2 = calcEigen(new IplImage(M2_copy));
	 
	if(eigenM.size() == eigenM2.size())
	for(int i= 0; i < eigenM.size(); i++)
	{		
		int n = eigenM[i].size();
		int m = eigenM2[i].size();
		
		//making copies
		vector< float > eigenM_copy2 = eigenM[i];
		vector< float > eigenM2_copy2 = eigenM2[i];
		
		//binarising eigenvectors
		vector<float> binirised_M = binirise_eigenvector(eigenM[i]);
		vector<float> binirised_M2 = binirise_eigenvector(eigenM2[i]);
		
		//copying to Mat object (easier to work with using OpenCV API)
		Mat cvt(binirised_M, true);
		Mat cvt2(binirised_M2, true);
		
		// //sim Q P 
		Mat cvt11 = Mat(n,m,CV_32F,cvScalar(0.)); 
		cvt11 = M * (cvt * cvt.t());
		
		// //sim Q Q
		Mat cvt22 = Mat(n,m,CV_32F,cvScalar(0.)); 
		cvt22 = M2 * (cvt2 * cvt2.t());
		
		//Mat cvt3 = cvt/cvt2;
		cv::Mat cvt3 = Mat(cvt11.rows,cvt11.cols,CV_32F,cvScalar(0.)); 
		int k,l;
		for(k=0; k<cvt11.rows; k++)
		   for(l=0; l<cvt11.cols; l++)
		   	if(cvt22.at<float>(k,l) != 0.0)
		   		cvt3.at<float>(k,l) = cvt11.at<float>(k,l) / cvt22.at<float>(k,l);
		   	else
		   		cvt3.at<float>(k,l) = 0.0;
		
		//1.0 - cvt3
		Mat ones = Mat(cvt11.rows,cvt11.cols, CV_32F, (float)1.0);
		cv::Mat cvt4 = Mat(cvt11.rows,cvt11.cols,CV_32F,cvScalar(0.));
		for(int q=0;q<cvt11.rows;q++)
	    {
	        for(int w=0;w<cvt11.cols;w++)
	        {
	            cvt4.at<float>(q,w) = ones.at<float>(q,w) - cvt3.at<float>(q,w);
	        }
	    }
		
		//perform dynamic time warping and obtain cost matrix
		Mat cost_path = dtw(eigenM_copy2, eigenM2_copy2, cvt4, i); //only cvt4 (distance) is actually used in dtw		
		
		int i2 = cost_path.rows-1;
		int j = cost_path.cols-1;

		cv::Mat path = Mat(cost_path.rows,cost_path.cols,CV_32F,cvScalar(0.)); 

		//find cheapest path over the cost matrix
		path.at<float>(i2, j) = 1.0;
		
		while (i2>0 || j>0){
					float diagCost;
					float leftCost;
					float downCost;
					
					//avoid can be used for exclusively vertical or exclusively horisontal movements
					int avoid = 0;
					
					if ((i2>0) && (j>0)){
					            diagCost = cost_path.at<float>(i2-1, j-1);
					}else
					            		diagCost = INFINITY;
					
					if (i2 > 0)
					   			leftCost = cost_path.at<float>(i2-1, j);
					else
					   			leftCost = INFINITY;
					
					if (j > 0)
					   			downCost = cost_path.at<float>(i2, j-1);
					else
					   			downCost = INFINITY;
					
					
							if ((diagCost<=leftCost) && (diagCost<=downCost))
							         {
							            i2--;
							            j--;
							         }
							         else if ((leftCost<diagCost) && (leftCost<downCost)){
										i2--;
										avoid = 1;
							         }else if ((downCost<diagCost) && (downCost<leftCost)){
										j--;
										avoid = 1;
							         }else if (i2 <= j){  // leftCost==rightCost > diagCost
										j--;
										avoid = 1;
							         }else{   // leftCost==rightCost > diagCost
										i2--;
										avoid = 1;
									 }
									
							         // Add the current step to the warp path.
									 if(/*avoid == 0 && */diagCost == diagCost && leftCost == leftCost && downCost == downCost)
							         	path.at<float>(i2, j) = 1.0;
				}
		
		///BOF TESTING 
		CvCapture* capture = cvCaptureFromFile("gesture_original.mov");
		cv::VideoCapture capture11("gesture_original.mov");
		CvCapture* capture2 = cvCaptureFromFile("gesture_query.mov");
		cv::VideoCapture capture22("gesture_query.mov");
		IplImage* frame = NULL;
		IplImage* frame2 = NULL;

		for(int k=0;k<path.rows; k++){
			for(int l=0;l<path.cols;l++){
				if(path.at<float>(k,l) == 1.0 && abs(cvt4.at<float>(k,l)) < 1.0){
					cout << floor(floor(k/7))  << " <-> " << floor(floor(l/7)) << endl;
					cout << "distance: " << cvt4.at<float>(k,l) << endl;
					//	
					// frame = skipNFrames(capture, k/7);
					// frame2 = skipNFrames(capture2, l/7);
					// if( frame != NULL && frame2 != NULL){
					//   		
					// 		int dstWidth=frame->width+frame->width;
					// 		int dstHeight=frame->height;
					//   		
					// 		IplImage* dst=cvCreateImage(cvSize(dstWidth,dstHeight),IPL_DEPTH_8U,3);
					//   		
					// 		cvSetImageROI(dst, cvRect(0, 0,frame->width,frame->height) );
					// 		cvCopy(frame,dst,NULL);
					// 		cvResetImageROI(dst);
					//   		
					// 		cvSetImageROI(dst, cvRect(frame2->width, 0,frame2->width,frame2->height) );
					// 		cvCopy(frame2,dst,NULL);
					// 		cvResetImageROI(dst);
					//   		
					// 		char str2[80];
					// 		snprintf(str2, sizeof str2, "similar_frames_%d_%d_distance_%f.png", k,l, cvt4.at<float>(k,l));
					// 		imwrite( str2, Mat(dst) );
					//  	
					// 		break;
					// }else{ printf("frames are NULL\n"); }
				}
			}
		}
	}
	///EOF TESTING 
	
	diff = clock() - start;

	int msec = diff * 1000 / CLOCKS_PER_SEC;
	printf("Time taken %d seconds %d milliseconds", msec/1000, msec%1000);
	
	return 0;
}
