//ghc genotype.c -c && ghc -c -O Safe.hs && ghc --make -no-hs-main -optc-O -I/usr/local/Cellar/opencv/2.4.10.1/include -L/usr/local/Cellar/opencv/2.4.10.1/lib -lopencv_objdetect -lopencv_core -lopencv_highgui -lopencv_video -lopencv_imgproc -L/usr/lib -lc++ -lc++abi -lm -lc -I/usr/include -lopencv_photo -lopencv_contrib -I/Library/Frameworks/Python.framework/Versions/2.7/include/ -lpython2.7 -L/Library/Frameworks/Python.framework/Versions/2.7/lib/python2.7/config -I/Applications/ghc-7.10.1.app/Contents/lib/ghc-7.10.1/include stable_version_gesture_recognition.cpp Safe -c && ghc --make -no-hs-main stable_version_gesture_recognition.o genotype.o Safe -o hand -I/usr/local/Cellar/opencv/2.4.10.1/include -L/usr/local/Cellar/opencv/2.4.10.1/lib -lopencv_objdetect -lopencv_core -lopencv_highgui -lopencv_video -lopencv_imgproc -L/usr/lib -lc++ -lc++abi -lm -lc -I/usr/include -lopencv_photo -lopencv_contrib -I/Library/Frameworks/Python.framework/Versions/2.7/include/ -lpython2.7 -L/Library/Frameworks/Python.framework/Versions/2.7/lib/python2.7/config -I/Applications/ghc-7.10.1.app/Contents/lib/ghc-7.10.1/include -lopencv_flann

#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<algorithm>
#include <numeric>
#include <fstream>

#include <HsFFI.h>
#ifdef __GLASGOW_HASKELL__
#include "Safe_stub.h"
extern "C" {
	void __stginit_Safe(void);
}
#endif

extern "C" {
#include "graph.h"
#include "genotype.h"
}

#if (defined _WIN32 || defined __WIN64)
    #include <windows.h>
#elif defined __APPLE__    
    #include <spawn.h>
    #include <sys/types.h>
    #include <unistd.h>
#endif


#include <sys/stat.h>

// various paths based on OS
#if (defined _WIN32 || defined __WIN64)
    static char *GENOTYPE_FILE_NAME = "genes";

    static char *GENOTYPE_FITNESS_FILE_NAME = "genes_fitness";

    static char *BEST_GENOTYPE_FILE_NAME = "best_solution";

    static char *PYTHON_PATH = "C:\\Windows\\py.exe ";

    static char *DETECTOR_FILENAME = "detector";

    static char *NN_FILENAME = "classifier\\net";
#elif defined __APPLE__
    static char *GENOTYPE_FILE_NAME = "genes";

    static char *GENOTYPE_FITNESS_FILE_NAME = "genes_fitness";

    static char *BEST_GENOTYPE_FILE_NAME = "best_solution";

    static char *DETECTOR_FILENAME = "detector";

    static char *NN_FILENAME = "classifier/net";
#else
    static char *GENOTYPE_FILE_NAME = "genes";

    static char *GENOTYPE_FITNESS_FILE_NAME = "genes_fitness";

    static char *BEST_GENOTYPE_FILE_NAME = "best_solution";

    static char *DETECTOR_FILENAME = "detector";

    static char *NN_FILENAME = "classifier/net";
#endif

//For peas
extern char **environ;

// detector evolution script, peas framework
#if (defined _WIN32 || defined __WIN64)
    const char *python_prog = "peas\\test\\feature_detector";
#elif defined __APPLE__
    const char *python_prog = "peas/test/feature_detector";
#else
    const char *python_prog = "";
#endif

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

// Checks whether a file is empty
int isEmpty(FILE *file)
{
    long savedOffset = ftell(file);
    fseek(file, 0, SEEK_END);
    
    if (ftell(file) == 0)
    {
        return 1;
    }
    
    fseek(file, savedOffset, SEEK_SET);
    return 0;
}

// checks whether file exists
int file_exist(char *filename)
{
    struct stat   buffer;
    return (stat (filename, &buffer) == 0);
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
	if (abs(det) < TOL) { return make_pair(Point(0,0),0); }

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
int start_capture(int mode, vector<Graph> &string_feature_graph, int limit, float &diagonal_size, int duration, int video_nr){
	//Matrices to hold frame, background and foreground
	Mat frame;
	Mat back;
	Mat fore;
	
	// holds detected palm centres 
	vector<pair<Point,double> > palm_centers;
	
	VideoCapture cap;
	
	//construct file name from given parameters for a pre-recorded video file
	std::stringstream ss;
	ss << video_nr;
	std::string full_filename;
	string filename = "data/test_data/test_data_";
	full_filename.append(filename);
	full_filename.append(ss.str());
	full_filename.append(".mov");
	
	//Two modes: 0 - start pre-recorded video; 1 - start camera stream
	if(mode == 0){
		cap = VideoCapture(full_filename);
	}
	else if(mode == 1){
		cap = VideoCapture(0);
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
			
			//frame = frame + Scalar(75, 75, 75); //increase the brightness by 75 units
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
			}

			// get rid of the contours of the foreground that are on the top of the biggest detected face (restrict current implementation)
			for(int i=0;i<contours.size();i++){
				RotatedRect rect_test=minAreaRect(Mat(contours[i]));
					Rect intersectionRectangle = rect_intersect(biggestFace, rect_test.boundingRect());
					if((intersectionRectangle.width * intersectionRectangle.height) > biggestFaceArea * 0.5){
						//..
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
	                        	if(dist(limb.start, limb.end) < dist((vertices[m] + vertices[(m+1)%4])*.5, (vertices[(m+2)%4] + vertices[(m+3)%4])*.5) && potential_limb_intersections.contains((vertices[m] + vertices[(m+1)%4])*.5))
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
												//increment hierarchy level
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
				Point faceCenter(biggestFaceCenter.x*scale_factor, biggestFaceCenter.y*scale_factor);
				circle(skeletonFrame,faceCenter,rad,Scalar(0,255,0),2);

		        if((biggestUpperBodyCenter.x != -1) || (biggestUpperBodyCenter.y != -1)){                     
		          //neck
		          Point neckCenter(biggestUpperBodyCenter.x*scale_factor, biggestUpperBodyCenter.y*scale_factor);
				  circle(skeletonFrame,neckCenter,rad,Scalar(0,0,255),2);
				  line( skeletonFrame, faceCenter, neckCenter, Scalar(0,0,0), 1, 8 );

		          //shoulders
		          Point shoulder1Center((biggestUpperBodyCenter.x-biggestUpperBody.width/2.0)*scale_factor, biggestUpperBodyCenter.y*scale_factor);
				  circle(skeletonFrame,shoulder1Center,rad,Scalar(0,0,255),2);
				  line( skeletonFrame, shoulder1Center, neckCenter, Scalar(0,0,0), 1, 8 );
				  Point shoulder2Center((biggestUpperBodyCenter.x+biggestUpperBody.width/2.0)*scale_factor, biggestUpperBodyCenter.y*scale_factor);
				  circle(skeletonFrame,shoulder2Center,rad,Scalar(0,0,255),2);
				  line( skeletonFrame, shoulder2Center, neckCenter, Scalar(0,0,0), 1, 8 );

				  //waist
				  Point waistCenter((biggestUpperBody.tl().x + biggestUpperBody.width/2.0)*scale_factor, (biggestUpperBody.tl().y+biggestUpperBody.height)*scale_factor);
				  circle(skeletonFrame,waistCenter,rad,Scalar(0,0,255),2);
				  line( skeletonFrame, shoulder1Center, waistCenter, Scalar(0,0,0), 1, 8 );
				  line( skeletonFrame, shoulder2Center, waistCenter, Scalar(0,0,0), 1, 8 );
		        }

				// limbs
		        for(int p=0; p<potential_limbs.size(); p++){
		          if(potential_limbs[p].break_point.x != -1 && potential_limbs[p].break_point.y != -1 && potential_limbs[p].start.x != -1 && potential_limbs[p].start.y != -1 && potential_limbs[p].end.x != -1 && potential_limbs[p].end.y != -1){
					Point limbEnd(potential_limbs[p].end.x*scale_factor, potential_limbs[p].end.y*scale_factor);
				  	circle(skeletonFrame,limbEnd,rad,Scalar(0,0,0),2);
					Point limbMiddle(potential_limbs[p].break_point.x*scale_factor, potential_limbs[p].break_point.y*scale_factor);
				  	circle(skeletonFrame,limbMiddle,rad,Scalar(0,0,0),2);
				  	Point limbStart(potential_limbs[p].start.x*scale_factor, potential_limbs[p].start.y*scale_factor);
				  	line( skeletonFrame, limbStart, limbMiddle, Scalar(0,0,0), 1, 8 );
					line( skeletonFrame, limbMiddle, limbEnd, Scalar(0,0,0), 1, 8 );
				  }else{
					  if(potential_limbs[p].start.x != -1 && potential_limbs[p].start.y != -1 && potential_limbs[p].end.x != -1 && potential_limbs[p].end.y != -1){
				  		Point limbEnd(potential_limbs[p].end.x*scale_factor, potential_limbs[p].end.y*scale_factor);
				  		circle(skeletonFrame,limbEnd,rad,Scalar(0,0,0),2);
				  		Point limbStart(potential_limbs[p].start.x*scale_factor, potential_limbs[p].start.y*scale_factor);
				  		line( skeletonFrame, limbStart, limbEnd, Scalar(0,0,0), 1, 8 );
				      }
			  	  }

				  //limb details
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
					
					//feature end-points: 
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
					putText(skeletonFrame, s00.str(), cvPoint(500,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,0), 1, CV_AA);

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
					//putText(skeletonFrame, s0.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
					std::stringstream s;
					s << "face-hand-2: ";
					if(potential_limbs.size() > 1){
						s << (int)round(sqrt(dist(faceCenter, potential_limbs[1].end)));
						addEdge(graph, 0, faceCenter, 2, potential_limbs[1].end, sqrt(dist(faceCenter, potential_limbs[1].end))); //face-hand2
					}else{
						s << "-";
						//addEdge(graph, 0, Point(0,0), 2, Point(0,0), 0); //face-hand2
					}
					//putText(skeletonFrame, s.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);


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
					//putText(skeletonFrame, s1.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

					//shoulder-shoulder
					Point shoulder1((biggestUpperBody.x + biggestUpperBody.width/2.0-biggestUpperBody.width/2.0)*scale_factor, (biggestFace.y+biggestFace.height)*scale_factor);
					Point shoulder2((biggestUpperBody.x + biggestUpperBody.width/2.0+biggestUpperBody.width/2.0)*scale_factor, (biggestFace.y+biggestFace.height)*scale_factor);
					std::stringstream s2;
					s2 << "shoulder-shoulder: " << (int)round(sqrt(dist(shoulder1, shoulder2)));
					//putText(skeletonFrame, s2.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
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
					//putText(skeletonFrame, s3.str(), cvPoint(30,15*(item++)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

					// show skeleton frame
					imshow("Frame2",skeletonFrame);

	        		// print the adjacency list representation of the above graph
	        		//printGraph(graph);
					string_feature_graph.push_back(*graph);

					imshow("Frame",frame);  //shows frame after frame with pre-recorded or camera stream video together with identified feature end-points
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

// function reads genotype from fifo-queue. Currently function wastes CPU by continuously trying to read the file 
// After genotype is read, fifo-queue is removed
void read_ann(Genotype genotype, char *NEW_GENOTYPE_FILE_NAME, char *NEW_GENOTYPE_FITNESS_FILE_NAME, int with_removal, int node_types_specified) {	
    int times = 0;
    int redo = 1;
    while(redo){
        FILE *fp = fopen(NEW_GENOTYPE_FILE_NAME, "rb");
        if ( fp == NULL ){
            fclose(fp);
            sleep(1);
            continue;
            read_ann(genotype, NEW_GENOTYPE_FILE_NAME, NEW_GENOTYPE_FITNESS_FILE_NAME, with_removal, node_types_specified);
        }
        
        while ( !feof (fp) )
        {
            redo = 0;
            if(isEmpty(fp)){
            fclose(fp);
                continue;
            }else{
                times++;
                
                if(times <= 1){
                    int result = genotype_fread(genotype, fp, node_types_specified);
                    fclose(fp);
                    //removing file
					if(with_removal)
                    	remove(NEW_GENOTYPE_FILE_NAME);

                    break;
                }else{
                    break;
                }
            }
            
        }
    }
    
    if((with_removal && file_exist(NEW_GENOTYPE_FILE_NAME)) || (with_removal && file_exist(NEW_GENOTYPE_FITNESS_FILE_NAME))){
        read_ann(genotype, NEW_GENOTYPE_FILE_NAME, NEW_GENOTYPE_FITNESS_FILE_NAME, with_removal, node_types_specified);
    }
}

// function converts float to double, leaving 7 decimal digits
double round_to_decimal(float f) {
    char buf[42];
    sprintf(buf, "%.7g", f);
    return atof(buf);
}

// funtion feeds input to a neural network by using hnn (Haskell) package
double feed(Genotype genotype, Mat matrix, int type) {
	//reconstruct matrices from available data to a suitable format
	double* matrixzvector = (double *)malloc(sizeof(double)*(matrix.rows * matrix.cols));
	int mm=0;
	for (int nr=0; nr<matrix.rows; nr++){ 
	    for (int nc=0; nc<matrix.cols; nc++){ 
	        matrixzvector[mm]=round_to_decimal(matrix.at<float>(nr,nc));
	        mm=mm+1;
	    }
	}
	
	double* node_types = (double *)malloc(sizeof(double)*(int)sqrt(genotype_get_size()));
	double* flat_matrix = (double *)malloc(sizeof(double)*(genotype_get_size()));

    for (int i=0; i<(int)sqrt(genotype_get_size()); i++){
        node_types[i] = genotype_get_node_types(genotype)[i];
    }

    for (int i=0; i<genotype_get_size(); i++){
        flat_matrix[i] = genotype_get_genes(genotype)[i];
    }

	//result array
	double *res;
	
	int asize = genotype_get_size();
	int bsize = matrix.rows * matrix.cols;
	int csize = (int)sqrt(genotype_get_size());

	/*
		type - feedworward (0) or recurrent (1)
		asize - number of weights
		bsize - number of inputs
		csize - number of nodes
		flat_matrix - weights
		matrixzvector - inputs
		node_types - node types
		res - result array
	*/
	process_network_input(&type, &asize, &bsize, &csize, &(*flat_matrix), &(*matrixzvector), &(*node_types), &res);
	
	free(matrixzvector);
	free(node_types);
	free(flat_matrix);
	
	double result;
	if(type == 0){
		result = *(res+((int)*res));  //the last element of the result array from recurrent 
	}else{
		result = 1;
		double val = *(res+1);
		for(int i=2;i<=(int)*res;i++){
			if(val < *(res+i)){
				result = i;
				val = *(res+i);
			}
		}
	}
	
	return result;
}

int main(int argc, char *argv[])
{
	//initialise face recognitions
	
	initFaces();
	
	//initialisation of haskell insterface
	int i;
	    hs_init(&argc, &argv);
	#ifdef __GLASGOW_HASKELL__
	    hs_add_root(__stginit_Safe);
	#endif
	
	
	int a;
	printf("1 for training detectors, 2 for getting output from evolved detectors, 3 for training classifying ANN, 4 for testing evolved detectors together with trained classifier:");
	scanf("%d", &a);
	
	//BOF EVOLVING DETECTORS
	if(a == 1){
		//bof start hyperneat algorithms
		int generations = 50000;   //ignore - had been used to make sure that peas and gesture recognition algorithm will end simultaneously
		int detectors = 10; //specify number of detectors that want to be evolved (NOTE: in current version, the same amount of script files must exist, numbered 1..N)
		for(int detector_id=1;detector_id<=detectors;detector_id++){
			
			//bof specify names of files
			char id[1*sizeof(int)+1];
			sprintf(id, "_%d", detector_id);
			
			char * NEW_GENOTYPE_FILE_NAME = new char[std::strlen(GENOTYPE_FILE_NAME)+std::strlen(id)+1];
		    std::strcpy(NEW_GENOTYPE_FILE_NAME,GENOTYPE_FILE_NAME);
		    std::strcat(NEW_GENOTYPE_FILE_NAME,id);

			char * NEW_GENOTYPE_FITNESS_FILE_NAME = new char[std::strlen(GENOTYPE_FITNESS_FILE_NAME)+std::strlen(id)+1];
		    std::strcpy(NEW_GENOTYPE_FITNESS_FILE_NAME,GENOTYPE_FITNESS_FILE_NAME);
		    std::strcat(NEW_GENOTYPE_FITNESS_FILE_NAME,id);
			//eof specify names
			
			// specify name of the detector's evolutionary script
			int parentID = getpid();  // should child kill when evolution is done?
	    	char str[1*sizeof(double)];
	    	sprintf(str, "%d", parentID);
	    	char* name_with_extension = (char*)malloc(2+strlen(python_prog)+std::strlen(id)+3*sizeof(char)+1*sizeof(int)+1+sizeof(GENOTYPE_FILE_NAME)+1);
	    	strcpy(name_with_extension, python_prog);
	    	strcat(name_with_extension, id);	
	    	strcat(name_with_extension, ".py");
	    	strcat(name_with_extension, " ");
	    	strcat(name_with_extension, str);
	    	strcat(name_with_extension, " ");
	    	strcat(name_with_extension, GENOTYPE_FILE_NAME);
        	        	
	    	pid_t pid;
	    	char *argvv[] = {"sh", "-c", name_with_extension, NULL};
	    	int status;
        	
			//different process spawning methods used on different platforms
	    	#if (defined _WIN32 || defined __WIN64)
        	
	    	    STARTUPINFO si;
	    	    PROCESS_INFORMATION pi;
        	
	    	    ZeroMemory( &si, sizeof(si) );
	    	    si.cb = sizeof(si);
	    	    ZeroMemory( &pi, sizeof(pi) );
        	
	    	    char str2[strlen(PYTHON_PATH)+strlen(name_with_extension)];
	    	    strcpy(str2, PYTHON_PATH);
	    	    strcat(str2, name_with_extension);
        	
	    	    //0 instead of CREATE_NEW_CONSOLE shows outoput in parent console (i.e. Webots if started from Webots environment)
	    	    if (!CreateProcess(NULL, str2, NULL, NULL, FALSE, CREATE_NEW_CONSOLE, NULL, NULL, &si, &pi))
	    	    {
	    	        printf( "CreateProcess failed (%d).\n", GetLastError() );
	    	    }
        	
	    	    CloseHandle(pi.hThread);
        	
	    	#elif defined __APPLE__
	    	    status = posix_spawn(&pid, "/bin/sh", NULL, NULL, argvv, environ);
	    	#else
        	
	    	#endif
	
			free(name_with_extension);
		}
		//eof start hyperneat algorithms

		// for every generation, use every detector on an a set of images
		for(int j=0;j<generations;j++){			
			
			// 10 detectors, 4 gestures
			Mat detectors_outputs = Mat(10,4, CV_32F, cvScalar(0.));
			
			// STEP_1: read ANN, create affinity matrix, feed affinity matrix to ANN and collect outputs for every detector in a matrix (rows - detectors; columns - gesture classes)
			for(int detector_id=1;detector_id<=detectors;detector_id++){
				
				//bof specify names
				char id[1*sizeof(int)+1];
				sprintf(id, "_%d", detector_id);
				
				char * NEW_GENOTYPE_FILE_NAME = new char[std::strlen(GENOTYPE_FILE_NAME)+std::strlen(id)+1];
			    std::strcpy(NEW_GENOTYPE_FILE_NAME,GENOTYPE_FILE_NAME);
			    std::strcat(NEW_GENOTYPE_FILE_NAME,id);
        	
				char * NEW_GENOTYPE_FITNESS_FILE_NAME = new char[std::strlen(GENOTYPE_FITNESS_FILE_NAME)+std::strlen(id)+1];
			    std::strcpy(NEW_GENOTYPE_FITNESS_FILE_NAME,GENOTYPE_FITNESS_FILE_NAME);
			    std::strcat(NEW_GENOTYPE_FITNESS_FILE_NAME,id);
				//eof specify names
				
				//genotype size 442x442 maximum connections - for the recurrent network (although it can still act as a feedforward network)
				genotype_set_size(442*442);
        		Genotype ann = genotype_create();

				//read ANN that had been generated from peas framework
				read_ann(ann, NEW_GENOTYPE_FILE_NAME, NEW_GENOTYPE_FITNESS_FILE_NAME, 1, 1);
        		
				for(int sfg=1;sfg<=20;sfg++){
					
					int mode = 0; // 0 - pre-recorder videos; 1 - camera video stream
					vector<Graph> string_feature_graph_demo;  // string of feature graphs (string=vector | feature graphs=adjacentcy-list graph structure)
					float diagonal_size = 0.0;
					
					//in order to skip viewing pre-recorded videos every time, their corresponding affinity matrix is stored in a separate file
					std::stringstream ss;
					ss << sfg;
					std::string full_filename;
					string filename = "data/test_data/stored/saved_sfg_";
					full_filename.append(filename);
					full_filename.append(ss.str());
					full_filename.append(".txt");
					Mat resized_M;
					
					//if saved affinity matrix exists, read it instead of viewing pre-recorded videos
					if(!std::ifstream(full_filename)){
						int frames = start_capture(mode, string_feature_graph_demo, 80, diagonal_size, 80, sfg);
                    	
						//make affinity matrix
						Mat M = match_strings(string_feature_graph_demo, string_feature_graph_demo, diagonal_size);
                    	
						//resize the affinity matrix to match 21x21 detector's input
						Size size(21,21);
						resized_M = Mat(21,21, CV_32F, cvScalar(0.));
						resize(M,resized_M,size);
						
						FileStorage fs(full_filename, FileStorage::WRITE);
						fs << "mat" << resized_M;
						fs.release();
					}else{
						FileStorage fs(full_filename, FileStorage::READ);
						fs["mat"] >> resized_M;
						fs.release();
					}
					
					//feed affinity matrix into HyperNEAT generated ANN
					int type=0;
					double ann_output = feed(ann, resized_M, type);
				
					// round the output. NOTE: 0.5 means that the output neuron did not fire (sigmoid(0)==0.5)
					// there is something wrong with using simply 0, therefore value 0.001 is used instead
					if(ann_output > (double)0.5)
						detectors_outputs.at<float>(detector_id-1, floor((sfg-1)/5)) += (float)((float)1.0);
					else
						detectors_outputs.at<float>(detector_id-1, floor((sfg-1)/5)) += (float)0.001;
				}
			}
			
			//STEP_2: calculate fitness function
			for(int detector_id=1;detector_id<=detectors;detector_id++){
				
				//bof specify names
				char id[1*sizeof(int)+1];
				sprintf(id, "_%d", detector_id);
				
				char * NEW_GENOTYPE_FILE_NAME = new char[std::strlen(GENOTYPE_FILE_NAME)+std::strlen(id)+1];
			    std::strcpy(NEW_GENOTYPE_FILE_NAME,GENOTYPE_FILE_NAME);
			    std::strcat(NEW_GENOTYPE_FILE_NAME,id);
        	
				char * NEW_GENOTYPE_FITNESS_FILE_NAME = new char[std::strlen(GENOTYPE_FITNESS_FILE_NAME)+std::strlen(id)+1];
			    std::strcpy(NEW_GENOTYPE_FITNESS_FILE_NAME,GENOTYPE_FITNESS_FILE_NAME);
			    std::strcat(NEW_GENOTYPE_FITNESS_FILE_NAME,id);
				//eof specify names
				
				// matrix to hold all rounded outputs from detectors_x_gestures matrix (apart from the currently reviewed detector).
				// this matrix will be used to calculate nearest neighbours Manhattan distance from this detector to all others. 
				CvMat* pointsForSearch = cvCreateMat(detectors-1, 4, CV_32F);
				float* pointsForSearch_ = (float *)malloc((sizeof(float)*detectors*4)-(4*sizeof(float)));
				int deviation = 0;
				for(int detector_id_query=1;detector_id_query<=detectors;detector_id_query++){
					if(detector_id_query != detector_id){
						pointsForSearch_[(detector_id_query-1-deviation)*4+0] = detectors_outputs.at<float>(detector_id_query-1,0);
						pointsForSearch_[(detector_id_query-1-deviation)*4+1] = detectors_outputs.at<float>(detector_id_query-1,1);
						pointsForSearch_[(detector_id_query-1-deviation)*4+2] = detectors_outputs.at<float>(detector_id_query-1,2);
						pointsForSearch_[(detector_id_query-1-deviation)*4+3] = detectors_outputs.at<float>(detector_id_query-1,3);
					}else{
						deviation++;
					}
				}

				cvSetData(pointsForSearch, pointsForSearch_, pointsForSearch->step); ///detectorN -> detector1(row1): gesture1(col1); detector2(row2): gesture2(col2); ...
				
				//calculate manhattan distance to every other detector and accumulate Manhattan distances to all detectors into a single variable
				double total_distance = 0.0;
				for(int detector_id_query2=1;detector_id_query2<=detectors-1;detector_id_query2++){
					CvMat* a_row = cvCreateMat(1, 4, CV_32FC1);

					cvSet2D(a_row, 0, 0, cvGet2D(pointsForSearch, detector_id_query2-1, 0));
					cvSet2D(a_row, 0, 1, cvGet2D(pointsForSearch, detector_id_query2-1, 1));
					cvSet2D(a_row, 0, 2, cvGet2D(pointsForSearch, detector_id_query2-1, 2));
					cvSet2D(a_row, 0, 3, cvGet2D(pointsForSearch, detector_id_query2-1, 3));
					
					CvMat* a_detector = cvCreateMat(1, 4, CV_32FC1);
					float a_detector_data[4] = {detectors_outputs.at<float>(detector_id-1,0), detectors_outputs.at<float>(detector_id-1,1), detectors_outputs.at<float>(detector_id-1,2), detectors_outputs.at<float>(detector_id-1,3)};
					cvSetData(a_detector, a_detector_data, a_detector->step);
					
					double detectors_manhattan_distance = cvCalcEMD2(a_row, a_detector, CV_DIST_L1, 0, 0, 0, 0, 0) / (float)10.0;
					printf("manhatta distance from this detector to others %f\n", detectors_manhattan_distance);
					
					total_distance += detectors_manhattan_distance;
					
					cvReleaseMat(&a_row);
					cvReleaseMat(&a_detector);
				}
				
				// take average Manhattan distance
				double fitness = total_distance/(float)10.0;
				
				//write fitness to fifo-queue
				char output[50];
	    		snprintf(output,50,"%f",fitness);
        		
	    		FILE *file;
	    		file = fopen(NEW_GENOTYPE_FITNESS_FILE_NAME,"w");
	    		fprintf(file, output);
	    		fclose(file);
	
				free(pointsForSearch_);
				cvReleaseMat(&pointsForSearch);
			}
		}
	//EOF EVOLVING DETECTORS
	
	//BOF COLLECTING EVOLVED DETECTORS OUTPUTS FROM A SET OF TRAINING VIDEOS
	}else if(a == 2){
		
		//holds detectors genotypes
		vector<Genotype> detectors;
		
		//for every stored training video
		for(int sfg=1;sfg<=20;sfg++){
			
			//collect detectors outputs
			vector<double> detector_output_per_video;
			
			//for 6 evolved detectors (should be 10, but not always all 10 detectors are evolved - some may not reach minimum fitness)
			for(int i=1; i<=6; i++){
				
				char id[1*sizeof(int)+1];
				sprintf(id, "_%d", i);

				char * NEW_DETECTOR_FILE_NAME = new char[std::strlen(DETECTOR_FILENAME)+std::strlen(id)+1];
			    std::strcpy(NEW_DETECTOR_FILE_NAME,DETECTOR_FILENAME);
			    std::strcat(NEW_DETECTOR_FILE_NAME,id);

				genotype_set_size(442*442);
				Genotype ann = genotype_create();


				char * NEW_GENOTYPE_FITNESS_FILE_NAME = new char[std::strlen(GENOTYPE_FITNESS_FILE_NAME)+std::strlen(id)+1];
			    std::strcpy(NEW_GENOTYPE_FITNESS_FILE_NAME,GENOTYPE_FITNESS_FILE_NAME);
			    std::strcat(NEW_GENOTYPE_FITNESS_FILE_NAME,id);

				//for every detector, read its ANN from file
				read_ann(ann, NEW_DETECTOR_FILE_NAME, NEW_GENOTYPE_FITNESS_FILE_NAME, 0, 1);
				
				int mode = 0; //using pre-recorded videos
				
				//string of feature graphs
				vector<Graph> string_feature_graph_demo;
				
				float diagonal_size = 0.0;
				
				//use pre-recorded raw video files or previously constructed and saved affinity matrices
				std::stringstream ss;
				ss << sfg;
				std::string full_filename;
				string filename = "data/test_data/stored/saved_sfg_";
				full_filename.append(filename);
				full_filename.append(ss.str());
				full_filename.append(".txt");
				Mat resized_M;
				if(!std::ifstream(full_filename)){
					int frames = start_capture(mode, string_feature_graph_demo, 80/*-1*/, diagonal_size, 80, sfg);
                	
					//make affinity matrix
					Mat M = match_strings(string_feature_graph_demo, string_feature_graph_demo, diagonal_size);
            
					Size size(21,21);
					resized_M = Mat(21,21, CV_32F, cvScalar(0.));
					resize(M,resized_M,size);
					
					FileStorage fs(full_filename, FileStorage::WRITE);
					fs << "mat" << resized_M;
					fs.release();
				}else{
					FileStorage fs(full_filename, FileStorage::READ);
					fs["mat"] >> resized_M;
					fs.release();
				}

				//collect detectors outputs
				int type=0;
				double ann_output = feed(ann, resized_M, type);
				
				detector_output_per_video.push_back(ann_output);
			}
			
			//print out the outputs. Should be collected manually and a training/testing data set constructed for the next level - classifier ANN training/testing
			printf("video %d : ", sfg);
			for (auto vec : detector_output_per_video)
				std::cout << vec << " | ";
			printf("\n");
		}
	//EOF COLLECTING EVOLVED DETECTORS OUTPUTS FROM A SET OF TRAINING VIDEOS
	
	}else if(a == 3){
	//BOF TRAINING CLASSIFICATION ANN
		int dummy=0; //should be ignored and eventually removed
		train(&dummy);
	//EOF TRAINING CLASSIFICATION ANN
	}else if(a == 4){
		//BOF USING BOTH DETECTORS AND CLASSIFIER ANN TO PROCESS GESTURES FROM VIDEO/CAMERA
		
		//get detectors from stored files
		vector<Genotype> detectors;
		for(int i=1; i<=6; i++){
			
			char id[1*sizeof(int)+1];
			sprintf(id, "_%d", i);

			char * NEW_DETECTOR_FILE_NAME = new char[std::strlen(DETECTOR_FILENAME)+std::strlen(id)+1];
		    std::strcpy(NEW_DETECTOR_FILE_NAME,DETECTOR_FILENAME);
		    std::strcat(NEW_DETECTOR_FILE_NAME,id);

			genotype_set_size(442*442);
			Genotype ann = genotype_create();
			
			char * NEW_GENOTYPE_FITNESS_FILE_NAME = new char[std::strlen(GENOTYPE_FITNESS_FILE_NAME)+std::strlen(id)+1];
		    std::strcpy(NEW_GENOTYPE_FITNESS_FILE_NAME,GENOTYPE_FITNESS_FILE_NAME);
		    std::strcat(NEW_GENOTYPE_FITNESS_FILE_NAME,id);

			//for every detector,
			read_ann(ann, NEW_DETECTOR_FILE_NAME, NEW_GENOTYPE_FITNESS_FILE_NAME, 0, 1);
			
			detectors.push_back(ann);
		}

		int type=0; //recurrent network
		while(true){ //non-stop
			
			int mode = 0; //from pre-recorded videos
			vector<Graph> string_feature_graph_demo;
			float diagonal_size = 0.0;
			Mat resized_M;

			//capture
			int frames = start_capture(mode, string_feature_graph_demo, 16/*-1*/, diagonal_size, 16, 9);  //use mode=1 and -1 for camera stream

			//make affinity matrix
			Mat M = match_strings(string_feature_graph_demo, string_feature_graph_demo, diagonal_size);

			//resize
			Size size(21,21);
			resized_M = Mat(21,21, CV_32F, cvScalar(0.));
			resize(M,resized_M,size);
			
			vector<double> detector_outputs;
			
			genotype_set_size(442*442);
			
			//feed detectors genotype and constructed affinity matrix
			type=0; //recurrent network
			for(int k=0;k<detectors.size();k++){
				double ann_output = feed(detectors[k], resized_M, type);
				detector_outputs.push_back(ann_output);
			}
			
			genotype_set_size(121);
			
			//feed classifier neural network together with detectors outputs
			type=1; //feed-forward network
			cv::Mat inputs(detector_outputs, true);
			double ann_output = feed(detectors[0], inputs, type);
		}
		//EOF USING BOTH DETECTORS AND CLASSIFIER ANN TO PROCESS GESTURES FROM VIDEO/CAMERA
	}
	
	return 0;
}