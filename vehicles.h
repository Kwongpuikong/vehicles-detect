#ifndef VEHICLES_H
#define VEHICLES_H

#include <cv.h>
#include <highgui.h>
#include <vector>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <ml.h>
#include <string>

//定义结构体
struct vehicle{
	int r;
	int vb;
	int ve;
};

struct vehicleBox{
	bool valid;
	CvPoint bmin;
	CvPoint bmax;
	unsigned int width;
	unsigned int height;
};

//子函数
/*initialize*/
bool initial(const char* filename);

/*release memory*/
void release();

/*crop iamge*/
void crop(IplImage* src,IplImage* dest,CvRect rect);

/*thresh*/
void threshold(IplImage* src,IplImage* dest,double threshold,unsigned char max_value);

/*Lane Model*/
void Lane(IplImage* grey,IplImage* laneModel,IplImage* roi,
	CvPoint p1,CvPoint p2,CvPoint p3,CvPoint p4,CvPoint p5,CvPoint p6,CvPoint p7,CvPoint p8,
	int LCR = 2);

/*border of lane*/
void laneBorder(CvSize video_size,
				CvPoint p1,CvPoint p2,CvPoint p3,CvPoint p4,CvPoint p5,CvPoint p6,CvPoint p7,CvPoint p8,
				std::vector<int> &l1,std::vector<int> &l2,std::vector<int> &l3,std::vector<int> &l4,
				int& drift);

/*shadow upper bound*/
//double shadowBound(IplImage* roi,IplImage* laneModel);
double shadowBound(IplImage* roi);

/*Sobel*/
void sobel(IplImage* roi,IplImage* roi_sobel);

/*continuous point*/
int getCb(IplImage* roi_sobel_erode,int row,int cb,int ce);

/*rate of white points*/
float whitePointsRate(IplImage* roi_sobel,IplImage* roi_sobel_erode,int row,int cb,int ce,int* vb=0,int* ve=0);

/*vehicles location*/
void vehiclesLocation(IplImage* roi_sobel,IplImage* roi_sobel_erode,
	std::vector<int> l1,std::vector<int> l2,std::vector<int> l3,std::vector<int> l4,
	vehicle location[3],int drift);

/*generate vehicle boxes*/
void genVehicleBoxes(vehicleBox boxes[],vehicle location[],
	std::vector<int> l1,std::vector<int> l2,std::vector<int> l3,std::vector<int> l4,
	int drift);

/*calculate entropy*/
double Entropy(IplImage* grey,CvRect rect,CvHistogram* hist);

/*verify vehicle box*/
void verifyBoxes(IplImage* grey,vehicleBox boxes[]);

/*classify box*/
void classifyBoxes(IplImage* grey,vehicleBox boxes[]);

/*draw vehicles*/
void drawVehicles(IplImage* frame,vehicleBox boxes[]);

/*algorithm*/
int algorithm(const char* filename);

/*svm training*/
void svmTrain(const char* pos,const char* neg);


#endif // VEHICLES_H