#include "vehicles313.h"

#define USE_VIDEO 1

using namespace std;

CvCapture* input_video;
CvSize video_size;//定义全局变量
CvSize roi_size;

IplImage* frame;

CvPoint pts[8];

IplImage* grey;
IplImage* laneModel;
IplImage* roi;
IplImage* roi_sobel;
IplImage* roi_sobel_erode;
IplImage* tmp;
	
IplConvKernel* structure;
IplConvKernel* erode;


/*initial variable*/
bool initial(const char* filename){

#ifdef USE_VIDEO
	input_video = cvCreateFileCapture(filename);
#else
	input_video = cvCaptureFromCAM(0);
#endif


	if(input_video == NULL){
		fprintf(stderr,"Error:Cant open video\n");
		return false;
	}

	video_size.height = (int)cvGetCaptureProperty(input_video,CV_CAP_PROP_FRAME_HEIGHT);
	video_size.width = (int)cvGetCaptureProperty(input_video,CV_CAP_PROP_FRAME_WIDTH);

	frame = NULL;

	pts[0] = cvPoint(video_size.width/2-25-50,video_size.height*0.65);
	pts[1] = cvPoint(video_size.width/2-25-350,video_size.height*0.8);
	pts[2] = cvPoint(video_size.width/2-25-25,video_size.height*0.65);
	pts[3] = cvPoint(video_size.width/2-25-150,video_size.height*0.8);
	pts[4] = cvPoint(video_size.width/2-25+25,video_size.height*0.65);
	pts[5] = cvPoint(video_size.width/2-25+150,video_size.height*0.8);
	pts[6] = cvPoint(video_size.width/2-25+50,video_size.height*0.65);
	pts[7] = cvPoint(video_size.width/2-25+350,video_size.height*0.8);

	roi_size.height = video_size.height-pts[0].y;
	roi_size.width = video_size.width;

	grey = cvCreateImage(video_size,IPL_DEPTH_8U,1);
	laneModel = cvCreateImage(video_size,IPL_DEPTH_8U,1);

	roi = cvCreateImage(roi_size,IPL_DEPTH_8U,1);
	roi_sobel = cvCreateImage(roi_size,IPL_DEPTH_8U,1);
	roi_sobel_erode = cvCreateImage(roi_size,IPL_DEPTH_8U,1);
	tmp = cvCreateImage(roi_size,IPL_DEPTH_8U,1);
	structure = cvCreateStructuringElementEx(5,1,3,0,CV_SHAPE_RECT,0);
	erode = cvCreateStructuringElementEx(5,1,0,0,CV_SHAPE_RECT,0);

	return true;
}

/*release memory*/
void release(){

	cvReleaseImage(&grey);
	cvReleaseImage(&laneModel);
	cvReleaseImage(&roi);
	cvReleaseImage(&roi_sobel);
	cvReleaseImage(&roi_sobel_erode);
	cvReleaseImage(&tmp);
	

	cvReleaseStructuringElement(&structure);
	cvReleaseStructuringElement(&erode);

	cvReleaseCapture(&input_video);
}


/*algorithm*/
int algorithm(const char* filename){
	
/*#ifdef USE_VIDEO
	CvCapture* input_video = cvCreateFileCapture("F:\\kwong\\videoForTest\\01510022.mov");
#else
	CvCapture* input_video = cvCaptureFromCAM(0);
#endif

	//debug
	char name[50];
	ofstream file("F:\\kwong\\data+time.txt",ios_base::app);
	
	if(input_video == NULL){
		fprintf(stderr,"Error:Cant open video\n");
		return -1;
	}

	double fps = (double)cvGetCaptureProperty(input_video,CV_CAP_PROP_FPS);
	CvVideoWriter* writer = cvCreateVideoWriter("G:\\kwong\\gain.avi",CV_FOURCC('D','I','V','X'),fps,video_size,1);

	std::vector<int> l1;
	std::vector<int> l2;
	std::vector<int> l3;
	std::vector<int> l4;
	int drift;
	vehicle location[3];
	vehicleBox boxes[3];
	clock_t tb;
	clock_t te;
	double t =0;

	IplImage* grey = cvCreateImage(video_size,IPL_DEPTH_8U,1);
	IplImage* laneModel = cvCreateImage(video_size,IPL_DEPTH_8U,1);

	CvSize roi_size;
	roi_size.height = video_size.height-pts[0].y;
	roi_size.width = video_size.width;

	IplImage* roi = cvCreateImage(roi_size,IPL_DEPTH_8U,1);
	IplImage* roi_sobel = cvCreateImage(roi_size,IPL_DEPTH_8U,1);
	IplImage* roi_sobel_erode = cvCreateImage(roi_size,IPL_DEPTH_8U,1);
	IplImage* tmp = cvCreateImage(roi_size,IPL_DEPTH_8U,1);
	
	IplConvKernel* structure = cvCreateStructuringElementEx(5,1,3,0,CV_SHAPE_RECT,0);
	IplConvKernel* erode = cvCreateStructuringElementEx(5,1,0,0,CV_SHAPE_RECT,0);*/

	clock_t start;
	clock_t end;
	double mspf = 0;

	std::vector<int> l1;
	std::vector<int> l2;
	std::vector<int> l3;
	std::vector<int> l4;
	int drift;
	vehicle location[3];
	vehicleBox boxes[3];

	long current_frame = 0;
	int key_pressed = 0;

	if(!initial(filename)){
		fprintf(stderr,"Error:Initialize Failed!!!\n");
		return -1;
	}


	laneBorder(video_size,
			pts[0],pts[1],pts[2],pts[3],pts[4],pts[5],pts[6],pts[7],
			l1,l2,l3,l4,drift);

	while(key_pressed != 27){
		
		start = clock();

		frame = cvQueryFrame(input_video);

		if(frame == NULL){
			fprintf(stderr,"Error: null frame received\n");
			printf("***********************\n");
			printf("video size:\t%d*%d\n",video_size.width,video_size.height);
			printf("ms per frame:\t%f\n",mspf/current_frame);
			printf("fps:\t\t%f\n",1000/(mspf/current_frame));
			printf("***********************\n");
			system("pause");
			release();
			return -1;
		}

		current_frame++;

		cvCvtColor(frame,grey,CV_RGB2GRAY);

		Lane(grey,laneModel,tmp,pts[0],pts[1],pts[2],pts[3],pts[4],pts[5],pts[6],pts[7]);
		crop(grey,roi,cvRect(0,pts[0].y,roi->width,roi->height));

		threshold(roi,roi,shadowBound(roi,laneModel),255);

		//cvDilate(roi,roi,0,1);

		sobel(roi,roi_sobel);

		cvErode(roi_sobel,roi_sobel_erode,erode,1);

		cvMorphologyEx(roi_sobel,roi_sobel,0,structure,CV_MOP_CLOSE,1);
		
		vehiclesLocation(roi_sobel,roi_sobel_erode,l1,l2,l3,l4,location,drift);

		genVehicleBoxes(boxes,location,l1,l2,l3,l4,drift);

		verifyBoxes(grey,boxes);
	
		drawVehicles(frame,boxes);

		end = clock();

		mspf += (double)(end-start);

		/*if(!strcmp(filename,"F:\\kwong\\videofortest\\test1.mov")){

			sprintf(name,"F:\\kwong\\test1\\frame%05d.jpg",current_frame);
			cvSaveImage(name,frame);
		}
		if(!strcmp(filename,"F:\\kwong\\videofortest\\test2.mov")){
		
			sprintf(name,"F:\\kwong\\test2\\frame%05d.jpg",current_frame);
			cvSaveImage(name,frame);
		}*/

		cvNamedWindow("vehicles");
		cvShowImage("vehicles",frame);
		cvNamedWindow("roi");
		cvShowImage("roi",tmp);

		/*sprintf(name,"F:\\kwong\\center\\center%05d.jpg",current_frame);
		cvSetImageROI(frame,cvRect(boxes[1].bmin.x,boxes[1].bmin.y,boxes[1].bmax.x-boxes[1].bmin.x,boxes[1].bmax.y-boxes[1].bmin.y));
		cvSaveImage(name,frame);
		cvResetImageROI(frame);*/

		/*sprintf(name,"F:\\kwong\\left\\left%05d.jpg",current_frame);
		cvSetImageROI(frame,cvRect(boxes[0].bmin.x,boxes[0].bmin.y,boxes[0].bmax.x-boxes[0].bmin.x,boxes[0].bmax.y-boxes[0].bmin.y));
		cvSaveImage(name,frame);
		cvResetImageROI(frame);*/

		/*sprintf(name,"F:\\kwong\\right\\right%05d.jpg",current_frame);
		cvSetImageROI(frame,cvRect(boxes[2].bmin.x,boxes[2].bmin.y,boxes[2].bmax.x-boxes[2].bmin.x,boxes[2].bmax.y-boxes[2].bmin.y));
		cvSaveImage(name,frame);
		cvResetImageROI(frame);*/


		/*printf("current_frame:%d\n",current_frame);
		cvWriteFrame(writer,frame);*/

		key_pressed = cvWaitKey(15);

		/*printf("current = %d\n",current_frame);*/
		//system("pause");
	}
	
	/*//cvReleaseImage(&frame);
	cvReleaseImage(&grey);
	cvReleaseImage(&laneModel);
	cvReleaseImage(&roi);
	cvReleaseImage(&roi_sobel);
	cvReleaseImage(&roi_sobel_erode);
	cvReleaseImage(&tmp);
	

	cvReleaseStructuringElement(&structure);
	cvReleaseStructuringElement(&erode);

	cvReleaseCapture(&input_video);
	cvReleaseVideoWriter(&writer);*/
	return 1;
}