#include "vehicles.h"


/*crop iamge*/
void crop(IplImage* src,IplImage* dest,CvRect rect){
	cvSetImageROI(src,rect);
	cvCopy(src,dest);
	cvResetImageROI(src);
}

/*thresh*/
void threshold(IplImage* src,IplImage* dest,double threshold,unsigned char max_value){

	for(int r=0;r<src->height;r++){
		unsigned char* sptr = (unsigned char*)src->imageData+r*src->widthStep;
		unsigned char* dptr = (unsigned char*)dest->imageData+r*dest->widthStep;
		for(int c=0;c<src->width;c++){
		
			if(sptr[c]>threshold)
				dptr[c] = max_value;
			else
				dptr[c] = sptr[c];
		}
	}
	
}


/*Lane Model*/
void Lane(IplImage* grey,IplImage* laneModel,IplImage* roi,
	CvPoint p1,CvPoint p2,CvPoint p3,CvPoint p4,CvPoint p5,CvPoint p6,CvPoint p7,CvPoint p8,
	int LCR){

	double k[4];
	k[0] = (p1.x-p2.x)/(p1.y-p2.y);
	k[1] = (p3.x-p4.x)/(p3.y-p4.y);
	k[2] = (p5.x-p6.x)/(p5.y-p6.y);
	k[3] = (p7.x-p8.x)/(p7.y-p8.y);

	int x[4];
	x[0] = k[0]*(laneModel->height-p1.y)+p1.x;
	x[1] = k[1]*(laneModel->height-p3.y)+p3.x;
	x[2] = k[2]*(laneModel->height-p5.y)+p5.x;
	x[3] = k[3]*(laneModel->height-p7.y)+p7.x;

	/*cvLine(frame,p1,cvPoint(x[0],frame->height),CV_RGB(255,0,0),5,8,0);
	cvLine(frame,p3,cvPoint(x[1],frame->height),CV_RGB(255,0,0),5,8,0);
	cvLine(frame,p5,cvPoint(x[2],frame->height),CV_RGB(255,0,0),5,8,0);
	cvLine(frame,p7,cvPoint(x[3],frame->height),CV_RGB(255,0,0),5,8,0);*/

	int npts[1] = {4};
	CvPoint** pts = new CvPoint*[1];
	pts[0] = new CvPoint[4];
	CvPoint pt[4];
	cvSetZero(laneModel);

	switch(LCR){

	case -1:
		/*CvPoint pt[4] = {p1,cvPoint(x[0],laneModel->height),cvPoint(x[1],laneModel->height),p3};*/
		pt[0] = p1;
		pt[1] = cvPoint(x[0],laneModel->height);
		pt[2] = cvPoint(x[1],laneModel->height);
		pt[3] = p3;
		break;

	case 0:
		/*CvPoint pt[4] = {p3,cvPoint(x[1],laneModel->height),cvPoint(x[2],laneModel->height),p5};*/
		pt[0] = p3;
		pt[1] = cvPoint(x[1],laneModel->height);
		pt[2] = cvPoint(x[2],laneModel->height);
		pt[3] = p5;
		break;

	case 1:
		/*CvPoint pt[4] = {p5,cvPoint(x[2],laneModel->height),cvPoint(x[3],laneModel->height),p7};*/
		pt[0] = p5;
		pt[1] = cvPoint(x[2],laneModel->height);
		pt[2] = cvPoint(x[3],laneModel->height);
		pt[3] = p7;
		break;

	case 2:
		/*CvPoint pt[4] = {p1,cvPoint(x[0],laneModel->height),cvPoint(x[3],laneModel->height),p7};*/
		pt[0] = p1;
		pt[1] = cvPoint(x[0],laneModel->height);
		pt[2] = cvPoint(x[3],laneModel->height);
		pt[3] = p7;
		break;
	}
	pts[0][0] = pt[0];
	pts[0][1] = pt[1];
	pts[0][2] = pt[2];
	pts[0][3] = pt[3];

	cvFillPoly(laneModel,pts,npts,1,CV_RGB(255,255,255));

	IplImage* temp1 = cvCreateImage(cvGetSize(roi),IPL_DEPTH_8U,1);
	IplImage* temp2 = cvCreateImage(cvGetSize(roi),IPL_DEPTH_8U,1);
	crop(grey,temp1,cvRect(0,p1.y,roi->width,roi->height));
	crop(laneModel,temp2,cvRect(0,p1.y,roi->width,roi->height));
	cvAnd(temp1,temp2,roi,0);
	/*crop(temp,roi,cvRect(0,p1.y,roi->width,roi->height));*/
	cvReleaseImage(&temp1);
	cvReleaseImage(&temp2);
}

/*border of lane*/
void laneBorder(CvSize video_size,
				CvPoint p1,CvPoint p2,CvPoint p3,CvPoint p4,CvPoint p5,CvPoint p6,CvPoint p7,CvPoint p8,
				std::vector<int> &l1,std::vector<int> &l2,std::vector<int> &l3,std::vector<int> &l4,
				int& drift){

	drift = p1.y;

	float k[4];
	k[0] = (p1.x-p2.x)/(p1.y-p2.y);
	k[1] = (p3.x-p4.x)/(p3.y-p4.y);
	k[2] = (p5.x-p6.x)/(p5.y-p6.y);
	k[3] = (p7.x-p8.x)/(p7.y-p8.y);

	for(int i=p1.y;i<video_size.height;i++){
		
		int tmp = k[0]*(i-p1.y)+p1.x;
		tmp = (tmp>0)?tmp:0;
		l1.push_back(tmp);

		l2.push_back(k[1]*(i-p3.y)+p3.x);
		l3.push_back(k[2]*(i-p5.y)+p5.x);

		tmp = k[3]*(i-p7.y)+p7.x;
		tmp = (tmp<video_size.width)?tmp:video_size.width;
		l4.push_back(tmp);
	}
}

/*shadow upper bound*/
double shadowBound(IplImage* roi,IplImage* laneModel){

	int h = laneModel->height-roi->height;
	IplImage* mask = cvCreateImage(cvGetSize(roi),IPL_DEPTH_8U,1);
	crop(laneModel,mask,cvRect(0,h,roi->width,roi->height));

	CvScalar mean;
	CvScalar dev;
	cvAvgSdv(roi,&mean,&dev,mask);
	cvReleaseImage(&mask);

	//printf("mean:%f\tdev:%f\nt:%f\n\n",mean.val[0],dev.val[0],mean.val[0]-dev.val[0]*dev.val[0]/mean.val[0]);
	//printf("t:%f\n",mean.val[0]-dev.val[0]);

	return (mean.val[0]-dev.val[0]);
}


/*Sobel*/
void sobel(IplImage* roi,IplImage* roi_sobel){

	IplImage* temp = cvCreateImage(cvGetSize(roi),IPL_DEPTH_16S,1);
	cvSobel(roi,temp,0,1,3);
	cvConvertScaleAbs(temp,roi_sobel,1,0);
	cvThreshold(roi_sobel,roi_sobel,0,255,CV_THRESH_OTSU);
	cvReleaseImage(&temp);

	//cvCanny(roi,roi_sobel,20,150,3);
}

/*continuous point*/
int getCb(IplImage* roi_sobel_erode,int row,int cb,int ce){

	unsigned char* ptr = (unsigned char*)roi_sobel_erode->imageData+row*roi_sobel_erode->widthStep;

	for(int c=cb;c<ce;c++){
	
		if(ptr[c]>250)
			return c;
	}

	return -1;
}


/*rate of white points*/
float whitePointsRate(IplImage* roi_sobel,IplImage* roi_sobel_erode,int row,int cb,int ce,int* vb,int* ve){

	/*int cnt = 0;
	int gap = (ce-cb)/10;
	unsigned char* ptr = (unsigned char *)(roi_sobel->imageData+row*roi_sobel->widthStep);
	
	for(int c=cb;c<ce;c++){

		if(ptr[c]>250){
		
			cnt++;
			if(cnt == 1)
				*vb = c;
			gap = (ce-cb)/10;
		}
		else if(cnt>0)
			gap--;
		else if(cnt==0 && c==ce/2)
			break;

		if(gap<=0 && ptr[c]<250){

			*ve = c;
			break;
		}
	}
	float rate = cnt/(float)(ce-cb);*/

	//printf("cb = %d\nce = %d\n",cb,ce);
	//printf("cnt = %d\nrate = %f\n",cnt,rate);
	/*std::ofstream outfile("rate-1.txt",std::ios_base::app);
	outfile<<"cb:"<<cb<<"\t"<<"ce:"<<ce<<"\t"<<"cnt:"<<cnt<<"\t"<<"rate:"<<rate<<"\n";*/

	unsigned char* ptr = (unsigned char*)roi_sobel->imageData+row*roi_sobel->widthStep;

	int cbegin = getCb(roi_sobel_erode,row,cb,ce);

	if(cbegin<0){
	
		//printf("cbegin:0\n");
		return 0;
	}

	int cnt = 0;
	int gap = 5;
	float rate;
	for(int c=cbegin;c<ce;c++){
	
		if(ptr[c]>250){

			cnt++;
			gap = 5;
		}
		else if(cnt>0 && gap>0){
			gap--;
		}
	}
	rate = cnt/(float)(ce-cb);

	//printf("cb:%d\tce:%d\tcbegin:%d\tptr[cbegin]:%d\tcnt:%d\trate:%f\n",cb,ce,cbegin,ptr[cbegin],cnt,rate);

	return rate;
}

/*vehicles location*/
void vehiclesLocation(IplImage* roi_sobel,IplImage* roi_sobel_erode,
	std::vector<int> l1,std::vector<int> l2,std::vector<int> l3,std::vector<int> l4,
	vehicle location[3],int drift){

		/*std::ofstream outleft("rate-1.txt",std::ios_base::app);
		std::ofstream outcenter("rate0.txt",std::ios_base::app);
		std::ofstream outright("rate1.txt",std::ios_base::app);
		static int n = 0;
		n++;*/

		location[0].r = location[1].r = location[2].r = -1;
		for(int r=0.5*roi_sobel->height;r>=0;r--){

			if(location[0].r==-1){

				float rate = whitePointsRate(roi_sobel,roi_sobel_erode,r,l1[r],l2[r]);
				if(rate>=0.3){
			
					location[0].r = r+drift;
					//outleft<<"n:"<<n<<"\trate:"<<rate<<"\tr:"<<r<<std::endl;
				}
			}
			if(location[1].r==-1){

				float rate = whitePointsRate(roi_sobel,roi_sobel_erode,r,l2[r],l3[r]);
				if(rate>=0.3){

					location[1].r = r+drift;
					//outcenter<<"n:"<<n<<"\trate:"<<rate<<"\tr:"<<r<<std::endl;
				}
			}
			if(location[2].r==-1){

				float rate = whitePointsRate(roi_sobel,roi_sobel_erode,r,l3[r],l4[r]);
				if(rate>=0.3){
				
					location[2].r = r+drift;
					//outright<<"n:"<<n<<"\trate:"<<rate<<"\tr:"<<r<<std::endl;
				}
			}
		}
}

/*generate vehicle boxes*/
void genVehicleBoxes(vehicleBox boxes[],vehicle location[],
	std::vector<int> l1,std::vector<int> l2,std::vector<int> l3,std::vector<int> l4,
	int drift){

		if(location[0].r>0){
			boxes[0].valid = true;
			int r0 = location[0].r-drift;
			boxes[0].bmin = cvPoint(l1[r0],location[0].r-0.5*(l2[r0]-l1[r0]));
			boxes[0].bmax = cvPoint(l2[r0],location[0].r);
			boxes[0].width = boxes[0].bmax.x-boxes[0].bmin.x;
			boxes[0].height = boxes[0].bmax.y-boxes[0].bmin.y;
		}
		else
			boxes[0].valid = false;

		if(location[1].r>0){
			boxes[1].valid = true;
			int r1 = location[1].r-drift;
			boxes[1].bmin = cvPoint(l2[r1],location[1].r-0.5*(l3[r1]-l2[r1]));
			boxes[1].bmax = cvPoint(l3[r1],location[1].r);
			boxes[1].width = boxes[1].bmax.x-boxes[1].bmin.x;
			boxes[1].height = boxes[1].bmax.y-boxes[1].bmin.y;
		}
		else
			boxes[1].valid = false;

		if(location[2].r>0){
			boxes[2].valid = true;
			int r2 = location[2].r-drift;
			boxes[2].bmin = cvPoint(l3[r2],location[2].r-0.5*(l4[r2]-l3[r2]));
			boxes[2].bmax = cvPoint(l4[r2],location[2].r);
			boxes[2].width = boxes[2].bmax.x-boxes[2].bmin.x;
			boxes[2].height = boxes[2].bmax.y-boxes[2].bmin.y;
		}
		else
			boxes[2].valid = false;


}

/*calculate entropy*/
double Entropy(IplImage* grey,CvRect rect,CvHistogram* hist){

	double H = 0;

	cvSetImageROI(grey,rect);

	IplImage* image[] = {grey};

	cvClearHist(hist);
	cvCalcHist(image,hist,0,0);
	cvNormalizeHist(hist,1.0);

	double p;
	for(int i=0;i<256;i++){
	
		p = cvQueryHistValue_1D(hist,i);
		if(!p)
			continue;
		H += p*log(1/p);
	}
	cvResetImageROI(grey);

	return (pow(H,2));
}


/*verify vehicle box*/
void verifyBoxes(IplImage* grey,vehicleBox boxes[]){
	
	double H;
	unsigned int size;
	/*construct histogram*/
	int dims = 1;
	int sizes[] = {256};
	int type = CV_HIST_ARRAY;
	float range[] = {0,255};
	float* ranges[] = {range};

	CvHistogram* hist = cvCreateHist(dims,sizes,type,ranges,1);

	for(int i=0;i<3;i++){

		////step 1:exclude the small boxes...
		//size = boxes[i].width*boxes[i].height;
		//if(size<1000)
		//	boxes[i].valid = false;
	
		//step 2:exclude boxes which entropy is low...
		if(boxes[i].valid){
			H = Entropy(grey,cvRect(boxes[i].bmin.x,boxes[i].bmin.y,boxes[i].width,boxes[i].height),hist);

			if(H<21)	//21 is threshold which should be calculated
				boxes[i].valid = false;
		}
		else
			H = 0;

		/*if(!file)
				cerr<<"oops!\n";
			else
				file<<H<<"\t";*/

	}
	//file<<"\n";

	cvReleaseHist(&hist);

}
/*draw vehicles*/
void drawVehicles(IplImage* frame,vehicleBox boxes[]){

		for(int i=0;i<3;i++){
		
			if(boxes[i].valid)
				cvRectangle(frame,boxes[i].bmin,boxes[i].bmax,CV_RGB(255,0,0),1,8,0);
		}

		/*printf("location:%d\tvb:%d\tve:%d\n",location[1].r,location[1].vb,location[1].ve);*/
}


