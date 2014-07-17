#include "vehicles.h"

using namespace cv;

void print(){

	printf("Usage:  project -[parameter]\n");
	printf("-[filename] : path of file to be calculate...\n");
}

int main(int argc,char* argv[]){

	/*char* file;

	if(argc!=2){
		print();
	}
	else{
	
		file = argv[1]+1;
	}*/
	
	//svmTrain("F:\\hog-svm-pos\\pos.txt","F:\\hog-svm-neg\\neg.txt");

	//algorithm("F:\\kwong\\videofortest\\01510022.mov");

	CvSVM svm = CvSVM();  
	CvSVMParams param;    
	CvTermCriteria criteria;      
	criteria = cvTermCriteria( CV_TERMCRIT_EPS, 1000, FLT_EPSILON );      
	param = CvSVMParams( CvSVM::C_SVC, CvSVM::RBF, 10.0, 0.09, 1.0, 10.0, 0.5, 1.0, NULL, criteria );
	svm.load("SVM_DATA.xml");

	HOGDescriptor *hog=new HOGDescriptor(cvSize(64,64),cvSize(16,16),cvSize(8,8),cvSize(8,8),9);
	vector<float> descriptors(1764,0);
	Mat feather = Mat::zeros(1,1764,CV_32FC1);

	IplImage* temp = cvCreateImage(cvSize(64,64),IPL_DEPTH_8U,3);
	IplImage* img = cvLoadImage("1.jpg");
	
	cvResize(img,temp);
	hog->compute(temp,descriptors,Size(1,1),Size(0,0));
	int n=0;    
	int flag = -1;
    for(vector<float>::iterator iter=descriptors.begin();iter!=descriptors.end();iter++){    
            
		feather.at<float>(0,n) = *iter;    
        n++;    
    }  
	flag = svm.predict(feather);
	printf("flag = %d\n",flag);
	system("pause");

	return 1;
}