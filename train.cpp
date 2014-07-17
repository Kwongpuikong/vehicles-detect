#include "vehicles.h"

using namespace cv;
using namespace std;

void svmTrain(const char* pos,const char* neg){

	int ImgWidth = 64;
	int ImgHeight = 64;
	vector<string> img_path;
	vector<int> img_catg;
	int nline = 0;
	string buf;
	ifstream svm_data_pos(pos);
	ifstream svm_data_neg(neg);
	unsigned long n;

	while(svm_data_pos){
	
		getline(svm_data_pos,buf);
		img_path.push_back(buf);
		img_catg.push_back(1);
	}

	svm_data_pos.close();

	while(svm_data_neg){
	
		getline(svm_data_neg,buf);
		img_path.push_back(buf);
		img_catg.push_back(0);
	}
	svm_data_neg.close();

	Mat data_mat,res_mat;
	int nImgNum = img_catg.size();

	res_mat = Mat::zeros(nImgNum,1,CV_32FC1);

	Mat src;
	Mat trainImg = Mat::zeros(ImgHeight, ImgWidth, CV_8UC3);
	HOGDescriptor hog=HOGDescriptor(cvSize(ImgWidth,ImgHeight),cvSize(16,16),cvSize(8,8),cvSize(8,8), 9);
	vector<float> descriptors(1764,0);

	for(string::size_type i=0;i!=img_path.size();i++){
	
		src = imread(img_path[i].c_str(),1);
		cout<<"processing "<<img_path[i].c_str()<<endl;
		resize(src, trainImg, cv::Size(ImgWidth,ImgHeight), 0, 0, INTER_CUBIC);
		hog.compute(trainImg,descriptors,Size(1,1),Size(0,0));
		if(i==0){
		
			data_mat = Mat::zeros(nImgNum,descriptors.size(),CV_32FC1);
		}
		//cout<<"HOG dims: "<<descriptors.size()<<endl;
		n=0;
		for(vector<float>::iterator iter=descriptors.begin();iter!=descriptors.end();iter++){
		
			data_mat.at<float>(i,n)=*iter;
			n++;
		}
		res_mat.at<float>(i,0) = img_catg[i];
		cout<<"end processing "<<img_path[i].c_str()<<" "<<img_catg[i]<<endl;
	}

	system("pause");
	
	CvSVM svm = CvSVM();  
    CvSVMParams param;    
    CvTermCriteria criteria;      
    criteria = cvTermCriteria( CV_TERMCRIT_EPS, 1000, FLT_EPSILON );      
    param = CvSVMParams( CvSVM::C_SVC, CvSVM::RBF, 10.0, 0.09, 1.0, 10.0, 0.5, 1.0, NULL, criteria ); 
	cout<<"start svm training..."<<endl;
	svm.train( data_mat, res_mat, Mat(),Mat(),param ); 
	svm.save( "SVM_DATA.xml" ); 
	cout<<"training end!"<<endl;

}
