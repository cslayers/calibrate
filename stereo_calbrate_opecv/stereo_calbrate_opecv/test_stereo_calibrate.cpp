#include <iostream>
using namespace std;
#include <vector>



#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "cmd.h"
#include "util.h"
using namespace cv;

class Setting {
public:
	size_t num_views = 10; //几组图
	size_t num_views_sucess = 10; //成功检测圆点几组图
	float grid = 1.f; //三维坐标间隔
	Size patternSize = Size(12, 9); //圆点个数
	Size imageSize;
}setting;







int get_imagePoints(vv2p_t& imagePoints1, vv2p_t& imagePoints2) {
	string image_dir = "./data/";
	string camera_1_dir = "1/"; //子文件夹名字是相机的编号
	string camera_2_dir = "2/";

	vector<string> filenames(setting.num_views); //文件名
	for (int i = 0; i < filenames.size(); i++) {
		filenames[i] = to_string(i) + ".jpg"; //从1开始
	}

	for (int i = 0; i < setting.num_views; i++) {
		string filename = filenames[i];
		string full_filepath_1 = image_dir + camera_1_dir + filename;
		string full_filepath_2 = image_dir + camera_2_dir + filename;
		Mat view_1 = imread(full_filepath_1, IMREAD_COLOR);
		Mat view_2 = imread(full_filepath_2, IMREAD_COLOR);

		v2p_t coors_1;
		v2p_t coors_2;
		coors_1.reserve(setting.patternSize.width * setting.patternSize.height);
		coors_2.reserve(setting.patternSize.width * setting.patternSize.height);
		int found_1 = findCirclesGrid(view_1, setting.patternSize, coors_1);
		int found_2 = findCirclesGrid(view_2, setting.patternSize, coors_2);
		//cout << found_1 << " " << found_2 << "    " << coors_1.size() << " " << coors_2.size() << endl;

		if (found_1 & found_2) {
			imagePoints1.push_back(coors_1);
			imagePoints2.push_back(coors_2);
		}

		setting.imageSize = view_1.size();
	}

	setting.num_views_sucess = imagePoints1.size();
	return 0;
}


int stereo_calibrate(int argc, char* argv[]) {
	//TODO 处理传入的参数



	vv2p_t imagePoints1, imagePoints2;
	get_imagePoints(imagePoints1, imagePoints2);

	vv3p_t objectPoints = get_objectPoints(setting.patternSize,setting.num_views_sucess,setting.grid);

	
	Mat K1, D1, K2, D2, R,T,E,F,perViewError;

	//TODO 畸变模型设置未了解
	D1 = Mat::zeros(Size(1, 8), CV_32F);
	D2 = Mat::zeros(Size(1, 8), CV_32F);

	double res = stereoCalibrate(objectPoints, imagePoints1, imagePoints2, K1, D1, K2, D2, 
		setting.imageSize, R, T, E, F, perViewError, 0);
	
	cout << "Return value: " << res << endl;

	cout << "size" << endl;
	cout << K1.size() << endl;
	cout << D1.size() << endl;
	cout << R.size() << endl;
	cout << T.size() << endl;
	cout << E.size() << endl;
	cout << F.size() << endl;
	cout << perViewError.size() << endl;
	cout << perViewError << endl;


	
	Mat rv1;
	Mat tv1 = Mat::zeros(1, 3, CV_32F);
	Mat RR = Mat::eye(3,3,CV_32F);
	Rodrigues(RR, rv1);
	Mat out;
	projectPoints(objectPoints[0], rv1, tv1, K1, D1, out);




	assert(imagePoints1.size() == imagePoints2.size());
	assert(objectPoints.size() == imagePoints1.size());
	assert(objectPoints.size() > 0);
	assert(objectPoints[0].size() == setting.patternSize.height * setting.patternSize.width);
	/*for (auto p : objectPoints[0])
		cout << p << endl;*/







	getchar();
	return 0;
}