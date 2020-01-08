#include "util.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <vector>
#include <numeric>

using namespace cv;
using namespace std;

vector<string> _getFilePath(string dir) {
	vector<string> res;
	for (int i = 0; i < 9; i++)
		res.push_back(dir + "a\\" + to_string(i + 1) + ".bmp");
	return res;
}

#define SPECIAL 1
vector<string> getFilePath(string dir, string ext_str,int special = 0) {

	if (special) return _getFilePath(dir);

	vector<string> res;
	vector<String> _res;

	glob(dir + ext_str, _res, false);

	for (auto S : _res) {
		res.push_back(string(S));
		cout << S << endl;
	}

	return res;
}

cv::Size getImagePoints(vv2p_t &imagePoints) {
	cv::Size res; //返回图像的大小

	SimpleBlobDetector::Params params;
	//params.minArea = 20;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);


	vector<string> imageFilePaths = getFilePath(R"(.\data\1\)", "*.jpg");
	//imageFilePaths = getFilePath(R"(.\data\lirui\)","*.bmp");
	//vector<string> imageFilePaths = getFilePath(R"(C:\Users\cslay\Desktop\3d-dic\sample\small\sample20191223\calibrate\)", "a*.bmp");
	for (auto path : imageFilePaths)
	{
		Mat image = imread(path, IMREAD_COLOR);
		res = image.size();
		vector<Point2f> centers;


		SimpleBlobDetector::Params params;
		params.filterByArea = false;
		params.filterByColor = false;
		params.filterByConvexity = false;
		params.filterByInertia = false;

		//设置使用圆度识别，圆度0.8~1.0
		params.filterByCircularity = true;
		params.maxCircularity = 1.0;
		params.minCircularity = 0.8f;

		//设置使用面积识别
		params.filterByArea = true;
		params.maxArea = 10e3;
		params.minArea = 100;

		//blob分块最小距离
		params.minDistBetweenBlobs = 1;

		Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

		if (findCirclesGrid(image, Size(12, 9), centers, CALIB_CB_SYMMETRIC_GRID, detector)) {
			imagePoints.push_back(centers);
		}
	}

	return res;
}


int single_calibrate(int argc, char* argv[]) {

	vv2p_t imagePoints;
	vv3p_t objectPoints;
	Size imageSize;
	Mat cameraMatrix;
	Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	Mat stdIn;
	Mat stdEx;
	vector<double> perViewErrors;
	int flag = CALIB_RATIONAL_MODEL | CALIB_ZERO_TANGENT_DIST; //calib_rational_model开启k4 k5 k6
	//flag = 0; 
	//flag = CALIB_RATIONAL_MODEL;



	//
	imageSize = getImagePoints(imagePoints);
	objectPoints = get_objectPoints(cv::Size(12, 9), imagePoints.size(), 1.f);

	cout << imageSize << " 的标定图像" << endl;
	cout << imagePoints.size() << " 成功检测圆点" << endl;

	TermCriteria tc(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON);

	calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
		stdIn, stdEx, perViewErrors, flag,tc);


	cout << "cameraMatrix" << endl;
	cout << cameraMatrix << endl;
	cout << "distCoeffs" << endl;
	cout << distCoeffs.size() << endl;
	cout << distCoeffs << endl;

	vector<Point2f> project_points;
	projectPoints(objectPoints[0], rvecs[0], tvecs[0], cameraMatrix, distCoeffs, project_points);
	
	/*	double sum = 0;
	for (int i = 0; i < project_points.size(); i++) {
		Point2f d = project_points[i] - imagePoints[0][i];
		sum += norm(d)*norm(d);
	}
	cout << sqrt(sum / project_points.size()) << endl;*/

	
	double error_avg = 0;
	for (auto it = perViewErrors.begin(); it != perViewErrors.end(); it++) {
		error_avg += *it;
	}
	error_avg /= perViewErrors.size();

	cout << "平均误差:"<< error_avg  << endl;

	return 0;
}