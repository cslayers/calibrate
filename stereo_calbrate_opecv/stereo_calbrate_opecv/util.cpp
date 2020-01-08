#include "util.h"


using namespace std;
using namespace cv;

vv3p_t get_objectPoints(cv::Size patternSize, size_t num_view, float grid) {
	//获得三维坐标
	//每组图都要一份，都相同
	//这是opencv接口决定的
	//grid 是三维坐标的间隔
	//num_view是每相机的图片数，取决于检测圆点成功与否

	v3p_t coors;
	for (int i = 0; i < patternSize.height; i++)
		for (int j = 0; j < patternSize.width; j++)
		{
			Point3f p((float)j *grid, (float)i*grid, 0.f);// z==0
			coors.push_back(p);
		}

	vv3p_t objectPoints(num_view, coors);
	return objectPoints;
}


cv::Size get_imagePoints(vector<string>paths, vector<bool>& flags, vv2p_t &imagePoints, cv::Size patternSize) {
	cv::Size res;
	for (int i = 0; i < paths.size(); i++)
	{
		string path = paths[i];
		Mat image = imread(path, IMREAD_COLOR);
		res = image.size();
		vector<Point2f> centers;
		if (findCirclesGrid(image, patternSize, centers, CALIB_CB_SYMMETRIC_GRID, SimpleBlobDetector::create(get_blob_params())))
			imagePoints.push_back(centers);
		else
			flags[i] = false;
	}
	return res;
}



SimpleBlobDetector::Params get_blob_params() {

	//调整这些参数，它们会影响重投影误差rms

	SimpleBlobDetector::Params params;
	params.filterByArea = false;
	params.filterByColor = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;

	//设置使用圆度识别，圆度0.8~1.0
	params.filterByCircularity = true;
	params.maxCircularity = 1.0f;
	params.minCircularity = 0.8f;

	//设置使用面积识别
	params.filterByArea = true;
	params.maxArea = 10e3;
	params.minArea = 100.f;

	//blob分块最小距离
	params.minDistBetweenBlobs = 1;
	return params;
}