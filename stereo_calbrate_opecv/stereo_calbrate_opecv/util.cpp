#include "util.h"


using namespace std;
using namespace cv;

vv3p_t get_objectPoints(cv::Size patternSize, size_t num_view, float grid) {
	//�����ά����
	//ÿ��ͼ��Ҫһ�ݣ�����ͬ
	//����opencv�ӿھ�����
	//grid ����ά����ļ��
	//num_view��ÿ�����ͼƬ����ȡ���ڼ��Բ��ɹ����

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

	//������Щ���������ǻ�Ӱ����ͶӰ���rms

	SimpleBlobDetector::Params params;
	params.filterByArea = false;
	params.filterByColor = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;

	//����ʹ��Բ��ʶ��Բ��0.8~1.0
	params.filterByCircularity = true;
	params.maxCircularity = 1.0f;
	params.minCircularity = 0.8f;

	//����ʹ�����ʶ��
	params.filterByArea = true;
	params.maxArea = 10e3;
	params.minArea = 100.f;

	//blob�ֿ���С����
	params.minDistBetweenBlobs = 1;
	return params;
}