#pragma once
#include <vector>
#include <string>
using std::string;
using std::vector;
#include "util.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using cv::Mat;
using cv::Point2f;
using cv::Point3f;
using cv::Size;

namespace calib {

	struct CameraParameters {
		int single_flag = cv::CALIB_RATIONAL_MODEL;// | cv::CALIB_ZERO_TANGENT_DIST;
		Mat rvecs;
		Mat tvecs;
		Mat K;
		Mat D;
		Size imageSize;
		vector<string> filepaths; //���е��ļ���ȫ·��
		vector<bool> detected; //��Ӧ��ͼƬ�Ƿ�ɹ�
		vv2p_t imagePoints; //ֻ�Ƿųɹ���
		vv3p_t objectPoints;//��һ���С��imagepointsһ��
	};


	struct filename_comparator //�����ļ������򣬷���10���ܻ�����2ǰ��
	{
		bool operator()(string a, string b) {
			if (a.size() == b.size()) return a < b; //һ�����Ͱ�string����
			else
				return a.size() < b.size(); //�̵Ŀ�ǰ
		}
	};


	struct StereoCalibrateParams{
		float grid; //�궨��Բ����
		cv::Size patternSize;
	};


	class StereoCalibrate
	{
	public:
		static StereoCalibrateParams createStereoCalibrateParams() {
			StereoCalibrateParams res;
			res.grid = 1;
			res.patternSize = cv::Size(12,9);
			return res;
		}

	public:
		StereoCalibrate(string dir, string ext, StereoCalibrateParams params); //dirʱĿ¼��ext��ͼƬ�ļ���׺
		void start();

		~StereoCalibrate();


	private:
		int write_calibration_file();

	private:
		string _dir;
		string _ext;
		string _sub_dir_1;
		string _sub_dir_2;
		CameraParameters _cps1, _cps2;

		StereoCalibrateParams _params;//���ñ궨�Ĳ���
		Mat _R, _T, _E, _F, _perViewError; //����ƥ����
	};

	static const string path_sep = "\\";
	static const string ele_sep = ",";
}
