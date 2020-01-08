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
		vector<string> filepaths; //所有的文件完全路径
		vector<bool> detected; //对应的图片是否成功
		vv2p_t imagePoints; //只是放成功的
		vv3p_t objectPoints;//第一层大小和imagepoints一样
	};


	struct filename_comparator //用于文件名排序，否则10可能会排在2前面
	{
		bool operator()(string a, string b) {
			if (a.size() == b.size()) return a < b; //一样长就按string排序
			else
				return a.size() < b.size(); //短的靠前
		}
	};


	struct StereoCalibrateParams{
		float grid; //标定板圆点间隔
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
		StereoCalibrate(string dir, string ext, StereoCalibrateParams params); //dir时目录，ext是图片文件后缀
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

		StereoCalibrateParams _params;//配置标定的参数
		Mat _R, _T, _E, _F, _perViewError; //立体匹配结果
	};

	static const string path_sep = "\\";
	static const string ele_sep = ",";
}
