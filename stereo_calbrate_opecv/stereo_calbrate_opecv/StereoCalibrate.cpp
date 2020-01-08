#include "StereoCalibrate.h"

#include <iostream>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <iomanip>

using std::cout;
using std::endl;


namespace calib {

	void calibrate_single(CameraParameters& cps) {

		cv::Mat stdIn, stdEx, perViewError;

		cv::calibrateCamera(cps.objectPoints, cps.imagePoints, cps.imageSize,
			cps.K, cps.D, cps.rvecs, cps.tvecs, stdIn, stdEx, perViewError, cps.single_flag);


		D(cout << "内参" << endl);
		D(cout << cps.K << endl);

		D(cout << "畸变" << endl);
		D(cout << cps.D << endl);

		D(cout << "重投影误差 每张图" << endl);
		D(cout << perViewError << endl);

		D(cout << endl << endl);
	}

	void get_imagePoints_from_cps(vv2p_t& imagePoints1, vv2p_t& imagePoints2,
		CameraParameters& cps1, CameraParameters&cps2, vector<bool>& both_detected) {
		//这里要复用单相机标定检测的圆点坐标,需要小心这里

		//这个向量存图像在cps的imagePoints的下标
		//有些图没有检测成功并不在里面，所以下标设为和上一个一样
		//0没有上一个所以特殊对待
		vector<int> cur_checked_1(cps1.detected.size(), 0);
		vector<int> cur_checked_2(cps2.detected.size(), 0);
		for (int i = 0; i < cps1.detected.size(); i++) {
			if (cps1.detected[i]) {
				if (i == 0) cur_checked_1[i] = 0;
				else {
					cur_checked_1[i] = cur_checked_1[i - 1] + 1;
				}
			}
			else {
				if (i == 0) cur_checked_1[i] = 0;
				cur_checked_1[i] = cur_checked_1[i - 1];
			}
		}

		//相似的
		for (int i = 0; i < cps2.detected.size(); i++) {
			if (cps2.detected[i]) {
				if (i == 0) cur_checked_2[i] = 0;
				else {
					cur_checked_2[i] = cur_checked_2[i - 1] + 1;
				}
			}
			else {
				if (i == 0) cur_checked_2[i] = 0;
				cur_checked_2[i] = cur_checked_2[i - 1];
			}
		}


		for (int i = 0; i < both_detected.size(); i++) {
			if (both_detected[i]) {
				imagePoints1.push_back(cps1.imagePoints[cur_checked_1[i]]);
				imagePoints2.push_back(cps2.imagePoints[cur_checked_2[i]]);
			}
		}

		/*for (int i = 0; i < cps1.detected.size(); i++) {
			cout << both_detected[i] << "\t";
		}cout << endl;

		for (int i = 0; i < cps1.detected.size(); i++) {
			cout << cps1.detected[i] << "\t";
		}cout << endl;
		for (int i = 0; i < cps1.detected.size(); i++) {
			cout << cur_checked_1[i] << "\t";
		}cout << endl;

		for (int i = 0; i < cps1.detected.size(); i++) {
			cout << cps2.detected[i] << "\t";
		}cout << endl;
		for (int i = 0; i < cps1.detected.size(); i++) {
			cout << cur_checked_2[i] << "\t";
		}cout << endl;*/
	}



	void calibrate_stereo(CameraParameters& cps1, CameraParameters& cps2, 
		Mat& R, Mat& T, Mat& E, Mat& F, Mat& perViewError, StereoCalibrateParams params) {

		assert(cps1.filepaths.size() == cps2.filepaths.size());
		assert(cps1.detected.size() == cps2.detected.size());

		vector<bool> both_detected(cps1.filepaths.size(), false);

		for (int i = 0; i < both_detected.size(); i++)
			if (cps1.detected[i] && cps2.detected[i])
				both_detected[i] = true;

		size_t view_num = std::count_if(both_detected.begin(), both_detected.end(), [](bool x)->bool {return x; });


		vv3p_t objectPoints = get_objectPoints(params.patternSize, view_num, params.grid);
		vv2p_t imagePoints1;
		vv2p_t imagePoints2;

		get_imagePoints_from_cps(imagePoints1, imagePoints2, cps1, cps2, both_detected);

		//Mat R, T, E, F,perViewError;
		int flag = CV_CALIB_RATIONAL_MODEL;
		cv::stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
			cps1.K, cps1.D, cps2.K, cps2.D, cps1.imageSize,
			R, T, E, F, perViewError,
			flag);


		D(cout << "旋转矩阵" << endl);
		D(cout << R << endl);

		D(cout << "平移矩阵" << endl);
		D(cout << T << endl);

		D(cout << "立体匹配误差 每张图" << endl);
		D(cout << perViewError << endl);
		cout << "平均误差： " << cv::sum(perViewError)[0] / double(view_num * 2) << endl; //sum函数返回一个4元素的向量，对应4个chanel


		D(cout << "畸变" << endl);
		D(cout << cps1.D << endl);
		D(cout << cps2.D << endl);

		D(cout << "内参" << endl);
		D(cout << cps1.K << endl);
		D(cout << cps2.K << endl);

		D(cout << "两相机检测均成功的图片数量： " << view_num << endl)

	}


	void prepare_single(CameraParameters& cps, string pattern1, StereoCalibrateParams params) {
		vector<cv::String> paths1;
		cv::glob(pattern1, paths1, false);
		for (auto p : paths1)
			cps.filepaths.push_back(p);
		std::sort(cps.filepaths.begin(), cps.filepaths.end(), filename_comparator());
		cps.detected.assign(cps.filepaths.size(), true); //初始设为true,检测失败就将他改为false

		cps.imageSize = get_imagePoints(cps.filepaths, cps.detected, cps.imagePoints, params.patternSize);
		size_t number_sucess_1 = cps.imagePoints.size();
		cps.objectPoints = get_objectPoints(params.patternSize, number_sucess_1, params.grid);


		D(cout << "排序前" << endl);
		for (auto p : paths1)
			D(cout << p << endl);
		D(cout << "排序后" << endl);
		for (auto p : cps.filepaths)
			D(cout << p << endl);

		D(cout << "文件数" << cps.filepaths.size() << endl);
		D(cout << "成功检测数目：" << cps.imagePoints.size() << endl);
		D(cout << endl << endl);

		auto lambda_true = [](bool x)->bool {return x; };
		size_t sucess_num_1 = std::count_if(cps.detected.begin(), cps.detected.end(), lambda_true);

	}



	StereoCalibrate::StereoCalibrate(string dir, string ext = "bmp", StereoCalibrateParams params = StereoCalibrate::createStereoCalibrateParams())
		:_dir(dir), _ext(ext), _params(params) {
		//假设文件夹下有1 2 两个子文件夹
		_sub_dir_1 = _dir + path_sep + "1" + path_sep;
		_sub_dir_2 = _dir + path_sep + "2" + path_sep;
	}


	void StereoCalibrate::start() {

		string pattern1 = _sub_dir_1 + "*." + _ext; //图片文件路径的pattern
		string pattern2 = _sub_dir_2 + "*." + _ext;

		prepare_single(_cps1, pattern1, _params);
		prepare_single(_cps2, pattern2, _params);

		calibrate_single(_cps1);
		calibrate_single(_cps2);

		calibrate_stereo(_cps1, _cps2, _R, _T, _E, _F, _perViewError, _params);

		if (write_calibration_file() != 0) cout << "写标定文件失败" << endl;
	}


	int StereoCalibrate::write_calibration_file() {
		string out_file_path = _dir + path_sep + ".." + path_sep + "RawCalibration.csv";
		D(cout << "企图输出文件 立体标定结果: " << out_file_path << endl);
		std::ofstream out(out_file_path);



		if (out.is_open()) {
			out.setf(std::ios::fixed);
			out << std::setprecision(8);

			//补充计算
			//TODO 旋转向量这里不太明确，其元素是否是旋转角度，而且顺序是存疑的
			Mat rvec;
			cv::Rodrigues(_R, rvec); //从旋转矩阵到旋转向量，PMLAB也是输出了旋转向量

			int view_num = _perViewError.size().height;
			double score = cv::sum(_perViewError)[0] / double(view_num * 2);



			//不要在前面插入行，可以在后面插入行,因为其他工具未必会更新
			{
				out << "立体标定输出文件" << ele_sep << "score" << ele_sep << score << ele_sep << "grid" << ele_sep << _params.grid << endl;
				out << "head" << ele_sep << "cam1" << ele_sep << "cam2" << endl;
				out << "Cx" << ele_sep << _cps1.K.at<double>(0, 2) << ele_sep << _cps2.K.at<double>(0, 2) << endl; //第三行
				out << "Cy" << ele_sep << _cps1.K.at<double>(1, 2) << ele_sep << _cps2.K.at<double>(1, 2) << endl;
				out << "Fx" << ele_sep << _cps1.K.at<double>(0, 0) << ele_sep << _cps2.K.at<double>(0, 0) << endl;
				out << "Fy" << ele_sep << _cps1.K.at<double>(1, 1) << ele_sep << _cps2.K.at<double>(1, 1) << endl;
				out << "Fs" << ele_sep << _cps1.K.at<double>(0, 1) << ele_sep << _cps2.K.at<double>(0, 1) << endl;
				out << "K1" << ele_sep << _cps1.D.at<double>(0, 0) << ele_sep << _cps2.D.at<double>(0, 0) << endl; //第八行
				out << "K2" << ele_sep << _cps1.D.at<double>(0, 1) << ele_sep << _cps2.D.at<double>(0, 1) << endl;
				out << "P1" << ele_sep << _cps1.D.at<double>(0, 2) << ele_sep << _cps2.D.at<double>(0, 2) << endl;
				out << "P2" << ele_sep << _cps1.D.at<double>(0, 3) << ele_sep << _cps2.D.at<double>(0, 3) << endl;
				out << "K3" << ele_sep << _cps1.D.at<double>(0, 4) << ele_sep << _cps2.D.at<double>(0, 4) << endl;
				out << "K4" << ele_sep << _cps1.D.at<double>(0, 5) << ele_sep << _cps2.D.at<double>(0, 5) << endl;
				out << "K5" << ele_sep << _cps1.D.at<double>(0, 6) << ele_sep << _cps2.D.at<double>(0, 6) << endl;
				out << "K6" << ele_sep << _cps1.D.at<double>(0, 7) << ele_sep << _cps2.D.at<double>(0, 7) << endl; //第15行
				out << "Alpha" << ele_sep << 0 << ele_sep << rvec.at<double>(0, 0) << endl; //第16行
				out << "Betar" << ele_sep << 0 << ele_sep << rvec.at<double>(0, 1) << endl;
				out << "Gamma" << ele_sep << 0 << ele_sep << rvec.at<double>(0, 2) << endl;
				out << "Tx" << ele_sep << 0 << ele_sep << _T.at<double>(0, 0) << endl;//第19行
				out << "Ty" << ele_sep << 0 << ele_sep << _T.at<double>(0, 1) << endl;
				out << "Tz" << ele_sep << 0 << ele_sep << _T.at<double>(0, 2) << endl;//第21行
			}
			cout << "成功写标定文件：" << out_file_path << endl;


			//参考资料 Detailed Description https://docs.opencv.org/3.4.5/d9/d0c/group__calib3d.html
			out.close();
			return 0;
		}
		else
		{
			return 1;
		}
	}




	StereoCalibrate::~StereoCalibrate() {

	}

}

