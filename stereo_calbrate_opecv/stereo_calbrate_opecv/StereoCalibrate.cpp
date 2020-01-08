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


		D(cout << "�ڲ�" << endl);
		D(cout << cps.K << endl);

		D(cout << "����" << endl);
		D(cout << cps.D << endl);

		D(cout << "��ͶӰ��� ÿ��ͼ" << endl);
		D(cout << perViewError << endl);

		D(cout << endl << endl);
	}

	void get_imagePoints_from_cps(vv2p_t& imagePoints1, vv2p_t& imagePoints2,
		CameraParameters& cps1, CameraParameters&cps2, vector<bool>& both_detected) {
		//����Ҫ���õ�����궨����Բ������,��ҪС������

		//���������ͼ����cps��imagePoints���±�
		//��Щͼû�м��ɹ����������棬�����±���Ϊ����һ��һ��
		//0û����һ����������Դ�
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

		//���Ƶ�
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


		D(cout << "��ת����" << endl);
		D(cout << R << endl);

		D(cout << "ƽ�ƾ���" << endl);
		D(cout << T << endl);

		D(cout << "����ƥ����� ÿ��ͼ" << endl);
		D(cout << perViewError << endl);
		cout << "ƽ���� " << cv::sum(perViewError)[0] / double(view_num * 2) << endl; //sum��������һ��4Ԫ�ص���������Ӧ4��chanel


		D(cout << "����" << endl);
		D(cout << cps1.D << endl);
		D(cout << cps2.D << endl);

		D(cout << "�ڲ�" << endl);
		D(cout << cps1.K << endl);
		D(cout << cps2.K << endl);

		D(cout << "����������ɹ���ͼƬ������ " << view_num << endl)

	}


	void prepare_single(CameraParameters& cps, string pattern1, StereoCalibrateParams params) {
		vector<cv::String> paths1;
		cv::glob(pattern1, paths1, false);
		for (auto p : paths1)
			cps.filepaths.push_back(p);
		std::sort(cps.filepaths.begin(), cps.filepaths.end(), filename_comparator());
		cps.detected.assign(cps.filepaths.size(), true); //��ʼ��Ϊtrue,���ʧ�ܾͽ�����Ϊfalse

		cps.imageSize = get_imagePoints(cps.filepaths, cps.detected, cps.imagePoints, params.patternSize);
		size_t number_sucess_1 = cps.imagePoints.size();
		cps.objectPoints = get_objectPoints(params.patternSize, number_sucess_1, params.grid);


		D(cout << "����ǰ" << endl);
		for (auto p : paths1)
			D(cout << p << endl);
		D(cout << "�����" << endl);
		for (auto p : cps.filepaths)
			D(cout << p << endl);

		D(cout << "�ļ���" << cps.filepaths.size() << endl);
		D(cout << "�ɹ������Ŀ��" << cps.imagePoints.size() << endl);
		D(cout << endl << endl);

		auto lambda_true = [](bool x)->bool {return x; };
		size_t sucess_num_1 = std::count_if(cps.detected.begin(), cps.detected.end(), lambda_true);

	}



	StereoCalibrate::StereoCalibrate(string dir, string ext = "bmp", StereoCalibrateParams params = StereoCalibrate::createStereoCalibrateParams())
		:_dir(dir), _ext(ext), _params(params) {
		//�����ļ�������1 2 �������ļ���
		_sub_dir_1 = _dir + path_sep + "1" + path_sep;
		_sub_dir_2 = _dir + path_sep + "2" + path_sep;
	}


	void StereoCalibrate::start() {

		string pattern1 = _sub_dir_1 + "*." + _ext; //ͼƬ�ļ�·����pattern
		string pattern2 = _sub_dir_2 + "*." + _ext;

		prepare_single(_cps1, pattern1, _params);
		prepare_single(_cps2, pattern2, _params);

		calibrate_single(_cps1);
		calibrate_single(_cps2);

		calibrate_stereo(_cps1, _cps2, _R, _T, _E, _F, _perViewError, _params);

		if (write_calibration_file() != 0) cout << "д�궨�ļ�ʧ��" << endl;
	}


	int StereoCalibrate::write_calibration_file() {
		string out_file_path = _dir + path_sep + ".." + path_sep + "RawCalibration.csv";
		D(cout << "��ͼ����ļ� ����궨���: " << out_file_path << endl);
		std::ofstream out(out_file_path);



		if (out.is_open()) {
			out.setf(std::ios::fixed);
			out << std::setprecision(8);

			//�������
			//TODO ��ת�������ﲻ̫��ȷ����Ԫ���Ƿ�����ת�Ƕȣ�����˳���Ǵ��ɵ�
			Mat rvec;
			cv::Rodrigues(_R, rvec); //����ת������ת������PMLABҲ���������ת����

			int view_num = _perViewError.size().height;
			double score = cv::sum(_perViewError)[0] / double(view_num * 2);



			//��Ҫ��ǰ������У������ں��������,��Ϊ��������δ�ػ����
			{
				out << "����궨����ļ�" << ele_sep << "score" << ele_sep << score << ele_sep << "grid" << ele_sep << _params.grid << endl;
				out << "head" << ele_sep << "cam1" << ele_sep << "cam2" << endl;
				out << "Cx" << ele_sep << _cps1.K.at<double>(0, 2) << ele_sep << _cps2.K.at<double>(0, 2) << endl; //������
				out << "Cy" << ele_sep << _cps1.K.at<double>(1, 2) << ele_sep << _cps2.K.at<double>(1, 2) << endl;
				out << "Fx" << ele_sep << _cps1.K.at<double>(0, 0) << ele_sep << _cps2.K.at<double>(0, 0) << endl;
				out << "Fy" << ele_sep << _cps1.K.at<double>(1, 1) << ele_sep << _cps2.K.at<double>(1, 1) << endl;
				out << "Fs" << ele_sep << _cps1.K.at<double>(0, 1) << ele_sep << _cps2.K.at<double>(0, 1) << endl;
				out << "K1" << ele_sep << _cps1.D.at<double>(0, 0) << ele_sep << _cps2.D.at<double>(0, 0) << endl; //�ڰ���
				out << "K2" << ele_sep << _cps1.D.at<double>(0, 1) << ele_sep << _cps2.D.at<double>(0, 1) << endl;
				out << "P1" << ele_sep << _cps1.D.at<double>(0, 2) << ele_sep << _cps2.D.at<double>(0, 2) << endl;
				out << "P2" << ele_sep << _cps1.D.at<double>(0, 3) << ele_sep << _cps2.D.at<double>(0, 3) << endl;
				out << "K3" << ele_sep << _cps1.D.at<double>(0, 4) << ele_sep << _cps2.D.at<double>(0, 4) << endl;
				out << "K4" << ele_sep << _cps1.D.at<double>(0, 5) << ele_sep << _cps2.D.at<double>(0, 5) << endl;
				out << "K5" << ele_sep << _cps1.D.at<double>(0, 6) << ele_sep << _cps2.D.at<double>(0, 6) << endl;
				out << "K6" << ele_sep << _cps1.D.at<double>(0, 7) << ele_sep << _cps2.D.at<double>(0, 7) << endl; //��15��
				out << "Alpha" << ele_sep << 0 << ele_sep << rvec.at<double>(0, 0) << endl; //��16��
				out << "Betar" << ele_sep << 0 << ele_sep << rvec.at<double>(0, 1) << endl;
				out << "Gamma" << ele_sep << 0 << ele_sep << rvec.at<double>(0, 2) << endl;
				out << "Tx" << ele_sep << 0 << ele_sep << _T.at<double>(0, 0) << endl;//��19��
				out << "Ty" << ele_sep << 0 << ele_sep << _T.at<double>(0, 1) << endl;
				out << "Tz" << ele_sep << 0 << ele_sep << _T.at<double>(0, 2) << endl;//��21��
			}
			cout << "�ɹ�д�궨�ļ���" << out_file_path << endl;


			//�ο����� Detailed Description https://docs.opencv.org/3.4.5/d9/d0c/group__calib3d.html
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

