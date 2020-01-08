#include "cmd.h"
#include <iostream>
using namespace std;


int main(int argc, char*argv[]) {

	int x = -1;
	cout << "1(testsingle) 2(teststereo) 3(testclasscalibrate)" << endl;

	cin >> x;
	if(x == 1)
		single_calibrate(argc, argv);
	if(x == 2)
		stereo_calibrate(argc, argv);
	if (x == 3)
		test_stereoCalibrate();
	
	cin >> x;

	return EXIT_SUCCESS;
}



#include "StereoCalibrate.h"
int test_stereoCalibrate() {

	//����dir���������ļ��� dir/1 dir/2
	//���ļ����·�������ͼƬ 1.bmp 2.bmp ... 10.bmp ...
	//�ļ���չ������bmp��Ҫָ��
	//����ļ�RawCalibration.csv�ᴦ�ں�dir���ڵ�Ŀ¼��

	string dir = ".\\data"; //Դ���µ�data�ļ���
	string image_file_ext = "jpg";
	calib::StereoCalibrateParams params;
	params.grid = 8;
	params.patternSize = cv::Size(12, 9);
	calib::StereoCalibrate sc(dir, image_file_ext, params);
	sc.start();
	return 0;
}
