#include "cmd.h"
#include <iostream>
using namespace std;


int main(int argc, char*argv[]) {

	int x = -1;
	cout << "1(testsingle) 2(teststereo) 3(testclasscalibrate)" << endl;

	cin >> x;
	if (x == 1)
		single_calibrate(argc, argv);
	if (x == 2)
		stereo_calibrate(argc, argv);
	if (x == 3)
		test_stereoCalibrate(argc, argv);

	cin >> x;

	return EXIT_SUCCESS;
}



#include "StereoCalibrate.h"
int test_stereoCalibrate(int argc, char* argv[]) {

	//����dir���������ļ��� dir/1 dir/2
	//���ļ����·�������ͼƬ 1.bmp 2.bmp ... 10.bmp ...
	//�ļ���չ������bmp��Ҫָ��
	//����ļ�RawCalibration.csv�ᴦ�ں�dir���ڵ�Ŀ¼��

	string dir = R"(C:\Users\cslay\Desktop\3d-dic\sample\small\sample20200103)" + string("\\calibrate");
	if (argc == 2) {
		dir = argv[1];
	}

	string image_file_ext = "bmp";
	calib::StereoCalibrateParams params;
	params.grid = 1;
	params.patternSize = cv::Size(12, 9);
	calib::StereoCalibrate sc(dir, image_file_ext, params);
	sc.start();
	return 0;
}
