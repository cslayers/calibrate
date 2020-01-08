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

	//假设dir下有两个文件夹 dir/1 dir/2
	//子文件夹下放着若干图片 1.bmp 2.bmp ... 10.bmp ...
	//文件扩展名不是bmp需要指出
	//输出文件RawCalibration.csv会处于和dir所在的目录下

	string dir = ".\\data"; //源码下的data文件夹
	string image_file_ext = "jpg";
	calib::StereoCalibrateParams params;
	params.grid = 8;
	params.patternSize = cv::Size(12, 9);
	calib::StereoCalibrate sc(dir, image_file_ext, params);
	sc.start();
	return 0;
}
