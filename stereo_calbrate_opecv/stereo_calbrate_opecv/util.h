#pragma once
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
using cv::Point2f;
using cv::Point3f;
using std::vector;
using std::string;

typedef vector<Point3f> v3p_t;
typedef vector<Point2f> v2p_t;
typedef vector<v3p_t> vv3p_t;
typedef vector<v2p_t> vv2p_t;

vv3p_t get_objectPoints(cv::Size patternSize, size_t num_view, float grid);

cv::Size get_imagePoints(vector<string>paths,vector<bool>& flags, vv2p_t &imagePoints,cv::Size patternSize);

cv::SimpleBlobDetector::Params get_blob_params();

#define COUT_DEBUG 1
#define D(x) if(COUT_DEBUG) x;


