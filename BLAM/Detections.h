#pragma once
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

int BlockMatchingMethod(VideoCapture& input);

vector<Vec3f> DetectCirclesHough(Mat* p, int* check);
Mat FilterRedMask(const Mat& src);
void DrawTrackedObject(int x, int y, Mat& frame, int check);
void ApplyMorphology(Mat& thresh);
void TrackObject(Mat image, Mat HSV, Mat& cameraFeed, int lineCheck);

void HoughTracking(VideoCapture& stream1);



