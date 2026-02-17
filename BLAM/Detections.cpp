#include "opencv2/opencv.hpp"
#include <iostream>
#include <math.h>
#include "Detections.h"

using namespace cv;
using namespace std;

const string originalWindowName = "Example 1. Motion extraction. Static method";
const string backgroundWindowName = "Example 1. Background";
const string motionWindowName = "Example 1. Motion";

const string trackbarName = "Threshold";
const string windowName = "Example 1. Block matching. Mean Absolute Difference";

const int ESC = 27;
const int SPC = 32;

const Size blockSize(16, 16);
const Size searchSize(48, 48);
const int differenceHue = 0;

const int startingFrame = 1;

int thresh = 50;

// hough / tracking
const int MAX_NUM_OBJECTS = 50;            // maximum number of objects
const int MIN_OBJECT_AREA = 30 * 30;       // minimum object area
const int FRAME_WIDTH = 800;
const int FRAME_HEIGHT = 800;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

int Threshold = 40;
int R;
int prevX, prevY;

int BlockMatchingMethod(VideoCapture& input)
{
    if (!input.isOpened()) return EXIT_FAILURE;

    Size frameSize(input.get(CAP_PROP_FRAME_WIDTH), input.get(CAP_PROP_FRAME_HEIGHT));
    if (frameSize.width % blockSize.width != 0 || frameSize.height % blockSize.height != 0) return EXIT_FAILURE;

    int xCount = frameSize.width / blockSize.width;
    int yCount = frameSize.height / blockSize.height;

    namedWindow(windowName, WINDOW_AUTOSIZE);

    double fps = input.get(CAP_PROP_FPS);
    int delay = int(1000 / fps);

    input.set(CAP_PROP_POS_FRAMES, startingFrame);

    Mat previousFrame;
    int t = 0;
    bool paused = false;

    while (true)
    {
        if (!paused)
        {
            cout << "Frame " << t << " being processed" << endl;
            t++;

            Mat currentFrame;
            input >> currentFrame;
            if (currentFrame.empty()) break;

            Mat displayFrame;
            currentFrame.copyTo(displayFrame);

            cvtColor(currentFrame, currentFrame, COLOR_RGB2GRAY);
            currentFrame.convertTo(currentFrame, CV_32F);

            if (!previousFrame.empty())
            {
                Mat copyFrame;
                displayFrame.copyTo(copyFrame);

                cvtColor(displayFrame, displayFrame, COLOR_RGB2YCrCb);
                vector<Mat> channels;
                split(displayFrame, channels);

                bool hasRedPixel;

                for (int blockX = 0; blockX < xCount; blockX++)
                    for (int blockY = 0; blockY < yCount; blockY++)
                    {
                        hasRedPixel = false;

                        for (int i = 0; i < 16; i++)
                        {
                            for (int j = 0; j < 16; j++)
                            {
                                if (copyFrame.at<Vec3b>(blockY * 16 + j, blockX * 16 + i)[2] == 255) {
                                    hasRedPixel = true;
                                }
                                else {
                                    continue;
                                }
                            }
                        }

                        if (hasRedPixel)
                        {
                            Point blockPos(blockX * blockSize.width, blockY * blockSize.height);

                            Point searchPos(
                                blockPos.x - (searchSize.width - blockSize.width) / 2,
                                blockPos.y - (searchSize.height - blockSize.height) / 2
                            );

                            Rect blockRect(blockPos, blockSize);
                            Mat block = previousFrame(blockRect);

                            Point bestPos;
                            float bestResult = 100000;

                            int searchXRange = searchSize.width - blockSize.width;
                            int searchYRange = searchSize.height - blockSize.height;

                            for (int searchX = 0; searchX < searchXRange; searchX += 4)
                                for (int searchY = 0; searchY < searchYRange; searchY += 4)
                                {
                                    Point kernelPos(searchPos.x + searchX, searchPos.y + searchY);

                                    if (kernelPos.x < 0 || kernelPos.x + blockSize.width >= frameSize.width) continue;
                                    if (kernelPos.y < 0 || kernelPos.y + blockSize.height >= frameSize.height) continue;

                                    Rect kernelRect(kernelPos, blockSize);
                                    Mat kernel = currentFrame(kernelRect);

                                    Mat diff;
                                    absdiff(block, kernel, diff);
                                    Scalar result = mean(diff);

                                    if (result[0] < bestResult)
                                    {
                                        bestResult = result[0];
                                        bestPos = kernelPos;
                                    }
                                }

                            Rect bestRect(bestPos.x, bestPos.y, blockSize.width, blockSize.height);

                            if (bestResult <= thresh / 10.0) {
                                channels[1](blockRect).setTo(128);
                                channels[2](blockRect).setTo(128);
                            }
                            else {
                                int mvx = bestPos.x - blockRect.x;
                                int mvy = bestPos.y - blockRect.y;

                                line(channels[0], blockPos, bestPos, Scalar(255, 255, 255), 2);

                                channels[1](blockRect).setTo(128 + 10 * mvx);
                                channels[2](blockRect).setTo(128 + 10 * mvy);
                            }
                        }
                    }

                merge(channels, displayFrame);
                cvtColor(displayFrame, displayFrame, COLOR_YCrCb2RGB);
            }

            imshow(windowName, displayFrame);
            currentFrame.copyTo(previousFrame);
        }

        int key = waitKey(delay);
        switch (key)
        {
        case SPC:
            paused = !paused;
            break;
        case ESC:
            return EXIT_SUCCESS;
        default:
            break;
        }
    }

    return EXIT_SUCCESS;
}

vector<Vec3f> DetectCirclesHough(Mat* p, int* check)
{
    Mat inImage, inImageY;
    (*p).copyTo(inImageY);
    (*p).copyTo(inImage);

    GaussianBlur(inImageY, inImageY, Size(3, 3), 2, 2);

    vector<Vec3f> circles;
    HoughCircles(inImageY, circles, HOUGH_GRADIENT_ALT, 2, 1, 180, 0.7, 1, 20);

    *check = (circles.size() == 1) ? 1 : 0;
    return circles;
}

Mat FilterRedMask(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat redOnly;
    inRange(src, Scalar(0, 0, 0), Scalar(0, 0, 255), redOnly);

    return redOnly;
}

void DrawTrackedObject(int x, int y, Mat& frame, int check)
{
    if (check == 0)
    {
        prevX = x;
        prevY = y;
    }

    line(frame, Point(x, y), Point(prevX, prevY), Scalar(255, 0, 0), 2, 20);
    circle(frame, Point(x, y), 40, Scalar(0, 0, 255));
    rectangle(frame, Point(x - R, y - R), Point(prevX + R, prevY + R), Scalar(0, 255, 0), 1, 8, 0);

    prevX = x;
    prevY = y;
}

void ApplyMorphology(Mat& thresh)
{
    Mat elementErode = getStructuringElement(MORPH_RECT, Size(4, 4));
    Mat elementDilate = getStructuringElement(MORPH_RECT, Size(8, 8));

    erode(thresh, thresh, elementErode);
    dilate(thresh, thresh, elementDilate);
}

void TrackObject(Mat image, Mat HSV, Mat& cameraFeed, int lineCheck)
{
    int x, y;
    int check = lineCheck;

    Mat temp;
    image.copyTo(temp);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    double refArea = 0;
    bool objectFound = false;

    if (hierarchy.size() > 0)
    {
        int numObjects = (int)hierarchy.size();

        if (numObjects < MAX_NUM_OBJECTS)
        {
            for (int index = 0; index >= 0; index = hierarchy[index][0])
            {
                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                if (area > MIN_OBJECT_AREA)
                {
                    x = moment.m10 / area;
                    y = moment.m01 / area;
                    objectFound = true;
                }
                else objectFound = false;
            }

            if (objectFound == true)
            {
                DrawTrackedObject(x, y, cameraFeed, check);
            }
        }
        else {
            putText(cameraFeed, "Error too much noise", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
        }
    }
}

void HoughTracking(VideoCapture& stream1)
{
    Mat gray, image;
    Mat frame;
    Mat threshold;
    Mat HSV;

    int check, prevCheck = 0, permission = 0;
    vector<Vec3f> previousCircles;

    while (true)
    {
        stream1 >> frame;
        if (frame.empty()) break;

        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);
        gray.copyTo(image);
        cvtColor(image, image, COLOR_GRAY2RGB);
        image.copyTo(frame);

        cvtColor(frame, HSV, COLOR_BGR2HSV);

        vector<Vec3f> currentCircles = DetectCirclesHough(&gray, &check);

        if (previousCircles.size() == 0)
        {
            previousCircles = currentCircles;
            continue;
        }

        FilterRedMask(image).copyTo(threshold);
        ApplyMorphology(threshold);

        if ((prevCheck == 1) && (prevCheck == 1))
            permission = 1;
        else
            permission = 0;

        cvtColor(gray, gray, COLOR_GRAY2RGB);
        TrackObject(threshold, HSV, gray, permission);

        prevCheck = check;

        for (int i = 0; i < (int)currentCircles.size(); i++)
        {
            for (int j = 0; j < (int)previousCircles.size(); j++)
            {
                double distance = sqrt(
                    pow((currentCircles[i][0] - previousCircles[j][0]), 2) +
                    pow((currentCircles[i][1] - previousCircles[j][1]), 2)
                );

                if (distance <= 16)
                {
                    Point prevPt(previousCircles[j][0], previousCircles[j][1]);
                    Point currPt(currentCircles[i][0], currentCircles[i][1]);

                    arrowedLine(gray, prevPt, currPt, 100, 1, 8, 0, 1);
                    break;
                }
            }
        }

        previousCircles = currentCircles;

        imshow("Circle_Tracking", gray);

        if (waitKey(30) >= 0)
            break;
    }
}
