#include "opencv2/opencv.hpp"
#include <iostream>
#include "Detections.h"
#include <cmath>
#include "StickFigure.h"

#define NFRAMES 241

using namespace std;
using namespace cv;

int videoWidth = 800;
int videoHeight = 800;

void GenerateBackgroundAndSave()
{
    const string NAME = "Background.mp4";

    VideoWriter outputVideo;
    Size S = Size(videoWidth, videoHeight);
    int fps = 24;
    int ex = outputVideo.fourcc('m', 'p', '4', 'v');
    bool isColor = true;
    outputVideo.open(NAME, ex, fps, S, isColor);

    int nframes = NFRAMES;

    for (int t = 0; t < nframes; t++)
    {
        Mat frame(videoWidth, videoHeight, CV_8UC3);

        GenerateBackground(frame, t);
        outputVideo << frame;

        cout << "Processing frame " << t << "...." << endl;
    }
}

void RenderStickFigureOnVideo(String backgroundPath, String outputName)
{
    Mat frame;
    VideoCapture cap(backgroundPath);

    Size s = Size(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT));
    int fps = (int)cap.get(CAP_PROP_FPS);

    VideoWriter outputVideo;
    int ex = outputVideo.fourcc('m', 'p', '4', 'v');
    outputVideo.open(outputName, ex, fps, s, true);

    int frameCount = NFRAMES;

    if (!cap.isOpened())
    {
        cout << "Error loading video" << endl;
        return;
    }

    for (double i = 0; i < frameCount; i++)
    {
        cap >> frame;

        StickFigure(frame, i * 3 + 40, s.height / 3, i, i / 40);
        outputVideo << frame;

        // imshow("output", frame);
        if ((waitKey(1) & 0xFF) == 'q')
            break;
    }
}

int main()
{
    GenerateBackgroundAndSave();
    RenderStickFigureOnVideo("Background.mp4", "stickfigure.mp4");

    VideoCapture generatedVideo("stickfigure.mp4");

    Mat frame;
    while (1)
    {
        if (!generatedVideo.read(frame))
        {
            cout << "\nCannot read the video file.\n";
            break;
        }

        imshow("StickFigure", frame);

        if (waitKey(30) == 27)
        {
            break;
        }
    }

    BlockMatchingMethod(generatedVideo);

    VideoCapture generatedVideo2("stickfigure.mp4");
    HoughTracking(generatedVideo2);
}


