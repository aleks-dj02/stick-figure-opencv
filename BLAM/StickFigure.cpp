#include "StickFigure.h"
#include <math.h>

const double PI = 3.14159265358979323846;

void GenerateBackground(Mat& frame, int t) {
    int width = frame.cols;
    int height = frame.rows;

    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            frame.at<Vec3b>(j, i)[0] = 155;
            frame.at<Vec3b>(j, i)[1] = 150;
            frame.at<Vec3b>(j, i)[2] = 0;
        }
    }
}

void StickFigure(Mat& frame, double x, double y, double i, double t) {
    int width = frame.cols;
    int height = frame.rows;

    int thickness = 3;
    int w = 35 + i;

    double upperArmLength = 80;
    double forearmLength = 70;
    double thighLength = 120;
    double lowerLegLength = 100;

    Point head(x, y - 55);
    Point neck(x, y - 30);
    Point shoulders(x, y);
    Point hips(x, y + 180);

    double xElbowOffset = upperArmLength * sin(t) * 1 / 2;
    double yElbowOffset = sqrt(upperArmLength * upperArmLength - xElbowOffset * xElbowOffset);
    Point rightElbow(x + xElbowOffset, y + yElbowOffset);

    double xRightHandOffset = (sin(t) < (-1 / 2)) ?
        forearmLength * sin(t) * 1 / 2 * sqrt(3) / 2 :
        forearmLength * sin(t) * sqrt(3) / 2;

    double yRightHandOffset = sqrt(forearmLength * forearmLength - xRightHandOffset * xRightHandOffset);
    Point rightHand(x + xElbowOffset + xRightHandOffset,
        y + yElbowOffset + yRightHandOffset);

    Point leftElbow(x - xElbowOffset, y + yElbowOffset);

    double xLeftHandOffset = sin(t) > 1 / 2 ?
        forearmLength * sin(t) * 1 / 2 * sqrt(3) / 2 :
        forearmLength * sin(t) * sqrt(3) / 2;

    double yLeftHandOffset = sqrt(forearmLength * forearmLength - xLeftHandOffset * xLeftHandOffset);
    Point leftHand(x - xElbowOffset - xLeftHandOffset,
        y + yElbowOffset + yLeftHandOffset);

    double xKneeOffset = thighLength * sin(t) * 1 / 2;
    double yKneeOffset = sqrt(thighLength * thighLength - xKneeOffset * xKneeOffset);

    Point rightKnee(x - xKneeOffset, y + 180 + yKneeOffset);
    Point leftKnee(x + xKneeOffset, y + 180 + yKneeOffset);

    double xRightFootOffset = sin(t) < (-1 / 2) ?
        lowerLegLength * sin(t) * 1 / 2 * sqrt(3) / 2 :
        lowerLegLength * sin(t) * sqrt(3) / 2;

    double yRightFootOffset = sqrt(lowerLegLength * lowerLegLength - xRightFootOffset * xRightFootOffset);

    Point rightFoot(x - xKneeOffset - xRightFootOffset,
        y + 180 + yKneeOffset + yRightFootOffset);

    double xLeftFootOffset = sin(t) > 1 / 2 ?
        lowerLegLength * sin(t) * 1 / 2 * sqrt(3) / 2 :
        lowerLegLength * sin(t) * sqrt(3) / 2;

    double yLeftFootOffset = sqrt(lowerLegLength * lowerLegLength - xLeftFootOffset * xLeftFootOffset);

    Point leftFoot(x + xKneeOffset + xLeftFootOffset,
        y + 180 + yKneeOffset + yLeftFootOffset);

    Scalar black(0, 0, 0);
    Scalar red(0, 0, 255);

    circle(frame, head, 25, black, thickness);
    line(frame, neck, shoulders, black, thickness);
    line(frame, shoulders, hips, black, thickness);
    line(frame, shoulders, rightElbow, black, thickness);
    line(frame, rightElbow, rightHand, black, thickness);
    line(frame, shoulders, leftElbow, black, thickness);
    line(frame, leftElbow, leftHand, black, thickness);
    line(frame, hips, rightKnee, black, thickness);
    line(frame, rightKnee, rightFoot, black, thickness);
    line(frame, hips, leftKnee, black, thickness);
    line(frame, leftKnee, leftFoot, black, thickness);

    int jointRadius = 10;
    Scalar jointColor(0, 0, 255);

    circle(frame, shoulders, jointRadius, jointColor, FILLED);
    circle(frame, rightElbow, jointRadius, jointColor, FILLED);
    circle(frame, rightHand, jointRadius, jointColor, FILLED);
    circle(frame, leftElbow, jointRadius, jointColor, FILLED);
    circle(frame, leftHand, jointRadius, jointColor, FILLED);
    circle(frame, hips, jointRadius, jointColor, FILLED);
    circle(frame, rightKnee, jointRadius, jointColor, FILLED);
    circle(frame, leftKnee, jointRadius, jointColor, FILLED);
    circle(frame, rightFoot, jointRadius, jointColor, FILLED);
    circle(frame, leftFoot, jointRadius, jointColor, FILLED);
}