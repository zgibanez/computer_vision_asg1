//
//  infomcv_calibration.hpp
//  Assignment_1
//
//  Created by Juan Fonseca on 2/17/17.
//  Copyright © 2017 Juan Fonseca. All rights reserved.
//

#ifndef infomcv_calibration_hpp
#define infomcv_calibration_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#endif /* infomcv_calibration_hpp */

using namespace cv;
using namespace std;

vector<Mat> GetImageListFromFolder(string folder = "data/");
void ExtractCornersFromFiles(vector<Mat> imageList, vector<vector<Point2f>>& foundCorners, bool show = false);
vector<Point3f> getKnownBoardPosition(Size boardSize, float squareLength);

bool calibrate(int camera,Mat& cameraMatrix, Mat& distCoeff, Mat& rvecs, Mat& tvecs);

bool readCalibrationParameters(string paramFile, Mat& cameraMatrix, Mat& distCoeffs, Mat& rvecs, Mat& tvecs);

bool saveCalibrationParameters(string parametersFileName, Mat& cameraMatrix, Mat& distCoeffs, Mat& rvecs, Mat& tvecs);

const float squareLength = 0.0245f;
const Size boardSize = Size(6, 9);

