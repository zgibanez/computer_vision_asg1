#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
using namespace cv;
using namespace std;

vector<Mat> GetImageListFromFolder(string folder = "data/");
vector<Mat> ExtractCornersFromFiles(vector<Mat> imageList, vector<vector<Point2f>>& foundCorners, bool show = false);
vector<Point3f> getKnownBoardPosition(Size boardSize, float squareLength);
void saveCalibrationParameters(string parametersFileName, Mat cameraMatrix, Mat distCoeffs);
void DrawCube();

string parametersFileName = "cameraParameters";

const float squareLength = 0.0245f;
Size boardSize = Size(6, 9);

int main()
{
	vector<Mat> imagesFromFolder = GetImageListFromFolder();

	vector<vector<Point2f>> foundCorners;

	vector<Mat> imagesWithCorners = ExtractCornersFromFiles(imagesFromFolder, foundCorners); //For DEBUG purposes we retrieve the images

	vector<Point3f> knownChessboardCoordinates = getKnownBoardPosition(boardSize, squareLength);

	vector<vector<Point3f>> worldCoordinatesCorners(1);
	worldCoordinatesCorners[0] = knownChessboardCoordinates;
	worldCoordinatesCorners.resize(foundCorners.size(), worldCoordinatesCorners[0]);

	//Camera intrinsic parameters
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	Mat rvecs, tvecs;

	calibrateCamera(worldCoordinatesCorners, foundCorners, boardSize, cameraMatrix, distCoeffs, rvecs, tvecs);
	cout << cameraMatrix << endl;
	cout << distCoeffs << endl;
	//saveCalibrationParameters(parametersFileName,cameraMatrix,distCoeffs);

	VideoCapture vid(1);
	Mat frame, frameUndistorted;

	//If video cannot be opened, terminate program.
	if (!vid.isOpened())
	{
		return -1;
	}


	while (true) 
	{
		//If frame is non-existent, terminate program.
		if (!vid.read(frame)) 
		{
			return -1;
		}
		undistort(frame, frameUndistorted, cameraMatrix, distCoeffs);
		imshow("webcam", frame);
		imshow("webcam undistorted", frameUndistorted);
		
		if (waitKey(30) == 27) {
			return 0;
		}
	}

	namedWindow("Photos");
	
	return 0;
}


void saveCalibrationParameters(string parametersFileName, Mat cameraMatrix, Mat distCoeffs) 
{
	
	ofstream out;

	if (!out)
	{
		cout << "File Writing failed" << endl;  return;
	}

	double value;

	//Save CameraMatrix values
	for (int i = 0; i < cameraMatrix.rows; i++) {
		for (int j = 0; i < cameraMatrix.cols; j++) {
			value = cameraMatrix.at<double>(i, j);
			out << value << endl;
		}
	}
	
	cout << "rows completed" << endl;

	for (int i = 0; i < distCoeffs.rows; i++) {
		for (int j = 0; i < distCoeffs.cols; j++) {
			value = distCoeffs.at<double>(i, j);
			out << value << endl;
		}
	}

			cout << "parameterFile successfully saved" << endl;
	out.close();
}

vector<Point3f> getKnownBoardPosition(Size boardSize, float squareLength) 
{
	vector<Point3f> chessBoardCorners;
	Point3f tempPoint;
	for (int i = 0; i < boardSize.height; i++) 
	{
		for (int j = 0; j < boardSize.width; j++) 
		{
			tempPoint = Point3f(j*squareLength, i*squareLength,0.0f);
			chessBoardCorners.push_back(tempPoint);
		}
	}

	return chessBoardCorners;
}

///<summary>
///Retrieves all the files in the "folder" path and stores them in a vector of images.
///</summary>
vector<Mat> GetImageListFromFolder(string folder)
{
	vector<Mat> imagesFromFolder;
	vector<String> fileNames;
	glob(folder, fileNames, false);
	
	size_t fileNumber = fileNames.size();
	for (size_t i = 0; i < fileNumber; i++) 
	{
		imagesFromFolder.push_back(imread(fileNames[i]));
	}
	
	return imagesFromFolder;
}

///<summary>
///Retrieves a list of images for which the chessboard corners have been found.
///</summary>
vector<Mat> ExtractCornersFromFiles(vector<Mat> imageList, vector<vector<Point2f>>& foundCorners, bool show)
{
	vector<Mat> imagesWithCorners;

	for (int i = 0; i < imageList.size(); i++)
	{
		vector<Point2f> tempCorners;
		bool success = findChessboardCorners(imageList.at(i), boardSize, tempCorners, CV_CALIB_CB_ADAPTIVE_THRESH); //USE CALIB_FAST_CHECK for real-time
		if (success) { 
			foundCorners.push_back(tempCorners);
			drawChessboardCorners(imageList.at(i), boardSize, tempCorners, success);
			imagesWithCorners.push_back(imageList.at(i));
		}
	}

	if (show) {

		size_t count = imagesWithCorners.size();
		for (size_t i = 0; i<count; i++)
		{
			imshow("Photos", imagesWithCorners[i]);
			waitKey(0);
		}
	}

	return imagesWithCorners;
}

void DrawCube() {

}
