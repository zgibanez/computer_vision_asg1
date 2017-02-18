#include "infomcv_calibration.hpp"

bool DrawCube(Mat frame, Mat cameraMatrix, Mat distCoeffs, vector<Point3f> boardPoints);
void getAxesInCameraCoordinates(vector<Point2d> cubePoints, Mat cameraMatrix, vector<Point3d>& cameraAxisPoints);

/*
 Idea for main: 
    calibrateCamera(number_of_the_camera, pictures_folder)
    saveCalibration
    showBoardLive
    drawCoordinateSystem
    drawCube
 */

int main()
{
	//Points on object coordinate space 
	vector<Point3f> boardPoints = getKnownBoardPosition(boardSize, squareLength);
    
    //Camera intrinsic parameters
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat::zeros(8, 1, CV_64F);

	//Camera extrinsic parameters
    Mat rvecs, tvecs;
    
	bool successfulCalibration = calibrate(1, cameraMatrix, distCoeffs, rvecs, tvecs);
	if (!successfulCalibration) {
		cout << "Camera could not be calibrated" << endl;
		return -1;
	}

    cout << "CAMERA MATRIX" << cameraMatrix << endl;
    cout << "DISTORTION COEFFICIENTS" << distCoeffs << endl;    

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

		//Apply undistortion with camera matrix
        undistort(frame, frameUndistorted, cameraMatrix, distCoeffs);
        imshow("webcam", frame);
		bool smt = DrawCube(frameUndistorted, cameraMatrix, distCoeffs, boardPoints);
        imshow("webcam undistorted", frameUndistorted);
        
        if (waitKey(30) == 27) {
            return 0;
        }
    }
    
    return 0;
}

///<summary>
/// Draws a cube around the upper-left corner of the chessboard
///</summary>
/*bool DrawCube(Mat frame, Mat cameraMatrix) {
    
	//Find the chessboard points in webcam
	vector<Point2d> foundCorners;
	bool found = findChessboardCorners(frame, boardSize, foundCorners, CALIB_CB_FAST_CHECK);

	//If chessboard is not found, return false
	if (!found) {
		return false;
	}

	//Take the base square as the first 4 points of the cube
	vector<Point2d> cubePoints(8);
	cubePoints[0] = foundCorners[0];
	cubePoints[1] = foundCorners[2];
	cubePoints[2] = foundCorners[boardSize.width*2];
	cubePoints[3] = foundCorners[boardSize.width * 2 + 2];

	//Calculate the axes points with respect to camera coordinates
	vector<Point3d> cameraAxesPoints(4);
	getAxesInCameraCoordinates(cubePoints, cameraMatrix, cameraAxesPoints);

	//Calculate position of Z axis
	Mat tempCubePoint = Mat::zeros(3, 1, CV_64F);
	tempCubePoint.at<double>(0, 0) = cameraAxesPoints.at(3).x;
	tempCubePoint.at<double>(1, 0) = cameraAxesPoints.at(3).y;
	tempCubePoint.at<double>(2, 0) = cameraAxesPoints.at(3).z;

	Mat temp = cameraMatrix*tempCubePoint;
	cubePoints[4].x = temp.at<double>(0, 0);
	cubePoints[4].y = temp.at<double>(1, 0);


	//Draw the base of the cube
	line(frame, cubePoints[0], cubePoints[1], Scalar(0, 255, 0), 3);
	line(frame, cubePoints[0], cubePoints[2], Scalar(0, 255, 0), 3);
	line(frame, cubePoints[2], cubePoints[3], Scalar(0, 255, 0), 3);
	line(frame, cubePoints[1], cubePoints[3], Scalar(0, 255, 0), 3);
	line(frame, cubePoints[0], cubePoints[4], Scalar(0, 255, 0), 3);

	return true;
}

/// This function uses the intrinsic parameters and the image points of the corners to determine
/// the cube axes in camera-coordinates.  
void getAxesInCameraCoordinates(vector<Point2d> cubePoints, Mat cameraMatrix, vector<Point3d>& cameraAxesPoints)
{
	Mat cameraMatrixInverse;
	invert(cameraMatrix, cameraMatrixInverse);
	Mat tempHomogeneousCoordinates = Mat::zeros(3, 1, CV_64F);
	Mat tempMatAxisPoint = Mat::zeros(3, 1, CV_64F);

	//Calculate origin, X-axis and Y-axis in camera coordinates
	for (int i = 0; i < 3; i++) {
		tempHomogeneousCoordinates.at<double>(0, 0) = cubePoints[i].x;
		tempHomogeneousCoordinates.at<double>(1, 0) = cubePoints[i].y;
		tempHomogeneousCoordinates.at<double>(2, 0) = 1;

		tempMatAxisPoint = cameraMatrixInverse*tempHomogeneousCoordinates;
		cout << cameraMatrixInverse << endl;
		cameraAxesPoints[i] = Point3d(tempMatAxisPoint);
	}
	
	//Calculate Z-axis as a cross-product of X-axis and Y-axis
	Point3d xDirection = cameraAxesPoints[2] - cameraAxesPoints[0];
	Point3d yDirection = cameraAxesPoints[1] - cameraAxesPoints[0];
	cameraAxesPoints[3] = xDirection.cross(yDirection);
	cout << xDirection << endl;
	cout << yDirection << endl;
	//cout << cameraAxesPoints[3] << endl;
	

}*/

bool DrawCube(Mat frame, Mat cameraMatrix, Mat distCoeffs, vector<Point3f> boardPoints) {

	//Find the chessboard points in webcam
	vector<Point2d> foundCorners;
	bool found = findChessboardCorners(frame, boardSize, foundCorners, CALIB_CB_FAST_CHECK);

	//If chessboard is not found, return false
	if (!found) {
		return false;
	}

	//Get the extrinsic parameters
	Mat rvect;
	Mat tvec;
	solvePnP(boardPoints, foundCorners, cameraMatrix, distCoeffs, rvect, tvec);
	Mat extrinsicMatrix;
	Rodrigues(rvect, extrinsicMatrix);
	hconcat(extrinsicMatrix, tvec, extrinsicMatrix);

	//Calculate axes points in imageSpace
	vector<Point3d> axesWorldPoints(4);
	vector<Point2d> axesImagePoints(4);
	axesWorldPoints.push_back(Point3d(0, 0, 0));
	axesWorldPoints.push_back(Point3d(squareLength * 2, 0, 0));
	axesWorldPoints.push_back(Point3d(0, squareLength * 2, 0));
	axesWorldPoints.push_back(Point3d(0, 0, -squareLength * 2));
	projectPoints(axesWorldPoints, rvect, tvec, cameraMatrix, distCoeffs, axesImagePoints);
	
	cout << "AXES IMAGE POINTS" << axesImagePoints << endl;
	line(frame, axesImagePoints[0], axesImagePoints[5], Scalar(255, 0, 0), 2);
	line(frame, axesImagePoints[0], axesImagePoints[6], Scalar(0, 255, 0), 2);
	line(frame, axesImagePoints[0], axesImagePoints[7], Scalar(0, 0, 255), 2);


	//Define the cube point coordinates in object space
	Mat1d cubeWorldPoints(8,3,CV_64F);
	Mat1d ones = Mat::ones(8, 1, CV_64F);
	cubeWorldPoints.row(0) << 0, 0, 0;
	cubeWorldPoints.row(1) << squareLength * 2, 0, 0;
	cubeWorldPoints.row(2) << 0, squareLength * 2, 0;
	cubeWorldPoints.row(3) << 0, 0, squareLength * 2;
	cubeWorldPoints.row(4) << squareLength * 2, squareLength * 2, 0;
	cubeWorldPoints.row(5) << 0, squareLength * 2, squareLength * 2;
	cubeWorldPoints.row(6) << squareLength * 2, 0, squareLength * 2;
	cubeWorldPoints.row(7) << squareLength * 2, squareLength * 2, squareLength * 2;
	hconcat(cubeWorldPoints, ones, cubeWorldPoints);

	//Determine their representation in the image space explicitly: [x,y,1] = K*[R|t]*[X,Y,Z]
	Mat cubeImagePoints(8,3,CV_64F);
	Mat worldPoint;
	for (int i = 0; i < cubeImagePoints.cols; i++) {
		transpose(cubeWorldPoints.row(i), worldPoint);
		transpose(cameraMatrix*extrinsicMatrix*worldPoint, cubeImagePoints.row(i));
		//cout << cameraMatrix*extrinsicMatrix*worldPoint << endl;
	}
	cubeImagePoints.colRange(0, 1);
	
	/*
	//Draw them on the frame
	vector<Point2f> cubePoints(8);
	for (int i = 0; i < cubePoints.size(); i++) {
		cubePoints[i] = Point2f(cubeImagePoints.at<double>(i,0), cubeImagePoints.at<double>(i, 1));
	}
	Scalar color = Scalar(0, 255, 0);
	int thickness = 4;
	line(frame, cubePoints[0], cubePoints[1], color, thickness);
	line(frame, cubePoints[0], cubePoints[2], color, thickness);
	line(frame, cubePoints[0], cubePoints[3], color, thickness);*/


	return true;
}
