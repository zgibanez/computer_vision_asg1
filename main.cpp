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
    
    bool successfulCalibration = calibrate(0, cameraMatrix, distCoeffs, rvecs, tvecs);
    if (!successfulCalibration) {
        cout << "Camera could not be calibrated" << endl;
        return -1;
    }
    
    cout << "CAMERA MATRIX" << cameraMatrix << endl;
    cout << "DISTORTION COEFFICIENTS" << distCoeffs << endl;
    
    VideoCapture vid(0);
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
        //imshow("webcam", frame);
        
        bool smt = DrawCube(frameUndistorted, cameraMatrix, distCoeffs, boardPoints);
        imshow("webcam undistorted", frameUndistorted);
        
        /*Mat debug = imread("./data/1.jpg");
        bool smt = DrawCube(debug, cameraMatrix, distCoeffs, boardPoints);
        imshow("Debug", debug);*/
        
        if (waitKey(30) == 27) {
            return 0;
        }
    }
    
    return 0;
}

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
    vector<Point3d> axesWorldPoints(0);
    vector<Point2d> axesImagePoints(0);
    axesWorldPoints.push_back(Point3d(0, 0, 0));
    axesWorldPoints.push_back(Point3d(squareLength * 4, 0, 0));
    axesWorldPoints.push_back(Point3d(0, squareLength * 4, 0));
    axesWorldPoints.push_back(Point3d(0, 0, -squareLength * 4));
    //Without extra points
    projectPoints(axesWorldPoints, rvect, tvec, cameraMatrix, distCoeffs, axesImagePoints);
    
    cout << "AXES IMAGE POINTS" << axesImagePoints << endl;
    line(frame, axesImagePoints[0], axesImagePoints[1], Scalar(255, 0, 0), 4);
    line(frame, axesImagePoints[0], axesImagePoints[2], Scalar(0, 255, 0), 4);
    line(frame, axesImagePoints[0], axesImagePoints[3], Scalar(0, 0, 255), 4);
    
    vector<Point3d> cubeWorldPoints(0);
    vector<Point2d> cubeImagePoints(0);
    cubeWorldPoints.push_back(Point3d(0, 0, 0));
    cubeWorldPoints.push_back(Point3d(squareLength * 2, 0, 0));
    cubeWorldPoints.push_back(Point3d(0, squareLength * 2, 0));
    cubeWorldPoints.push_back(Point3d(0, 0, -squareLength * 2));
    cubeWorldPoints.push_back(Point3d(squareLength * 2, squareLength * 2, 0));
    cubeWorldPoints.push_back(Point3d(0, squareLength * 2, -squareLength * 2));
    cubeWorldPoints.push_back(Point3d(squareLength * 2, 0, -squareLength * 2));
    cubeWorldPoints.push_back(Point3d(squareLength * 2, squareLength * 2, -squareLength * 2));
    
    //Without extra points
    projectPoints(cubeWorldPoints, rvect, tvec, cameraMatrix, distCoeffs, cubeImagePoints);
    
    //********CUBE DRAWING
    
    //Define the cube point coordinates in object space
    /*Mat1d cubeWorldPoints(8,3,CV_64F);
    Mat1d ones = Mat::ones(8, 1, CV_64F);
    //origin
    cubeWorldPoints.row(0) << 0, 0, 0;
    //X = 1
    cubeWorldPoints.row(1) << squareLength * 2, 0, 0;
    //Y = 1
    cubeWorldPoints.row(2) << 0, squareLength * 2, 0;
    //Z = 1
    cubeWorldPoints.row(3) << 0, 0, squareLength * 2;
    //x = Y = 1
    cubeWorldPoints.row(4) << squareLength * 2, squareLength * 2, 0;
    cubeWorldPoints.row(5) << 0, squareLength * 2, squareLength * 2;
    cubeWorldPoints.row(6) << squareLength * 2, 0, squareLength * 2;
    cubeWorldPoints.row(7) << squareLength * 2, squareLength * 2, squareLength * 2;
    //hconcat(cubeWorldPoints, ones, cubeWorldPoints);
    */
    /*//Determine their representation in the image space explicitly: [x,y,1] = K*[R|t]*[X,Y,Z]
    Mat cubeImagePoints(8,3,CV_64F);
    Mat worldPoint;
    for (int i = 0; i < cubeImagePoints.rows; i++) {
        transpose(cubeWorldPoints.row(i), worldPoint);
        transpose(cameraMatrix*extrinsicMatrix*worldPoint, cubeImagePoints.row(i));
        cout << "Mult" << cameraMatrix*extrinsicMatrix*worldPoint << endl;
    }
    //Ignore the homogenous element
    cubeImagePoints.colRange(0, 1);
    //cout << "CubeImagePoints: " << cubeImagePoints << endl;*/
    
    
    
     /*//Draw them on the frame
     vector<Point2d> cubePoints(8);
     for (int i = 0; i < cubePoints.size(); i++) {
         cubePoints[i] = Point2f(cubeImagePoints.at<double>(i,0), cubeImagePoints.at<double>(i, 1));
     }
    //cout << cubePoints << endl;*/
    vector<Point2d> cubePoints = cubeImagePoints;
     Scalar color = Scalar(0, 255, 255);
     int thickness = 4;
    //Cara 1
    line(frame, cubePoints[0], cubePoints[1], color, thickness);
    line(frame, cubePoints[1], cubePoints[6], color, thickness);
    line(frame, cubePoints[6], cubePoints[3], color, thickness);
    line(frame, cubePoints[3], cubePoints[0], color, thickness);
    //Face 2
    line(frame, cubePoints[1], cubePoints[4], color, thickness);
    line(frame, cubePoints[4], cubePoints[7], color, thickness);
    line(frame, cubePoints[7], cubePoints[6], color, thickness);
    //Face 3
    line(frame, cubePoints[4], cubePoints[2], color, thickness);
    line(frame, cubePoints[2], cubePoints[5], color, thickness);
    line(frame, cubePoints[5], cubePoints[7], color, thickness);
    //Face 4
    line(frame, cubePoints[3], cubePoints[5], color, thickness);
    line(frame, cubePoints[2], cubePoints[0], color, thickness);
    
    
    return true;
}
