#include "infomcv_calibration.hpp"

void DrawCube();

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
    //Gets the list of images from the specified folder
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
        undistort(frame, frameUndistorted, cameraMatrix, distCoeffs);
        imshow("webcam", frame);
        imshow("webcam undistorted", frameUndistorted);
        
        if (waitKey(30) == 27) {
            return 0;
        }
    }
    
    return 0;
}

void DrawCube() {
    
}
