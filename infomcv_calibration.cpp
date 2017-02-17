//
//  infomcv_calibration.cpp
//  Assignment_1
//

#include "infomcv_calibration.hpp"

string parametersFileName = "cameraParameters";

///<summary>
///Retrieves all the files in the "folder" path and stores them in a vector of images.
///</summary>
vector<Mat> GetImageListFromFolder(string folder)
{
    //Variables
    //Vector of images
    vector<Mat> imagesFromFolder;
    //Vector of file names (strings)
    vector<String> fileNames;
    //Completes the fileNames with the folder name
    glob(folder, fileNames, false);
    //Get the number of pictures
    size_t fileNumber = fileNames.size();
    //Retrieve each picture and store it into the Mat imagesFrom Folder
    for (size_t i = 0; i < fileNumber; i++)
    {
        imagesFromFolder.push_back(imread(fileNames[i]));
    }
    //Return the vector of images
    return imagesFromFolder;
}

///<summary>
///Retrieves a list of images for which the chessboard corners have been found.
///</summary>
void ExtractCornersFromFiles(vector<Mat> imageList, vector<vector<Point2f>>& foundCorners, bool show)
{
    //Vector of images
    vector<Mat> imagesWithCorners;
    
    //
    for (int i = 0; i < imageList.size(); i++)
    {
        //Creates a vector of 2D points
        vector<Point2f> tempCorners;
        //Automatic function that finds the chess board corners, and store them in the vector
        //This function takes a picture, the boardSize, the vector to store the points, and flag
        bool success = findChessboardCorners(imageList.at(i), boardSize, tempCorners, CV_CALIB_CB_ADAPTIVE_THRESH); //USE CALIB_FAST_CHECK for real-time
        //If the points are found
        if (success) {
            //The corners are pushed into 
            foundCorners.push_back(tempCorners);
            drawChessboardCorners(imageList.at(i), boardSize, tempCorners, success);
            imagesWithCorners.push_back(imageList.at(i));
        }
    }
    
    //show images for debugging purposes
    if (show) {
        
        size_t count = imagesWithCorners.size();
        for (size_t i = 0; i<count; i++)
        {
            imshow("Photos", imagesWithCorners[i]);
            waitKey(0);
        }
    }
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

bool calibrate(int camera,Mat& cameraMatrix, Mat& distCoeffs, Mat& rvecs, Mat& tvecs)
{
    //Try to read the calibration file
    ostringstream oss;
    oss<< "camera_" << camera << "_parameters.xml";
    string paramFile = oss.str() ;
    
    if (readCalibrationParameters(paramFile, cameraMatrix, distCoeffs, rvecs, tvecs))
    {
        cout << "Parameters read from file: " << paramFile << endl;
        return true;
    }
    else
        cout << "No parameters file found for camera" << camera << ". Calibrating..." << endl;
    
    //Gets the list of images from the specified folder
    vector<Mat> imagesFromFolder = GetImageListFromFolder();
    
    if (imagesFromFolder.size() < 3)
    {
        cout << "Not enough sources to calibrate. " << endl;
        return false;
    }
    
    vector<vector<Point2f>> foundCorners;
    
    ExtractCornersFromFiles(imagesFromFolder, foundCorners);
    
    vector<Point3f> knownChessboardCoordinates = getKnownBoardPosition(boardSize, squareLength);
    
    vector<vector<Point3f>> worldCoordinatesCorners(1);
    worldCoordinatesCorners[0] = knownChessboardCoordinates;
    worldCoordinatesCorners.resize(foundCorners.size(), worldCoordinatesCorners[0]);
    
    calibrateCamera(worldCoordinatesCorners, foundCorners, boardSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    
    //Save parameters to file
    saveCalibrationParameters(paramFile, cameraMatrix, distCoeffs, rvecs, tvecs);
    
    return true;
}

bool readCalibrationParameters(string paramFile, Mat& cameraMatrix, Mat& distCoeffs, Mat& rvecs, Mat& tvecs)
{
    FileStorage fs(paramFile, FileStorage::READ);
    
    if(!fs.isOpened())
    {
        return false;
    }
    
    fs["K"] >> cameraMatrix;
    fs["dist"] >> distCoeffs;
    fs["rvecs"] >> rvecs;
    fs["tvecs"] >> tvecs;
    
    cout << parametersFileName << " successfully read" << endl;
    return true;
}

bool saveCalibrationParameters(string parametersFileName, Mat& cameraMatrix, Mat& distCoeffs, Mat& rvecs, Mat& tvecs)
{
    FileStorage fs(parametersFileName, FileStorage::WRITE);
    
    fs << "K" << cameraMatrix;
    fs << "dist" << distCoeffs;
    fs << "rvecs" << rvecs;
    fs << "tvecs" << tvecs;
    
    cout << parametersFileName << " successfully saved" << endl;
    
    return true;
}





