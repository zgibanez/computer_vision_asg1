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
vector<Mat> ExtractCornersFromFiles(vector<Mat> imageList, vector<vector<Point2f>>& foundCorners, bool show)
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





