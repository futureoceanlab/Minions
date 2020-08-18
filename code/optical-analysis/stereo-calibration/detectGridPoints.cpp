/**
 * detectGridPoints.cpp 
 * 
 * author: Junsu Jang (junsu.jang94@gmail.com)
 * date: Aug 18th, 2020
 * 
 * Description:
 *  Detects circular grid points on the calibration target and save them 
 *  in a file.
 * 
 */


#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <bits/stdc++.h> 

using namespace std;

void help(char *argv[]);
string concatFileName(int camNo, int curFrame);
string concatFilePath(string dataDir, string fileName, string ext);
bool checkExistingCorners(string imgName, string ymlName,
                          vector<cv::Point2f> *corners, 
                          int lr, int nCorners);
bool autoFindCorners(cv::Mat img, cv::Size board_sz, 
                     vector<cv::Point2f> *corners, int lr);
static void StereoCalib(int nImages, int nx, int ny,
                        string dataDir, bool useUncalibrated);

/**
 * inline function for checking whether a file exists or not
 */
inline bool fileExists (const string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

/**
 * main: entrance to the software. Parse input arguments and serve help 
 * instructions if necessary
 */
int main(int argc, char **argv) 
{
  int board_w = 9, board_h = 5;
  int nImages;
  string dataDir;
  
  if (argc == 5) 
  {
    nImages = atoi(argv[1]);
    board_w = atoi(argv[2]);
    board_h = atoi(argv[3]);
    dataDir = argv[4];
    struct stat info;
    if( stat( dataDir.c_str(), &info ) != 0 )
    {
      cout << "cannot access " << dataDir << endl;
      return -1;
    }
    else if( (info.st_mode & S_IFDIR) == 0)
    {
      cout << dataDir << "is not a directory" << endl;
      return -1;
    }
    detectGridPoints(nImages, board_w, board_h, dataDir, true);
  } 
  else 
  {
    help(argv);
  }
  return 0;
}

/**
 * detectGridPoints: detects and save the grid points in each images given
 */
static void detectGridPoints(int nImages, int nx, int ny,
                        string dataDir,
                        bool useUncalibrated) {
  bool displayCorners = true;
  const float squareSize = 1.0f;
  int nCam = 2;
  struct stat info;
  string outputDir = dataDir + "/output";
  if( stat( outputDir.c_str(), &info ) != 0 )
  {
    mkdir(outputDir.c_str(), 0777);
  } 
  // actual square size
  int i, j, k, lr;
  int nCorners = nx * ny;
  cv::Size board_sz = cv::Size(nx, ny);
  vector<string> imageNames[2];
  vector<cv::Point3f> boardModel;
  vector<vector<cv::Point2f> > points[2];
  vector<cv::Point2f> corners[2];
  bool found[2] = {false, false};
  cv::Size imageSize;

  // create board model for chessboard detection
  for (i = 0; i < ny; i++) {
    for (j = 0; j < nx; j++) {
      boardModel.push_back(cv::Point3f((float)(i * squareSize), (float)(j * squareSize), 0.f));
    }
  }

  // Extract corners out of each provided images of calibration target
  for (k = 1; k <= nImages; k++) 
  {
    for (lr = 0; lr < nCam; lr++) 
    {
      string imgName = concatFileName(lr+1, k);
      string imgPath = concatFilePath(dataDir, imgName, "png");

      cv::Mat img = cv::imread(imgPath, cv::IMREAD_GRAYSCALE);
      cv::flip(img, img, -1);
      if (img.empty()) 
        break;

      string ymlName = outputDir + "/" + imgName + ".yml";
      string txtName = outputDir + "/" + imgName + ".txt";
      imageNames[lr].push_back(imgPath);
      imageSize = img.size();
                      cout << imgPath << endl;

      // check if corners were already detected before and saved in yml files
      found[lr] = checkExistingCorners(imgName, ymlName, corners, lr, nCorners);
      // new data, attempt to find corners/grids
      if (!found[lr]) 
      {
        found[lr] = autoFindCorners(img, board_sz, corners, lr);
        if (found[lr]) {
          // corners have been found
          // convert to opencv matrix to save in yml
          cv::Mat mcorners(corners[lr]);

          cv::FileStorage fs(ymlName, cv::FileStorage::WRITE);
          fs << imgName << mcorners;
          fs.release();
        }
      }
      // corners have been found from either an existing data or 
      // autoFindCorners. Now we save the result in .txt, which will be 
      // processed by the MATLAB calibration file
      if (found[lr])
      {
        ofstream outputfile;
        outputfile.open(txtName);
        for (int c = 0; c < nCorners; c++)
        {
          outputfile << corners[lr][c].x << "\t" << corners[lr][c].y << endl;
        }
      }
      // Save the corners found for visual feedback
      if (displayCorners) {
        // draw chessboard corners works for circle grids too
        cv::drawChessboardCorners(img, cv::Size(nx, ny), corners[lr], found[lr]);
        string chessboardName = outputDir + "/chessboard_" + to_string(k) + "_" + to_string(lr+1) + ".png";
        cv::imwrite(chessboardName, img);
      }
    }
  }
}

/**
 * help: help instructions for terminal commmand
 */ 
void help(char *argv[]) {
  cout
      << "\n\nMinions: Stereo calibration, rectification, and "
         "correspondence"
      << "\n    Parameters: number of images, board width, board height, image directory"
      << "\n" << endl;
}

/**
 *  concatFileName: file name generator 
 */
string concatFileName(int camNo, int curFrame)
{
    string fileName = "cam_" + std::to_string(camNo) + "_" + std::to_string(curFrame);
    return fileName;
}

/**
 * concatFilePath: path generator
 */
string concatFilePath(string dataDir, string fileName, string ext)
{
  string filePath = dataDir + "/" + fileName + "." + ext;
  return filePath;
}

/**
 * checkExistingCorners: Check if there is an existing yml file that contains
 * the grid points of the image of interest. If there is, we copy over the 
 * grid points into 'corners'
 */
bool checkExistingCorners(string imgName, string ymlName,
                          vector<cv::Point2f> *corners, 
                          int lr, int nCorners)
{
  bool hasData = false;

  if (fileExists(ymlName)) { 
    cv::FileStorage fs(ymlName, cv::FileStorage::READ);
    if (fs.isOpened()) {
      cv::Mat mcorners;
      fs[imgName] >> mcorners;
      cout << mcorners.size() << endl;
      if (mcorners.cols == 1 && mcorners.rows == nCorners ) {
        mcorners = mcorners.reshape(2, nCorners);
        corners[lr] = mcorners.clone();
        hasData = true;
      }
    } 
    fs.release();
  }
  return hasData;
}

/**
 * autoFindCorners: circular grid detector application. 
 */
bool autoFindCorners(cv::Mat img, cv::Size board_sz, 
                     vector<cv::Point2f> *corners, int lr)
{
  bool foundGrid = false;
  int beta = 0;
  cv::SimpleBlobDetector::Params params;

  // Change thresholds
  params.minThreshold = 50;
  params.maxThreshold = 180;

  // Filter by Area.
  params.filterByArea = true;
  // params.minArea = 10;

  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.1;

  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.87;

  // Filter by Inertia
  params.filterByInertia = true;
  params.minInertiaRatio = 0.01;

  // params.maxArea = 1e4;
  // params.minArea = 1e3;
  cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);
  std::vector<cv::KeyPoint> keypoints;
  blobDetector->detect( img, keypoints);
 
  // Uncomment below to see the blob detection performance
  /*
  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
  cv::Mat im_with_keypoints;
  cv::drawKeypoints( img, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  cout << keypoints.size() << endl;
  // Show blobs
  cv::imwrite("keypoints.png", im_with_keypoints );
  */

  // contrast affects the detection quality. 
  // slowly raise the contrast higher in every iteration
  for (int alpha = 1; alpha <= 50; alpha++) 
  {
    cv::Mat timg;
    img.convertTo(timg, -1, alpha*0.25f, beta);
    // below for circular grid calibration target
    foundGrid = cv::findCirclesGrid(timg, board_sz,
                                    corners[lr],
                                    cv::CALIB_CB_SYMMETRIC_GRID,
                                    blobDetector
                                    );
  }
  return foundGrid;
}
