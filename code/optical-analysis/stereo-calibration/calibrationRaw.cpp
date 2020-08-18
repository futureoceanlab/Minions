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
bool checkImagePairExists();
string concatFileName(int camNo, int curFrame);
string concatFilePath(string dataDir, string fileName, string ext);
bool checkExistingCorners(string imgName, string ymlName,
                          vector<cv::Point2f> *corners, 
                          int lr, int nCorners);
bool autoFindCorners(cv::Mat img, cv::Size board_sz, 
                     vector<cv::Point2f> *corners, int lr);
double checkCalibrationQuality(vector<vector<cv::Point2f>> points[2],
                              cv::Mat M1, cv::Mat D1,  cv::Mat M2, cv::Mat D2,
                              cv::Mat F, int nFrames, int nCorners);
static void StereoCalib(int nImages, int nx, int ny,
                        string dataDir, bool useUncalibrated);
cv::Mat rectifyAndDisp(cv::Mat img1, cv::Mat img2, cv::Mat pair, 
                    cv::Mat map11, cv::Mat map12, cv::Mat map21, cv::Mat map22,
                    cv::Ptr<cv::StereoMatcher> stereo,
                    string rectName, string dispName);
void drawEpipolarLines(cv::Mat pair, cv::Mat img1, cv::Mat img2,
                      vector<vector<cv::Point2f>> points[2],
                      cv::Mat M1, cv::Mat D1,  cv::Mat M2, cv::Mat D2,
                      cv::Mat F, int nFrames, int nCorners, string fileName);

inline bool fileExists (const string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

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
    StereoCalib(nImages, board_w, board_h, dataDir, true);
  } 
  else 
  {
    help(argv);
  }
  return 0;
}

// The main calibration function that 
// 1. detects corners on chessboards
// 2. calibrate each camera's intrinsics
// 3. calibrate stereo camera extrinsics
// 4. rectifies images
// 5. finds correspondents
//
static void StereoCalib(int nImages, int nx, int ny,
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
  string intrinsicsYmlL = outputDir + "/intrinsics_1.yml";
  string intrinsicsYmlR = outputDir + "/intrinsics_2.yml";
  string extrinsicsYml = outputDir + "/extrinsics.yml";

  // actual square size
  int i, j, k, lr;
  int nCorners = nx * ny;
  cv::Size board_sz = cv::Size(nx, ny);
  vector<string> imageNames[2];
  vector<cv::Point3f> boardModel;
  vector<vector<cv::Point3f> > objectPoints;
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

  // 1. Extract corners out of each provided images of calibration target
  for (k = 1; k <= nImages; k++) 
  {
    bool pairExists  = checkImagePairExists();
    if (pairExists)
    {
      for (lr = 0; lr < nCam; lr++) {
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
        ofstream outputfile;
        outputfile.open(txtName);
        for (int c = 0; c < nCorners; c++)
        {
          // int new_c = (c % ny) * nx + c / nx;
          outputfile << corners[lr][c].x << "\t" << corners[lr][c].y << endl;
        }
        // new data, attempt to find corners/grids
        if (!found[lr]) 
        {
          cout << "hello" << endl;
          found[lr] = autoFindCorners(img, board_sz, corners, lr);
          cout << "HAH" << endl;
          if (found[lr]) {
            // convert to opencv matrix to save in yml
            cv::Mat mcorners(corners[lr]);

            cv::FileStorage fs(ymlName, cv::FileStorage::WRITE);
            fs << imgName << mcorners;
            fs.release();
          }
        }
        if (displayCorners) {
          // draw chessboard corners works for circle grids too
          cv::drawChessboardCorners(img, cv::Size(nx, ny), corners[lr], found[lr]);
          string chessboardName = outputDir + "/chessboard_" + to_string(k) + "_" + to_string(lr+1) + ".png";
          cv::imwrite(chessboardName, img);
        }
        // add corners when both left and rigth images were successfully
        // detected
        if (lr == 1 && found[0] && found[1]) {
          objectPoints.push_back(boardModel);
          points[0].push_back(corners[0]);
          points[1].push_back(corners[1]);
          for (int lr2 = 0; lr2 < 2; lr2++)
          {
            ofstream myfile;
            int	tix, new_tix, printf_ok, success = 0;
            char fileout[100];
            char dataOut[100];
            char file_base[100];
            sprintf(file_base, "cam%d.1", lr2+1);
            sprintf(fileout, "%s%04d%s", file_base, k, "_targets");

            myfile.open(fileout);
            sprintf(dataOut, "%d\n", nCorners);
            myfile << dataOut;
            for (tix = 0; tix < corners[lr2].size(); tix++)	{
              new_tix = ((ny - ((tix + 1) % ny)) % ny) * nx + tix / nx;
              // new_tix = (tix % nx)*ny + (nx - (tix / nx)) ;
              // cout << new_tix << endl;
              sprintf(dataOut, "%4d %9.4f %9.4f %5d %5d %5d %5d %5d\n",
                tix+1, corners[lr2][new_tix].x, corners[lr2][new_tix].y, 0, 0, 0, 100, tix+1);
              myfile << dataOut;
            }
            myfile.close();
          }
          cout << "ADDED" << endl;
        }
      }
    }
  }

  // 2. Calibrate single cameras first for better optimzation
  double initialIntrinsics[9] = {11363.6364, 0, 1296, 0, 11363.6364, 972, 0, 0, 1};
  cv::Mat M1 = cv::Mat(3, 3, CV_64F, initialIntrinsics);
  cv::Mat M2 = M1.clone();
  cv::Mat D1, D2, R, T, E, F;
  cout << "\nRunning single camera calibration ...\n";
  double error1 = cv::calibrateCamera(objectPoints, points[0],
                     imageSize, M1, D1,  R, T,
                     cv::CALIB_USE_INTRINSIC_GUESS |
                     cv::CALIB_FIX_ASPECT_RATIO |
                     cv::CALIB_ZERO_TANGENT_DIST);
  double error2 = cv::calibrateCamera(objectPoints, points[1],
                     imageSize, M2, D2,  R, T,
                     cv::CALIB_USE_INTRINSIC_GUESS |
                     cv::CALIB_FIX_ASPECT_RATIO |
                     cv::CALIB_ZERO_TANGENT_DIST);
  cout << M1 << endl;
  cout << error1 << endl;
  cout << error2 << endl;

  // save the intrinsic results
  cv::FileStorage fIntrinsicsL(intrinsicsYmlL, cv::FileStorage::WRITE);
  cv::FileStorage fIntrinsicsR(intrinsicsYmlR, cv::FileStorage::WRITE);
  fIntrinsicsL << "M" << M1;
  fIntrinsicsL << "D" << D1;
  fIntrinsicsL << "e" << error1;
  fIntrinsicsR << "M" << M2;
  fIntrinsicsR << "D" << D2;
  fIntrinsicsR << "e" << error2;
  cout << "Done single calibration!\n\n";

  fIntrinsicsL.release();
  fIntrinsicsR.release();

  // 3. stereo calibration
  //
  cout << "\nRunning stereo camera calibration ...\n";
  cv::stereoCalibrate(objectPoints, points[0], points[1], 
                    M1, D1, M2, D2, imageSize, R, T, E, F,
                    cv::CALIB_FIX_ASPECT_RATIO | 
                    cv::CALIB_ZERO_TANGENT_DIST |
                    cv::CALIB_FIX_INTRINSIC,
                    cv::TermCriteria(
                      cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100,1e-5)
                    );
  
  int nFrames = (int)objectPoints.size();
  double err = checkCalibrationQuality(points, M1, D1, M2, D2, F, nFrames, nCorners);
  cout << "avg err = " <<  err << endl;

  // Save the extrinsic calibration results
  cv::FileStorage fExtrinsics(extrinsicsYml, cv::FileStorage::WRITE);
  fExtrinsics << "R" << R;
  fExtrinsics << "T" << T;
  fExtrinsics << "E" << E;
  fExtrinsics << "F" << F;
  fExtrinsics << "e" << err;
  fExtrinsics.release();

  cout << "Stereo calibration Done!\n\n";

  // 4. rectification (BOUGUET'S METHOD)
  cv::Mat R1, R2, P1, P2, Q, map11, map12, map21, map22;
  stereoRectify(M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2,
                Q, 0);
  // Precompute maps for cvRemap()
  initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11,
                          map12);
  initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map21,
                          map22);
  
  // 5. Setup for finding stereo corrrespondences
  //
  cv::Mat pair;
  pair.create(imageSize.height, imageSize.width * 2, CV_8UC3);
  // cv::Ptr<cv::StereoMatcher> stereo = cv::StereoSGBM::create(
  //     -64, 128, 7, 600, 2400, 0, 4, 1, 150, 1, cv::StereoSGBM::MODE_HH);
  // cv::Ptr<cv::StereoMatcher> stereo = cv::StereoSGBM::create(15,   // minimum disparity
  //                                                          128,  // maximum disparity
  //                                                           13);  // block size
  cv::Ptr<cv::StereoMatcher> stereo = cv::StereoBM::create(128, 13);
  for (i = 0; i < 1 ; i++) { //nFrames; i++) {
    cv::Mat img1 = cv::imread(imageNames[0][i], 0);
    cv::Mat img2 = cv::imread(imageNames[1][i], 0);
    string rectName = "rectified_" + to_string(i) + ".png";
    string dispName = "disp_" + to_string(i) + ".png";
    string epipoleName = "epipole_" + to_string(i) + ".png";
    cv::Mat disp = rectifyAndDisp(img1, img2, pair,
                                  map11, map12, map21, map22,
                                  stereo, rectName, dispName);
    // cv::Mat depthMap(disp.size(), CV_8U);
    // cv::reprojectImageTo3D(disp, depthMap, Q);
    // // cv::Vec3f &point = XYZ..at<cv::Vec3f>(235, 374);
    // cout << depthMap.at<cv::Vec3f>(235, 374) << endl;
    // drawEpipolarLines(pair, img1, img2, points,M1, D1, M2, D2,
    //                   F, i, nCorners, epipoleName);
    // if ((cv::waitKey() & 255) == 27)
    //   break;
  }

  // cv::Mat targetL = cv::imread("../targetL3.png", 0);
  // cv::Mat targetR = cv::imread("../targetR3.png", 0); 
  // cv::GaussianBlur(targetL, targetL, cv::Size(7, 7), 0);
  // cv::GaussianBlur(targetR, targetR, cv::Size(7, 7), 0);

  // string rectName = "targetRect3.png";
  // string dispName = "targetDisp_bm31.png";
  // cv::Mat disp = rectifyAndDisp(targetL, targetR, pair,
  //               map11, map12, map21, map22,
  //               stereo, rectName, dispName);
                
  // cv::Mat depthMap(disp.size(), CV_8U);
  // cv::reprojectImageTo3D(disp, depthMap, Q);
  // cout << depthMap.at<cv::Vec3f>(1899, 651) << endl; //235, 374) << endl;

  // cout << cv::mean(depthMap) << endl;

  // cout << "done" <<endl;
}

cv::Mat rectifyAndDisp(cv::Mat img1, cv::Mat img2, cv::Mat pair, 
                    cv::Mat map11, cv::Mat map12, cv::Mat map21, cv::Mat map22,
                    cv::Ptr<cv::StereoMatcher> stereo,
                    string rectName, string dispName) 
{
    cv::Mat img1r, img2r, disp;
    if (img1.empty() || img2.empty())
      return disp;
    cv::remap(img1, img1r, map11, map12, cv::INTER_LINEAR);
    cv::remap(img2, img2r, map21, map22, cv::INTER_LINEAR);
    // paste two rectified images side-by-side 
    cv::Mat part = pair.colRange(0, img1.cols);
    cvtColor(img1r, part, cv::COLOR_GRAY2BGR);
    part = pair.colRange(img1.cols, img1.cols * 2);
    cvtColor(img2r, part, cv::COLOR_GRAY2BGR);
    // and draw horizontal epipolar lines
    for (int j = 0; j < img1.cols; j += 16)
      cv::line(pair, cv::Point(0, j), cv::Point(img1.cols * 2, j),
                cv::Scalar(0, 255, 0));
    // compute disparity image using rectified images
    stereo->compute(img1r, img2r, disp);
    cv::imwrite(rectName, pair);
    cv::imwrite("rect_1_10.png", img1r);
    cv::imwrite("rect_2_10.png", img2r);
    cv::imwrite(dispName, disp);
    return disp;
}

void help(char *argv[]) {
  cout
      << "\n\nMinions: Stereo calibration, rectification, and "
         "correspondence"
      << "\n    Parameters: number of images, board width, board height, image directory"
      << "\n" << endl;
}

string concatFileName(int camNo, int curFrame)
{
    string fileName = "cam_" + std::to_string(camNo) + "_" + std::to_string(curFrame);
    return fileName;
}

string concatFilePath(string dataDir, string fileName, string ext)
{
  string filePath = dataDir + "/" + fileName + "." + ext;
  return filePath;
}

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
  cout << "DUD" << endl;
  return hasData;
}

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
 
// Draw detected blobs as red circles.
// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
  cv::Mat im_with_keypoints;
  cv::drawKeypoints( img, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  cout << keypoints.size() << endl;
// // Show blobs
cv::imwrite("keypoints.png", im_with_keypoints );
//   // cout << board_sz << endl;
  for (int alpha = 1; alpha <= 50; alpha++) 
  {
    // contrast affects the detection quality. 
    // slowly raise the contrast higher in every iteration
    cv::Mat timg;
    img.convertTo(timg, -1, alpha*0.25f, beta);
    // foundGrid = cv::findChessboardCorners(timg, board_sz, corners[lr]);//,
                                          // cv::CALIB_CB_FAST_CHECK);
    // below for ccircular grid calibration target
    foundGrid = cv::findCirclesGrid(timg, board_sz,
                                    corners[lr],
                                    cv::CALIB_CB_SYMMETRIC_GRID,
                                    blobDetector
                                    );
    cout << alpha << endl;
    // cout << corners[lr] << endl;

    if(foundGrid)
    {
 
    //   // we will do more accurate corner detection using SubPix
    //   cout << alpha << endl;
    //   cornerSubPix(timg, corners[lr], cv::Size(10, 10), cv::Size(-1, -1),
    //     cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-2));
      break;
    }
  }
  return foundGrid;
}


// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
double checkCalibrationQuality(vector<vector<cv::Point2f>> points[2],
                              cv::Mat M1, cv::Mat D1,  cv::Mat M2, cv::Mat D2,
                              cv::Mat F, int nFrames, int nCorners)
{ 
  vector<cv::Point3f> lines[2];
  double avgErr = 0;
  for (int i = 0; i < nFrames; i++) {
    vector<cv::Point2f> &pt0 = points[0][i];
    vector<cv::Point2f> &pt1 = points[1][i];
    cv::undistortPoints(pt0, pt0, M1, D1, cv::Mat(), M1);
    cv::undistortPoints(pt1, pt1, M2, D2, cv::Mat(), M2);
    cv::computeCorrespondEpilines(pt0, 1, F, lines[0]);
    cv::computeCorrespondEpilines(pt1, 2, F, lines[1]);

    for (int j = 0; j < nCorners; j++) {
      double err = fabs(pt0[j].x * lines[1][j].x + pt0[j].y * lines[1][j].y +
                        lines[1][j].z) +
                   fabs(pt1[j].x * lines[0][j].x + pt1[j].y * lines[0][j].y +
                        lines[0][j].z);
      avgErr += err;
    }
  }
  return avgErr / (nFrames * nCorners);
}

bool checkImagePairExists()
{
  return true;
}

void drawEpipolarLines(cv::Mat pair, cv::Mat img1, cv::Mat img2,
                      vector<vector<cv::Point2f>> points[2],
                      cv::Mat M1, cv::Mat D1,  cv::Mat M2, cv::Mat D2,
                      cv::Mat F, int nthFrame, int nCorners, string fileName)
{
  cv::Mat part1 = pair.colRange(0, img1.cols);
  cvtColor(img1, part1, cv::COLOR_GRAY2BGR);
  cv::Mat part2 = pair.colRange(img1.cols, img1.cols * 2);
  cvtColor(img2, part2, cv::COLOR_GRAY2BGR);

  vector<cv::Point3f> lines[2];
  cv::RNG rng(0);
  vector<cv::Point2f> &pt0 = points[0][nthFrame];
  vector<cv::Point2f> &pt1 = points[1][nthFrame];
  cv::undistortPoints(pt0, pt0, M1, D1, cv::Mat(), M1);
  cv::undistortPoints(pt1, pt1, M2, D2, cv::Mat(), M2);
  cv::computeCorrespondEpilines(pt0, 1, F, lines[0]);
  cv::computeCorrespondEpilines(pt1, 2, F, lines[1]);

  for (int j = 0; j < nCorners; j+=25) 
  {
    cv::Scalar color(rng(256),rng(256),rng(256));
    cv::line(part2,
      cv::Point(0,-lines[0][j].z/lines[0][j].y),
      cv::Point(img1.cols,-(lines[0][j].z + lines[0][j].x*img1.cols)/lines[0][j].y),
      color);
    cv::circle(part1, pt0[j], 3, color, -1, cv::LINE_AA);
  }
  cv::imwrite(fileName, pair);
}
