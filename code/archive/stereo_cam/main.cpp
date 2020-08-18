//////////////////////////////////////////////////////////////////
/*
Tcam Software Trigger
This sample shows, how to trigger the camera by software and use a callback for image handling.

Prerequisits
It uses the the examples/cpp/common/tcamcamera.cpp and .h files of the *tiscamera* repository as wrapper around the
GStreamer code and property handling. Adapt the CMakeList.txt accordingly.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include <unistd.h>

#include "tcamcamera.h"
#include <unistd.h>

#include "opencv2/opencv.hpp"



using namespace gsttcam;
using namespace std;

// Create a custom data structure to be passed to the callback function. 
typedef struct
{
    int camera;
    int ImageCounter;
    bool SaveNextImage;
    bool busy;
   	cv::Mat frame; 
} CUSTOMDATA;

string concatFileName(int camNo, int curFrame);

string curPath;


////////////////////////////////////////////////////////////////////
// List available properties helper function.
void ListProperties(TcamCamera &cam)
{
    // Get a list of all supported properties and print it out
    auto properties = cam.get_camera_property_list();
    std::cout << "Properties:" << std::endl;
    for(auto &prop : properties)
    {
        std::cout << prop->to_string() << std::endl;
    }
}

////////////////////////////////////////////////////////////////////
// Callback called for new images by the internal appsink
GstFlowReturn new_frame_cb(GstAppSink *appsink, gpointer data)
{
    int width, height ;
    const GstStructure *str;

    // Cast gpointer to CUSTOMDATA*
    CUSTOMDATA *pCustomData = (CUSTOMDATA*)data;
    if( !pCustomData->SaveNextImage)
        return GST_FLOW_OK;


    // The following lines demonstrate, how to acces the image
    // data in the GstSample.
    GstSample *sample = gst_app_sink_pull_sample(appsink);

    GstBuffer *buffer = gst_sample_get_buffer(sample);

    GstMapInfo info;

    gst_buffer_map(buffer, &info, GST_MAP_READ);
    
    if (info.data != NULL) 
    {
        // info.data contains the image data as blob of unsigned char 

        GstCaps *caps = gst_sample_get_caps(sample);
        // Get a string containg the pixel format, width and height of the image        
        str = gst_caps_get_structure (caps, 0);    

        if( strcmp( gst_structure_get_string (str, "format"),"GRAY8") == 0)  
        {
            // Now query the width and height of the image
            gst_structure_get_int (str, "width", &width);
            gst_structure_get_int (str, "height", &height);
            // Create a cv::Mat, copy image data into that and save the image.
            pCustomData->frame.create(height,width,CV_8U);
            memcpy( pCustomData->frame.data, info.data, width*height);
            string fileDir = concatFileName(pCustomData->camera, pCustomData->ImageCounter);
            cv::imwrite(fileDir,pCustomData->frame);

            // Only capture photo when we reviewed current image
            pCustomData->SaveNextImage = false;
        }
    }
    
    // Calling Unref is important!
    gst_buffer_unmap (buffer, &info);
    gst_sample_unref(sample);

    // Set our flag of new image to true, so our main thread knows about a new image.
    return GST_FLOW_OK;
}


void processImage(string imgPaths[2])
{
    cv::Mat imgs[2];
    int width=500, height=500;
    for (int i = 0; i < 2; i++) 
    {
        cv::Mat img = cv::imread(imgPaths[i], CV_8U);
        int xl = (img.cols/2)-(width/2);
        int yl = (img.rows/2)-(height/2);
        cv::Rect rect(xl, yl, width, height);

        cv::rectangle(img, rect, cv::Scalar(255), 1);
        cv::circle(img, cv::Point(width/2, height/2), 5, cv::Scalar(255), 1);

        cv::Mat img32;
        img(rect).convertTo(img32, CV_32F);
        cv::GaussianBlur( img32, img32, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
        cv::Mat dx, dy;
        cv::Sobel( img32, dx, -1, 2, 0, 3 );
        cv::Sobel( img32, dy, -1, 0, 2, 3 );
        cv::magnitude( dx, dy, img32 );
        // cv::Mat cont_output;
        img32.convertTo(img32, CV_8U);

        double m = cv::mean(img32)[0];

        // Display the image with mean information
        cv::putText(img32, 
                    std::to_string(m),
                    cv::Point(5,40), // Coordinates
                    cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                    1.0, // Scale. 2.0 = 2x bigger
                    cv::Scalar(255,255,255), // BGR Color
                    1, // Line Thickness (Optional)
                    2); // Anti-alias (Optional)
        imgs[i] = img32.clone();
    }
    cv::Mat matDst(cv::Size(width*2,height), CV_8U, cv::Scalar::all(0));
    cv::Mat matRoi = matDst(cv::Rect(0,0,width,height));
    imgs[0].copyTo(matRoi);
    matRoi = matDst(cv::Rect(width,0,width,height));
    imgs[1].copyTo(matRoi);

    cv::imshow("Focus In Process", matDst);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

bool is_file_exist(string fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

string concatFileName(int camNo, int curFrame)
{
    string fileName = curPath + "/cam_" + std::to_string(camNo) + "_" + std::to_string(curFrame) + ".jpg";
    return fileName;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    char cCurrentPath[FILENAME_MAX];
    if (!getcwd(cCurrentPath, sizeof(cCurrentPath)))
    {
        return errno;
    }
    curPath = string(cCurrentPath);

    gst_init(&argc, &argv);
    string SN[2] = {"15410110", "41810422"};

    // Declare custom data structure for the callback
    CUSTOMDATA CustomData1, CustomData2;
    CustomData1.camera = 1;
    CustomData1.ImageCounter = 0;
    CustomData1.SaveNextImage = false;
    CustomData2.camera = 2;
    CustomData2.ImageCounter = 0;
    CustomData2.SaveNextImage = false;

    
    printf("Tcam OpenCV Image Sample\n");

    // Open camera by serial number
    TcamCamera cam1(SN[0]);
    TcamCamera cam2(SN[1]);
    
    // Set video format, resolution and frame rate
    // cam1.set_capture_format("GRAY8", FrameSize{2592,1944}, FrameRate{4, 1});
    // cam2.set_capture_format("GRAY8", FrameSize{2592,1944}, FrameRate{4, 1});
    cam1.set_capture_format("GRAY8", FrameSize{2592,1944}, FrameRate{4, 1});
    cam2.set_capture_format("GRAY8", FrameSize{2592,1944}, FrameRate{4, 1});

    // Comment following line, if no live video display is wanted.
    cam1.enable_video_display(gst_element_factory_make("ximagesink", NULL));

    // Register a callback to be called for each new frame
    cam1.set_new_frame_callback(new_frame_cb, &CustomData1);
    cam2.set_new_frame_callback(new_frame_cb, &CustomData2);

    // std::shared_ptr<Property> ExposureAuto1 = NULL;
    // std::shared_ptr<Property> ExposureAuto2 = NULL;
    // std::shared_ptr<Property> ExposureValue1 = NULL;
    // std::shared_ptr<Property> ExposureValue2 = NULL;
    // try
    // {
    //     ExposureAuto1 = cam1.get_property("Exposure Auto");
    //     ExposureAuto2 = cam2.get_property("Exposure Auto");

    // }
    // catch(std::exception &ex)    
    // {
    //     printf("Error %s : %s\n",ex.what(), "Exposure Automatic");
    // }

    // try
    // {
    //     ExposureValue1 = cam1.get_property("Exposure");
    //     ExposureValue2 = cam2.get_property("Exposure");
    // }
    // catch(std::exception &ex)    
    // {
    //     printf("Error %s : %s\n",ex.what(), "Exposure Value");
    // }
    // // Disable automatics, so the property values can be set 
    // if( (ExposureAuto1 != NULL) && (ExposureAuto2 != NULL)){
    //     ExposureAuto1->set(cam1,0);
    //     ExposureAuto2->set(cam2,0);
    // }    // set a value
    // if((ExposureValue1 != NULL) && (ExposureValue2 != NULL)) {
    //     ExposureValue1->set(cam1,40000);
    //     ExposureValue2->set(cam2,40000);
    // }

    // Start the camera
    cam1.start();
    cam2.start();
    
    // Uncomment following line, if properties shall be listed. Many of the
    // properties that are done in software are available after the stream 
    // has started. Focus Auto is one of them.
    ListProperties(cam1);

    for( int i = 0; i< 1000000; i++)
    {
        CustomData1.SaveNextImage = true;
        CustomData2.SaveNextImage = true;
        CustomData1.ImageCounter++;
        CustomData2.ImageCounter++;

        // Check if there is a file written with the current index
        string fileName1 = concatFileName(1, CustomData1.ImageCounter);
        string fileName2 = concatFileName(2, CustomData2.ImageCounter);
        // cout << "Main1: " << fileName1 << endl;
        // cout << "Main2: " << fileName1 << endl;
        
        while (!(is_file_exist(fileName1) && is_file_exist(fileName2))) {};

        string imgPaths[2] = {fileName1, fileName2};
        processImage(imgPaths);

        // Done with processing, display the image


        sleep(1);
    }


    // Simple implementation of "getch()"
    printf("Press Enter to end the program");
    char dummyvalue[10];
    scanf("%c",dummyvalue);

    cam1.stop();
    cam2.stop();
    return 0;
}
