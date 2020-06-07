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

#include "tcamcamera.h"
#include <unistd.h>

#include "Image.h"

using namespace gsttcam;
using namespace std;

// Create a custom data structure to be passed to the callback function. 
typedef struct
{
    int ImageCounter;
    bool SaveNextImage;
    bool busy;
   	// cv::Mat frame; 
} CUSTOMDATA;

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
    pCustomData->SaveNextImage = false;

    pCustomData->ImageCounter++;

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

            // pCustomData->frame.create(height,width,CV_8U);

            // memcpy( pCustomData->frame.data, info.data, width*height);
            // memcpy( pCustomData->frame.data, img.data, width*height);
            cout << "EH" <<endl;
            char ImageFileName[256];
            sprintf(ImageFileName,"image%05d.png", 1);
            Image img(640, 480, 1);
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    img(x, y) = info.data[y*width+x];
                }
            } 
            // memcpy(img.image_data, info.data, width*height);
            img.write(ImageFileName);
            // cv::Mat img(width, height, 1);
            // img = pCustomData->frame.clone();
            // int x_offset = -0;
            // int y_offset = 0;
            // int xl = width/2-250+x_offset;
            // int yl = height/2-250+y_offset;
            // cv::Rect rect(xl, yl, 500, 500);

            // cv::rectangle(img, rect, cv::Scalar(255), 1);
            // cv::circle(img, cv::Point(width/2+x_offset, height/2+y_offset), 5, cv::Scalar(255), 1);

            // cv::imshow("Test", img);
            // cv::waitKey(0);
            // cv::destroyAllWindows();

            // cv::Mat img32;
            // img.convertTo(img32, CV_32F);
            // cv::Mat dx, dy;
            // cv::Sobel( img32, dx, -1, 2, 0, 3 );
            // cv::Sobel( img32, dy, -1, 0, 2, 3 );
            // cv::magnitude( dx, dy, dx );
            // cv::Mat cont_output;
            // dx.convertTo(cont_output, CV_8U);
            // std::cout << cv::sum(cont_output)[0] << std::endl;
            // cv::imwrite(ImageFileName, cont_output);
        }

    }
    
    // Calling Unref is important!
    gst_buffer_unmap (buffer, &info);
    gst_sample_unref(sample);

    // Set our flag of new image to true, so our main thread knows about a new image.
    return GST_FLOW_OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    gst_init(&argc, &argv);
    // Declare custom data structure for the callback
    CUSTOMDATA CustomData;

    CustomData.ImageCounter = 0;
    CustomData.SaveNextImage = false;

    
    printf("Tcam OpenCV Image Sample\n");

    // Open camera by serial number
    // TcamCamera cam("43810451");
    TcamCamera cam("15410110");

    // Set video format, resolution and frame rate
    cam.set_capture_format("GRAY8", FrameSize{640,480}, FrameRate{15,2});

    // Comment following line, if no live video display is wanted.
    // cam.enable_video_display(gst_element_factory_make("ximagesink", NULL));

    // Register a callback to be called for each new frame
    cam.set_new_frame_callback(new_frame_cb, &CustomData);
    
    // Start the camera
    cam.start();

    // Uncomment following line, if properties shall be listed. Many of the
    // properties that are done in software are available after the stream 
    // has started. Focus Auto is one of them.
    // ListProperties(cam);

    for( int i = 0; i< 200; i++)
    {
        CustomData.SaveNextImage = true; // Save the next image in the callcack call
        sleep(1);
    }


    // Simple implementation of "getch()"
    printf("Press Enter to end the program");
    char dummyvalue[10];
    scanf("%c",dummyvalue);

    cam.stop();

    return 0;
}
