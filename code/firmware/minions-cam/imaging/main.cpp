/**
 *  imaging: firmware to process image from the camera
 * 
 *  Aug 20, 2020
 *  Author(s): Junsu Jang (junsu.jang94@gmail.com)
 *  
 *      Description:
 * 
 *  This firmware configures and runs the camera. It also receives, 
 *  processes and saves images from the camera. 
 * 
 *  The expected cameras are from The Imaging Source, and thus, 
 *  we use the library provided the manufacturor. We also 
 *  use the LibTIFF to compress and save the images. 
 * 
 *  Thresholding:
 *      We take the average and variance of the image for provided number 
 *  of times. Then, we apply a threshold 
 *  
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>

#include "tiffio.h"
#include "tcamcamera.h"

// Default number of images for averaging for thresholding
#define N_AVG_DEFAULT 100
// Default exposure and gain settings of the camera
#define EXPOSURE_DEFAULT 1000
#define GAIN_DEFAULT 16
// Camera dimension
#define WIDTH 2592
#define HEIGHT 1944
// Camera resolution
#define N_PX_IMG WIDTH*HEIGHT

// Create a custom data structure to be passed to the callback function. 
typedef struct
{
    int ID;
} CUSTOMDATA;


using namespace gsttcam;
using namespace std;

// Function declaration
int run_camera(string sn,  int exposure, int gain);
int writeTiff(unsigned char *buf, char* outFileName);
GstFlowReturn new_frame_cb(GstAppSink *appsink, gpointer data);

void ListProperties(TcamCamera &cam);
void setCameraProperty(TcamCamera &cam, string property, int value);
void signalHandler(int signum);
// Global variables

// Serial numbers used for testing
// const string SN[2] = {"15410110", "41810422"};
char ImageFileName[256];
uint32_t tempPxSum, camID, nAvg, tempVarSum, pxThreshold;
uint32_t pxAvgSum = 0, pxSTDSum = 0, k=0;
uint32_t pxAvg;
char *dataDirName;
struct timespec now;
uint8_t camRunning = 0;

/**
 * main: entrance to the main program. It handles input arguments here.
 */
int main(int argc, char *argv[])
{

    string sn;
    nAvg = N_AVG_DEFAULT;
    int exposure = EXPOSURE_DEFAULT;
    int gain = GAIN_DEFAULT;
    for (int i = 0; i < argc; i++) 
    {
        if (strcmp(argv[i], "-s") == 0)
        {
            sn = string(argv[i+1]);
        }
        else if (strcmp(argv[i], "-i") == 0)
        {
            camID = (uint32_t) atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-a") == 0)
        {
            nAvg = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-e") == 0)
        {
            exposure = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-g") == 0)
        {
            gain = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "-d") == 0)
        {
            dataDirName = argv[i+1];
        }
        else if (strcmp(argv[i], "-h") == 0)
        {
            cout << "\t-s\tcamera serial number (8 digit)\n\
        -i\tcamera id (user defined)\n\
        -a\tnumber of frames to average image background for thresholding -- default: 100\n\
        -e\texposure (us) -- default: 1000\n\
        -g\tgain -- defaul: 16\n\
        -d\tpath to data directory\n" << endl;
            return 0;
        }
    }

    if (argc < 3) {
        cout << "Please provide at least the serial number, camera id, \
and data directory. Type -h for help.\n";
        return -1;
    }
    if (sn.length() != 8)
    {
        cout << "Serial number is 8 digits long" << endl;
        return -1;
    }
    if (nAvg <= 0)
    {
        cout << "Please provide a number of frames for averaging above 0" << endl;
        return -1;
    }

    struct stat sb;
    if (!(stat(dataDirName, &sb) == 0 && S_ISDIR(sb.st_mode)))
    {
        // the directory does not exist, so we create it here
        mkdir(dataDirName, 0775);
    }
    run_camera(sn, exposure, gain);
    return 0;
}


/**
 * run_camera: configures and starts the camera
 */
int run_camera(string sn, int exposure, int gain)
{
    // Declare custom data structure for the callback
    CUSTOMDATA CustomData;
    printf("Tcam OpenCV Image Sample\n");

    // Open camera by serial number
    TcamCamera cam(sn); 

    // Set video format, resolution and frame rate
    cam.set_capture_format("GRAY8", FrameSize{WIDTH, HEIGHT}, FrameRate{15, 2});
    // Register a callback to be called for each new frame
    cam.set_new_frame_callback(new_frame_cb, &CustomData);
    // Set camera properties for desired operation
	setCameraProperty(cam, "Exposure Auto", 0);
	setCameraProperty(cam, "Gain Auto", 0);
	setCameraProperty(cam, "Exposure", exposure); //us
	setCameraProperty(cam, "Gain", gain);
	setCameraProperty(cam, "Trigger Global Reset Shutter", 1);
	setCameraProperty(cam, "Trigger Mode", 1);
    
    //ListProperties(cam);

    // Start the camera
    cam.start();
    camRunning = 1;
    signal(SIGINT, signalHandler);  
    while (1)
    {
        if (camRunning == 0)
        {
            cam.stop();
            return 0;
        }
        else
        {
            sleep(1);
        }
    }
}


/**
 * new_frame_cb: Callback called for new images by the internal appsink
 * In this function, we retrieve, filter and compress images.
 */
GstFlowReturn new_frame_cb(GstAppSink *appsink, gpointer data)
{
    int i;
    const GstStructure *str;

    // Acces the image data in the GstSample.
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    // Map gst buffer for data access
    GstMapInfo info;
    gst_buffer_map(buffer, &info, GST_MAP_READ);
    
    if (info.data != NULL) 
    {
        // info.data contains the image data as blob of unsigned char 
		clock_gettime(CLOCK_MONOTONIC, &now);
        GstCaps *caps = gst_sample_get_caps(sample);
        // Get a string containg the pixel format, width and height of the image        
        str = gst_caps_get_structure (caps, 0);    

        if( strcmp( gst_structure_get_string (str, "format"),"GRAY8") == 0)  
        {	     
            k++;
            sprintf(ImageFileName,"%s/image%05d_%d_%ld_%ld.tif", 
                dataDirName, k, camID, now.tv_sec, now.tv_nsec);
            
            // Reduce latency / improve compression rate of TIFF compressor
            // by filtering out noise, which is computed to be the average 
            // of the image. We expect dark-grey background and very sparse 
            // particles, and thus, the average is a good threshold.
            // 
            // Note: simply adding multiple addition of images would have 
            // required long long instead of uint32_t. This is an operation
            // with overhead in 32-bit architecture on Pis. 
            if ( k <= nAvg)
            { 
                // Compute average of the image
                uint32_t tempPxSum = 0; 
                int tempVarSum = 0;
                int curPxAvg, xDiff;
                for (i=0; i < N_PX_IMG; i++)
                {
                    tempPxSum += info.data[i];
                }
                // Add to the sum of averages of the images
                curPxAvg = (int) (tempPxSum / N_PX_IMG);
                pxAvgSum += curPxAvg;
                // Compute variance of the image=
                for (i=0; i < N_PX_IMG; i++)
                {
                    xDiff = ((int)info.data[i]) - curPxAvg;
                    tempVarSum += (xDiff * xDiff);
                }
                pxSTDSum += (uint32_t) sqrt((double)(tempVarSum / (N_PX_IMG-1)));
                if (k == nAvg)
                {
                    // Compute the average over # images
                    pxAvg = pxAvgSum / nAvg;
                    // Average STD
                    uint32_t pxSTD = pxSTDSum / nAvg;
                    // Set the new threshold
                    pxThreshold = pxAvg + 4*pxSTD;
                }
            }
            else 
            {
                for (i=0; i < N_PX_IMG; i++) 
                {
                    info.data[i] = pxAvg ? info.data[i] : (info.data[i] < pxThreshold);
                }
            }
            // Compress image and save it
            writeTiff(info.data, ImageFileName);

            /*fstream myFile;
            myFile.open(ImageFileName, fstream::out);
            myFile << info.data;
            myFile.close();*/

        }
    }
    
    // Calling Unref is important!
    gst_buffer_unmap (buffer, &info);
    gst_sample_unref(sample);

    // Set our flag of new image to true, so our main thread knows 
    // about a new image.
    return GST_FLOW_OK;
}


/**
 *  writeTiff: Compress and save images into TIFF with Packbit (RLE) 
 *  compression algorithm
 * 
 */
int writeTiff(unsigned char *buf, char* outFileName)
{
	int	fd, c;
	uint32_t row, col, band;
    uint32_t linebytes, bufsize;
	TIFF	*out;

	uint32_t width = 2592, length = 1944;
    uint32_t nbands = 1, rowsperstrip=3; /* number of bands in input image*/
	off_t	hdr_size = 0;		    /* size of the header to skip */
	unsigned char *buf1 = NULL;


	uint16_t	photometric = PHOTOMETRIC_MINISBLACK;
	uint16_t	config = PLANARCONFIG_CONTIG;
	uint16_t	fillorder = FILLORDER_LSB2MSB;
	TIFFDataType dtype = TIFF_BYTE;
    uint16_t compression = COMPRESSION_PACKBITS;
    int16_t depth = TIFFDataWidth(dtype); /* bytes per pixel in input image */


	out = TIFFOpen(outFileName, "w");
	if (out == NULL) {
		fprintf(stderr, "%s: Cannot open file for output.\n", outFileName);
		return (-1);
	}

	TIFFSetField(out, TIFFTAG_IMAGEWIDTH, width);
	TIFFSetField(out, TIFFTAG_IMAGELENGTH, length);
	TIFFSetField(out, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
	TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, nbands);
	TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, depth * 8);
	TIFFSetField(out, TIFFTAG_FILLORDER, fillorder);
	TIFFSetField(out, TIFFTAG_PLANARCONFIG, config);
    TIFFSetField(out, TIFFTAG_PHOTOMETRIC, photometric);
    TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);
	TIFFSetField(out, TIFFTAG_COMPRESSION, compression);

    linebytes = width * nbands * depth;
	bufsize = width * nbands * depth;
    
	TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, rowsperstrip );

	lseek(fd, hdr_size, SEEK_SET);		/* Skip the file header */
	for (row = 0; row < length; row++) {
		buf1 = buf + row * bufsize;	
		if (TIFFWriteScanline(out, buf1, row, 0) < 0) {
			fprintf(stderr,	"%s: scanline %lu: Write error.\n",
                    outFileName, (unsigned long) row);
			break;
		}
	}
	TIFFClose(out);
	return (0);
}


/**
 *  ListProperties: List available properties helper function.
 */
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


/**
 * setCameraProperty: Communicate with the camera firmware to set 
 * desired property values for operation
 */
void setCameraProperty(TcamCamera &cam, string property, int value)
{
    shared_ptr<Property> Property = NULL;
	try
	{
		Property = cam.get_property(property);
	}
	catch(std::exception &ex)    
	{
		printf("Error %s : %s\n",ex.what(), property.c_str());
	}

    if( Property != NULL){
        Property->set(cam,value);
        cout << property << " set to: " << value << endl;
    } 
	else 
	{
		cout << property << " setting failed!" << endl;
	}
}

void signalHandler( int signum ) {
    // cleanup and close up stuff here  
    // terminate program  
    if (signum == SIGINT)
        camRunning = 0;
}