#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

//#include <fstream>
//#include <omp.h>


#define O_BINARY 0
#define N_AVG_IMG 100
#define WIDTH 2592
#define HEIGHT 1944
#define N_PX_IMG WIDTH*HEIGHT

// Create a custom data structure to be passed to the callback function. 
typedef struct
{
    int ID;
} CUSTOMDATA;


using namespace gsttcam;
using namespace std;


const string SN[2] = {"15410110", "41810422"};
char ImageFileName[256];

uint32_t tempPxSum, pxAvg, camID;
uint32_t pxAvgSum = 0, k=0;

int run_camera(string sn);
int writeTiff(unsigned char *buf, char* outFileName);
GstFlowReturn new_frame_cb(GstAppSink *appsink, gpointer data);

void ListProperties(TcamCamera &cam);
void setCameraProperty(TcamCamera &cam, string property, int value);

struct timespec now;


int main(int argc, char **argv)
{
    if (argc < 3) {
        printf("Need Serial number");
        return 0;
    }
    int sn_i = atoi(argv[1]);
    camID = (uint32_t) atoi(argv[2]);
    run_camera(SN[sn_i]);
    return 0;
}


int run_camera(string sn)
{
    // Declare custom data structure for the callback
    CUSTOMDATA CustomData;
    printf("Tcam OpenCV Image Sample\n");

    // Open camera by serial number
    TcamCamera cam(sn); 

    // Set video format, resolution and frame rate
    cam.set_capture_format("GRAY8", FrameSize{2592,1944}, FrameRate{15, 2});
    // Register a callback to be called for each new frame
    cam.set_new_frame_callback(new_frame_cb, &CustomData);
    // Set camera properties for desired operation
	setCameraProperty(cam, "Exposure Auto", 0);
	setCameraProperty(cam, "Gain Auto", 0);
	setCameraProperty(cam, "Exposure", 1500); //us
	setCameraProperty(cam, "Gain", 16);
	setCameraProperty(cam, "Trigger Global Reset Shutter", 1);
	setCameraProperty(cam, "Trigger Mode", 1);
    
    //ListProperties(cam);

    // Start the camera
    cam.start();
    sleep(100000);
    cam.stop();
    return 0;
}


// Callback called for new images by the internal appsink
// In this function, we retrieve, filter and compress images.
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
            sprintf(ImageFileName,"/home/pi/data/image%05d_%d_%d_%d.tif", 
                k, camID, now.tv_sec, now.tv_nsec);
            
            // Reduce latency / improve compression rate of TIFF compressor
            // by filtering out noise, which is computed to be the average 
            // of the image. We expect dark-grey background and very sparse 
            // particles, and thus, the average is a good threshold.
            // 
            // Note: simply adding multiple addition of images would have 
            // required long long instead of uint32_t. This is an operation
            // with overhead in 32-bit architecture on Pis. 
            if ( k <= N_AVG_IMG)
            {
                // Compute average of the image
                tempPxSum = 0;
                for (i=0; i<N_PX_IMG; i++)
                {
                    tempPxSum += info.data[i];
                }
                // Add to the sum of averages of the images
                pxAvgSum += (tempPxSum / N_PX_IMG);
                if (k == N_AVG_IMG)
                {
                    // Compute the average over # images
                    pxAvg = pxAvgSum / N_AVG_IMG;
                }
            }
            else 
            {
                for (i=0; i < N_PX_IMG; i++) 
                {
                    info.data[i] = 0 : info.data[i] ? info.data[i] < pxAvg;
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


// Compress and save images into TIFF with Packbit (RLE) compression algorithm
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
\    
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


// Communicate with the camera firmware to set 
// desired property values for operation
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