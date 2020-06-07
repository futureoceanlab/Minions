/* $Id: raw2tiff.c,v 1.23 2006/03/23 14:54:02 dron Exp $
 *
 * Project:  libtiff tools
 * Purpose:  Convert raw byte sequences in TIFF images
 * Author:   Andrey Kiselev, dron@ak4719.spb.edu
 *
 ******************************************************************************
 * Copyright (c) 2002, Andrey Kiselev <dron@ak4719.spb.edu>
 *
 * Permission to use, copy, modify, distribute, and sell this software and 
 * its documentation for any purpose is hereby granted without fee, provided
 * that (i) the above copyright notices and this permission notice appear in
 * all copies of the software and related documentation, and (ii) the names of
 * Sam Leffler and Silicon Graphics may not be used in any advertising or
 * publicity relating to the software without the specific, prior written
 * permission of Sam Leffler and Silicon Graphics.
 * 
 * THE SOFTWARE IS PROVIDED "AS-IS" AND WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY 
 * WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  
 * 
 * IN NO EVENT SHALL SAM LEFFLER OR SILICON GRAPHICS BE LIABLE FOR
 * ANY SPECIAL, INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY KIND,
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
 * WHETHER OR NOT ADVISED OF THE POSSIBILITY OF DAMAGE, AND ON ANY THEORY OF 
 * LIABILITY, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE 
 * OF THIS SOFTWARE.
 */

// #include "tif_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <math.h>
#include <ctype.h>
#include <iostream>
#include <chrono>

# include <unistd.h>
# include <fcntl.h>
// # include <io.h>
#include "tiffio.h"

#ifndef HAVE_GETOPT
extern int getopt(int, char**, char*);
#endif

#ifndef O_BINARY
# define O_BINARY 0
#endif

typedef enum {
	PIXEL,
	BAND
} InterleavingType;

static	uint16 compression = (uint16) -1;
static	int jpegcolormode = JPEGCOLORMODE_RGB;
static	int quality = 75;		/* JPEG quality */
static	uint16 predictor = 0;

static void swapBytesInScanline(void *, uint32, TIFFDataType);

int
main(int argc, char* argv[])
{
    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::milliseconds ms;
    typedef std::chrono::duration<float> fsec;
	uint32	width = 2592, length = 1944, linebytes, bufsize;
	uint32	nbands = 1;		    /* number of bands in input image*/
	off_t	hdr_size = 0;		    /* size of the header to skip */
	TIFFDataType dtype = TIFF_BYTE;
	int16	depth = 1;		    /* bytes per pixel in input image */
	int	swab = 0;		    /* byte swapping flag */
	uint32  rowsperstrip = 3; //(uint32) -1;
	uint16	photometric = PHOTOMETRIC_MINISBLACK;
	uint16	config = PLANARCONFIG_CONTIG;
	uint16	fillorder = FILLORDER_LSB2MSB;
	int	fd;
	char	*outfilename = NULL, *infilename = NULL;
	TIFF	*out;

	uint32 row, col, band;
	int	c;
	unsigned char *buf = NULL;
    bufsize = width*length;
    unsigned char buf1[bufsize];
	extern int optind;
	extern char* optarg;


    compression = COMPRESSION_PACKBITS;
    //compression = COMPRESSION_LZW;
    dtype = TIFF_BYTE;

    depth = TIFFDataWidth(dtype);
    outfilename = "test_jj2.tif";
    infilename = "image00000_2.bin";
    fd = open(infilename, O_RDONLY|O_BINARY, 0);
	if (fd < 0) {
		fprintf(stderr, "%s: Cannot open input file.\n", infilename);
		return (-1);
	}

    if (read(fd, buf1, bufsize) < 0) {
        fprintf(stderr,
            "scanline %lu: Read error.\n",
            (unsigned long) row);
        return -1;
    }
    auto t0 = Time::now();
	out = TIFFOpen(outfilename, "w");
	if (out == NULL) {
		fprintf(stderr, "%s: Cannot open file for output.\n", outfilename);
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
    //TIFFSetField(out, TIFFTAG_PREDICTOR, 2);

    linebytes = width * nbands * depth;
	bufsize = width * nbands * depth;
    unsigned char *buf2 = NULL; //(unsigned char *)_TIFFmalloc(bufsize);

	/*rowsperstrip = TIFFDefaultStripSize(out, rowsperstrip);
	if (rowsperstrip > length) {
		rowsperstrip = length;
	}
    std::cout << rowsperstrip << std::endl;*/
	TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, rowsperstrip );

	//lseek(fd, hdr_size, SEEK_SET);		/* Skip the file header */
	for (row = 0; row < length; row++) {
       /* if (read(fd, buf1, bufsize) < 0) {
            fprintf(stderr,
                "scanline %lu: Read error.\n",
                (unsigned long) row);
            break;
        }*/
		buf2 = buf1 + row * bufsize;		
		if (TIFFWriteScanline(out, buf2, row, 0) < 0) {
			fprintf(stderr,	"%s: scanline %lu: Write error.\n",
                    outfilename, (unsigned long) row);
			break;
		}
	}
	/*if (buf)
		_TIFFfree(buf);
	if (buf1)
		_TIFFfree(buf1);*/
    auto t1 = Time::now();
	TIFFClose(out);
    auto t2=Time::now();
    fsec fs1 = t1-t0;
    fsec fs2 = t2-t1;
    ms d1 = std::chrono::duration_cast<ms>(fs1);
    ms d2 = std::chrono::duration_cast<ms>(fs2);
    //std::cout << fs1.count() << "s\n";
    std::cout << d1.count() << "ms" << std::endl; 
    std::cout << d2.count() << "ms" << std::endl;
	return (0);
}
