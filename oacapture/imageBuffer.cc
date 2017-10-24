/*****************************************************************************
 *
 * imageBuffer.cc -- class for the encapsulation of image buffer machinery
 *
 * Copyright 2013,2014,2015,2016 James Fidell (james@openastroproject.org)
 *
 * License:
 *
 * This file is part of the Open Astro Project.
 *
 * The Open Astro Project is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Open Astro Project is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Open Astro Project.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************/

#include "imageBuffer.h"

#include <oa_common.h>

#ifdef HAVE_QT5
#include <QtWidgets>
#endif
#include <QtCore>
#include <QtGui>

extern "C" {
#include <pthread.h>

#include <openastro/camera.h>
#include <openastro/demosaic.h>
#include <openastro/video.h>
}

#include "configuration.h"

#include <map>

namespace {
    static const int8_t dBin2x2[] =
    { 1, 1, 0,
      1, 1, 0,
      0, 0, 0 };
    static const int8_t dBin3x3[] =
    { 1, 1, 1,
      1, 1, 1,
      1, 1, 1 };
    static const int8_t dBin4x4[] =
    { 1, 1, 1, 1, 0,
      1, 1, 1, 1, 0,
      1, 1, 1, 1, 0,
      1, 1, 1, 1, 0,
      0, 0, 0, 0, 0};
    static const int8_t sharpenSoft[] =
    {-1, -1, -1, -1, -1,
      -1,  2,  2,  2, -1,
      -1,  2,  8,  2, -1,
      -1,  2,  2,  2, -1,
      -1, -1, -1, -1, -1};
    static const int8_t sharpenHard[] =
    { -1, -1, -1,
      -1,  9, -1,
      -1, -1, -1};
}

ImageBuffer::ImageBuffer()
  : imageData(0), pixelFormat(-1), x(0), y(0), length(0),
    current(0), bufferLength(0), nBuffer(-1)
{}


ImageBuffer::~ImageBuffer()
{
    if ( buffer[0] ) {
        free ( buffer[0] );
        free ( buffer[1] );
    }
}


bool ImageBuffer::is8bit() const
{
    return 1 == OA_BYTES_PER_PIXEL(pixelFormat);
}


void* ImageBuffer::write_buffer() {
    if (current != imageData)
        return const_cast<void*>(current);

    void* next = nextBuffer();
    memcpy(next, current, length);
    current = next;
    return next;
}


void ImageBuffer::reset(void* img, int fmt, int width, int height)
{
    imageData = img;
    current = imageData;
    pixelFormat = fmt;
    x = width;
    y = height;
    length = x * y * OA_BYTES_PER_PIXEL( fmt );
    reserve(x*y*6);
} 


bool ImageBuffer::reserve(int newBufferLength)
{
    if ( newBufferLength > bufferLength ) {
        if (!( buffer[0] = realloc ( buffer[0],
                        newBufferLength ))) {
            return false;
        }
        if (!( buffer[1] = realloc ( buffer[1],
                        newBufferLength ))) {
            return false;
        }
        bufferLength = newBufferLength;
    }
    return true;
}


void* ImageBuffer::nextBuffer() {
    nBuffer = ( -1 == nBuffer ) ? 0 :
        !nBuffer;
    return buffer[nBuffer];
}


void ImageBuffer::convert(int newPixelFormat)
{
    void* next = nextBuffer();
    oaconvert ( current, next, x, y, pixelFormat, newPixelFormat );
    pixelFormat = newPixelFormat;
    current = next;
}


void ImageBuffer::ensure24BitRGB()
{
    if ( pixelFormat == OA_PIX_FMT_RGB24 )
        return;
    if ( OA_ISGREYSCALE( pixelFormat ) || OA_ISBAYER( pixelFormat ) )
        return; 
    if ( OA_IS_LUM_CHROM( pixelFormat )) {
        convert( OA_PIX_FMT_RGB24 );
        return;
    }
    qWarning() << __FUNCTION__ << " unable to convert format " << pixelFormat << " to RGB24 (48bit colour?)";
}


void ImageBuffer::ensure8BitGreyOrRaw()
{
    int fmt;
    bool convertTo8 = false;
    switch (pixelFormat) {
        case OA_PIX_FMT_BGGR16LE:
        case OA_PIX_FMT_BGGR16BE:
            convertTo8 = true;
            fmt = OA_PIX_FMT_BGGR8;
            break;
        case OA_PIX_FMT_RGGB16LE:
        case OA_PIX_FMT_RGGB16BE:
            convertTo8 = true;
            fmt = OA_PIX_FMT_RGGB8;
            break;
        case OA_PIX_FMT_GBRG16LE:
        case OA_PIX_FMT_GBRG16BE:
            convertTo8 = true;
            fmt = OA_PIX_FMT_GBRG8;
            break;
        case OA_PIX_FMT_GRBG10:
        case OA_PIX_FMT_GRBG10P:
        case OA_PIX_FMT_GRBG16LE:
        case OA_PIX_FMT_GRBG16BE:
            convertTo8 = true;
            fmt = OA_PIX_FMT_GRBG8;
            break;
        case OA_PIX_FMT_GREY16BE:
        case OA_PIX_FMT_GREY16LE:
            convertTo8 = true;
            fmt = OA_PIX_FMT_GREY8;
            break;
    }
    if (convertTo8) {
        void* next = nextBuffer();
        oaconvert(current, next,
                this->x, this->y,
                pixelFormat, OA_PIX_FMT_GREY8);
        current = next;
        pixelFormat = fmt;
    }
}


void ImageBuffer::demosaic(int pattern, int method)
{
    if ( !OA_ISBAYER ( pixelFormat ))
        return;

    // TODO TODO
    // Until oademosic() can do 10/16bit, we have to downsample to 8bit
    ensure8BitGreyOrRaw();
    // Now we know we'll end up with a valid RGB24 after demosic

    // Demosaic the output frame
    void* next = nextBuffer();
    oademosaic ( current, next, x, y, 8, pattern, method );
    pixelFormat = OA_PIX_FMT_RGB24;
    length = x*y*3;
    current = next;
}


void ImageBuffer::greyscale(int targetPixelFormat)
{
    if (targetPixelFormat == -1)
        targetPixelFormat = OA_GREYSCALE_FMT( pixelFormat );

    void* next = nextBuffer();
    oaconvert ( current, next, x, y, pixelFormat, targetPixelFormat );
    pixelFormat = targetPixelFormat;
    length = x*y*OA_BYTES_PER_PIXEL(targetPixelFormat);
    current = next;
}


void
ImageBuffer::flip ( bool flipX, bool flipY )
{
  if (!flipX && !flipY) return;

  // fake up a format for mosaic frames here as properly flipping a
  // mosaicked frame would be quite hairy
  int assumedFormat = pixelFormat;
  if ( OA_ISBAYER8 ( pixelFormat )) {
    assumedFormat = OA_PIX_FMT_GREY8;
  } else {
    if ( OA_ISBAYER16 ( pixelFormat )) {
      assumedFormat = OA_PIX_FMT_GREY16BE;
    }
  }

  void* buf = write_buffer();
  current = buf;

  switch ( assumedFormat ) {
    case OA_PIX_FMT_GREY8:
      processFlip8Bit ( (uint8_t*) buf, x, y, flipX, flipY );
      break;
    case OA_PIX_FMT_GREY16BE:
    case OA_PIX_FMT_GREY16LE:
      processFlip16Bit (  (uint8_t*)buf, x, y, flipX, flipY );
      break;
    case OA_PIX_FMT_RGB24:
    case OA_PIX_FMT_BGR24:
      processFlip24BitColour (  (uint8_t*)buf, x, y, flipX, flipY );
      break;
    default:
      qWarning() << __FUNCTION__ << " unable to flip format " << pixelFormat;
      break;
  }
}


void
ImageBuffer::processFlip8Bit ( uint8_t* imageData, int imageSizeX, int imageSizeY, bool flipX, bool flipY )
{
  int length = imageSizeX * imageSizeY;
  if ( flipX && flipY ) {
    uint8_t* p1 = imageData;
    uint8_t* p2 = imageData + length - 1;
    uint8_t s;
    while ( p1 < p2 ) {
      s = *p1;
      *p1++ = *p2;
      *p2-- = s;
    }
  } else {
    if ( flipX ) {
      uint8_t* p1;
      uint8_t* p2;
      uint8_t s;
      for ( unsigned int y = 0; y < imageSizeY; y++ ) {
        p1 = imageData + y * imageSizeX;
        p2 = p1 + imageSizeX - 1;
        while ( p1 < p2 ) {
          s = *p1;
          *p1++ = *p2;
          *p2-- = s;
        }
      }
    }
    if ( flipY ) {
      uint8_t* p1;
      uint8_t* p2;
      uint8_t s;
      p1 = imageData;
      for ( unsigned int y = imageSizeY - 1; y >= imageSizeY / 2;
          y-- ) {
        p2 = imageData + y * imageSizeX;
        for ( unsigned int x = 0; x < imageSizeX; x++ ) {
          s = *p1;
          *p1++ = *p2;
          *p2++ = s;
        }
      }
    }
  }
}


void
ImageBuffer::processFlip16Bit ( uint8_t* imageData, int imageSizeX, int imageSizeY, bool flipX, bool flipY )
{
  int length = imageSizeX * imageSizeY * 2;
  if ( flipX && flipY ) {
    uint8_t* p1 = imageData;
    uint8_t* p2 = imageData + length - 2;
    uint8_t s;
    while ( p1 < p2 ) {
      s = *p1;
      *p1++ = *p2;
      *p2++ = s;
      s = *p1;
      *p1++ = *p2;
      *p2 = s;
      p2 -= 3;
    }
  } else {
    if ( flipX ) {
      uint8_t* p1;
      uint8_t* p2;
      uint8_t s;
      for ( unsigned int y = 0; y < imageSizeY; y++ ) {
        p1 = imageData + y * imageSizeX * 2;
        p2 = p1 + ( imageSizeX - 1 ) * 2;
        while ( p1 < p2 ) {
          s = *p1;
          *p1++ = *p2;
          *p2++ = s;
          s = *p1;
          *p1++ = *p2;
          *p2 = s;
          p2 -= 3;
        }
      }
    }
    if ( flipY ) {
      uint8_t* p1;
      uint8_t* p2;
      uint8_t s;
      p1 = imageData;
      for ( unsigned int y = imageSizeY - 1; y > imageSizeY / 2;
          y-- ) {
        p2 = imageData + y * imageSizeX * 2;
        for ( unsigned int x = 0; x < imageSizeX * 2; x++ ) {
          s = *p1;
          *p1++ = *p2;
          *p2++ = s;
        }
      }
    }
  }
}


void
ImageBuffer::processFlip24BitColour ( uint8_t* imageData, int imageSizeX, int imageSizeY, bool flipX, bool flipY )
{
  int length = imageSizeX * imageSizeY * 3;
  if ( flipX && flipY ) {
    uint8_t* p1 = imageData;
    uint8_t* p2 = imageData + length - 3;
    uint8_t s;
    while ( p1 < p2 ) {
      s = *p1;
      *p1++ = *p2;
      *p2++ = s;
      s = *p1;
      *p1++ = *p2;
      *p2++ = s;
      s = *p1;
      *p1++ = *p2;
      *p2 = s;
      p2 -= 5;
    }
  } else {
    if ( flipX ) {
      uint8_t* p1;
      uint8_t* p2;
      uint8_t s;
      for ( unsigned int y = 0; y < imageSizeY; y++ ) {
        p1 = imageData + y * imageSizeX * 3;
        p2 = p1 + ( imageSizeX - 1 ) * 3;
        while ( p1 < p2 ) {
          s = *p1;
          *p1++ = *p2;
          *p2++ = s;
          s = *p1;
          *p1++ = *p2;
          *p2++ = s;
          s = *p1;
          *p1++ = *p2;
          *p2 = s;
          p2 -= 5;
        }
      }
    }
    if ( flipY ) {
      uint8_t* p1;
      uint8_t* p2;
      uint8_t s;
      p1 = imageData;
      for ( unsigned int y = imageSizeY - 1; y > imageSizeY / 2;
          y-- ) {
        p2 = imageData + y * imageSizeX * 3;
        for ( unsigned int x = 0; x < imageSizeX * 3; x++ ) {
          s = *p1;
          *p1++ = *p2;
          *p2++ = s;
        }
      }
    }
  }
}


// Note: some of these transformations will work when applied to a
// single buffer, rather than writing into a new one.
// We've preallocated our buffers anyway, so I don't bother.
// Perhaps not good for the cache, but these were written quick and
// dirty to begin with.
void ImageBuffer::boost( bool stretch, int sharpen, int multiply, int algorithm )
{
    // Multiply works ok on RGB24, but my binning doesn't so don't bother
    // Restrict to GREY8 for now.
    if (pixelFormat != OA_PIX_FMT_GREY8) {
        convert(OA_PIX_FMT_GREY8);
    }

    // ALGORITHM
    switch (algorithm) {
        case CONFIG::boost::ALGO_NONE:
            break;

        case CONFIG::boost::ALGO_ADPB4:
            adpb(4);
            break;

        case CONFIG::boost::ALGO_ADPB6:
            adpb(6);
            break;

        case CONFIG::boost::ALGO_ADPB8:
            adpb(8);
            break;

        case CONFIG::boost::ALGO_BIN2X2: // bin2x2
        case CONFIG::boost::ALGO_BIN3X3: // bin3x3
        case CONFIG::boost::ALGO_BIN4X4: // bin4x4
        case CONFIG::boost::ALGO_AVG2X2: // avg2x2
        case CONFIG::boost::ALGO_AVG3X3: // avg3x3
        case CONFIG::boost::ALGO_AVG4X4: // avg4x4
            {
                const uint8_t* source = reinterpret_cast<const uint8_t*>(current);
                uint16_t* next = reinterpret_cast<uint16_t*>(nextBuffer());

                int bin;
                bool avg = false;
                switch (algorithm) {
                    case CONFIG::boost::ALGO_BIN2X2: bin = 2; break;
                    case CONFIG::boost::ALGO_BIN3X3: bin = 3; break;
                    case CONFIG::boost::ALGO_BIN4X4: bin = 4; break;
                    case CONFIG::boost::ALGO_AVG2X2: bin = 2; avg = true; break;
                    case CONFIG::boost::ALGO_AVG3X3: bin = 3; avg = true; break;
                    case CONFIG::boost::ALGO_AVG4X4: bin = 4; avg = true; break;
                    default: bin = 1;
                }

                // accumulate 8bit sums as 16bit
                memset(next, 0, length*2);
                for (int i = 0; i < x; ++i) {
                    for (int j = 0; j < y; ++j) {
                        next[(j/bin)*(x/bin) + (i/bin)] += source[j*x + i];
                    }
                }
                length = x*y;

                // clip (bin) or divide (avg) 16bit sums back to 8bit
                for (int i=0; i<length; ++i)
                {
                    reinterpret_cast<int8_t*>(next)[i] = std::min(0xff, next[i] / (avg?(bin*bin):1));
                }

                x = x/bin;
                y = y/bin;
                current = next;
            }
            break;

        case CONFIG::boost::ALGO_dBIN2X2:
            convolve(dBin2x2);
            break; 
        case CONFIG::boost::ALGO_dBIN3X3:
            convolve(dBin3x3); 
            break; 
        case CONFIG::boost::ALGO_dBIN4X4:
            convolve(dBin4x4); 
            break; 
        case CONFIG::boost::ALGO_dAVG2X2:
            convolve(dBin2x2, 1.0/4.0); 
            break; 
        case CONFIG::boost::ALGO_dAVG3X3:
            convolve(dBin3x3, 1.0/9.0); 
            break; 
        case CONFIG::boost::ALGO_dAVG4X4:
            convolve(dBin4x4, 1.0/16.0); 
            break; 
    }


    // MULTIPLY
    if (multiply > 1) {
        const int8_t identity[] = { multiply };
        convolve(identity);
    }

    // STRETCH
    if (stretch)
    {
        const uint8_t* source = reinterpret_cast<const uint8_t*>(current);
        uint8_t* next = reinterpret_cast<uint8_t*>(nextBuffer());

        uint8_t min = 255;
        uint8_t max = 1;
        for (int i=0; i<length; ++i)
        {
            if (source[i] < min) min = source[i];
            if (source[i] > max) max = source[i];
        }
        //std::cerr << "strech min: " << (int)min << " max: " << (int)max << "\n"; 
        double scale_factor = 255/max;
        for (int i=0; i<length; ++i)
        {
            next[i] = source[i] * scale_factor;
        }
        current = next;
    }


    // Sharpen
    switch (sharpen) {
        case CONFIG::boost::SHARPEN_NONE:
            break;
        case CONFIG::boost::SHARPEN_SOFT:
            convolve(sharpenSoft, 1.0/8.0);
            break;
        case CONFIG::boost::SHARPEN_HARD:
            convolve(sharpenHard);
            break;
    }
}

void ImageBuffer::convolve(const uint8_t* source, uint8_t* target, int x, int y, const int8_t (&k)[1], double factor, double bias)
{
    for(int i = 0; i < x; i++) {
        for(int j = 0; j < y; j++)
        {
            int64_t val = source[ j * x +  i   ] * k[0];

            //truncate values smaller than zero and larger than 255
            target[j * x + i] = std::min(std::max(int(factor * val + bias), 0), 255);
        }
    }
}

void ImageBuffer::convolve(const uint8_t* source, uint8_t* target, int x, int y, const int8_t (&k)[9], double factor, double bias)
{
    for(int i = 1; i < x-1; i++) {
        for(int j = 1; j < y-1; j++)
        {
            int64_t val =
                source[(j-1) * x + (i-1)] * k[0] +
                source[(j-1) * x +  i   ] * k[1] +
                source[(j-1) * x + (i+1)] * k[2] +
                source[ j    * x + (i-1)] * k[3] +
                source[ j    * x +  i   ] * k[4] +
                source[ j    * x + (i+1)] * k[5] +
                source[(j+1) * x + (i-1)] * k[6] +
                source[(j+1) * x +  i   ] * k[7] +
                source[(j+1) * x + (i+1)] * k[8];

            //truncate values smaller than zero and larger than 255
            target[j * x + i] = std::min(std::max(int(factor * val + bias), 0), 255);
        }
    }
}

void ImageBuffer::convolve(const uint8_t* source, uint8_t* target, int x, int y, const int8_t (&k)[25], double factor, double bias)
{
    for(int i = 2; i < x-2; i++) {
        for(int j = 2; j < y-2; j++)
        {
            int64_t val =
                source[(j-2) * x + (i-2)] * k[0] +
                source[(j-2) * x + (i-1)] * k[1] +
                source[(j-2) * x +  i   ] * k[2] +
                source[(j-2) * x + (i+1)] * k[3] +
                source[(j-2) * x + (i+2)] * k[4] +
                source[(j-1) * x + (i-2)] * k[5] +
                source[(j-1) * x + (i-1)] * k[6] +
                source[(j-1) * x +  i   ] * k[7] +
                source[(j-1) * x + (i+1)] * k[8] +
                source[(j-1) * x + (i+2)] * k[9] +
                source[ j    * x + (i-2)] * k[10] +
                source[ j    * x + (i-1)] * k[11] +
                source[ j    * x +  i   ] * k[12] +
                source[ j    * x + (i+1)] * k[13] +
                source[ j    * x + (i+2)] * k[14] +
                source[(j+1) * x + (i-2)] * k[15] +
                source[(j+1) * x + (i-1)] * k[16] +
                source[(j+1) * x +  i   ] * k[17] +
                source[(j+1) * x + (i+1)] * k[18] +
                source[(j+1) * x + (i+2)] * k[19] +
                source[(j+2) * x + (i-2)] * k[20] +
                source[(j+2) * x + (i-1)] * k[21] +
                source[(j+2) * x +  i   ] * k[22] +
                source[(j+2) * x + (i+1)] * k[23] +
                source[(j+2) * x + (i+2)] * k[24];

            //truncate values smaller than zero and larger than 255
            target[j * x + i] = std::min(std::max(int(factor * val + bias), 0), 255);
        }
    }
}


struct AbsCompare { bool operator()(int8_t a, int8_t b) { return abs(a) < abs(b); } };
void ImageBuffer::adpb(int Rb)
{
    // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4541814/

    // Rb is the maximum binning ratio

    int L = x*y;



    // ============================================================
    // 2.1. Image Degradation Model for Low-Light Image Acquisition
    // ============================================================

    // A low-light input image consists of signal plus noise
    // This is what we have captured and are processing here

    // ------------------------------------
    // Equation 1: g(x,y) = f(x,y) + η(x,y)
    // ------------------------------------

    const uint8_t* g = reinterpret_cast<const uint8_t*>(current);

    
    // ======================================
    // 3.1. Brightness Adaptive Binning Ratio
    // ======================================

    // Calculates the optimal amplification ratio at each pixel
    // using a 3x3 average filter.

    // -----------------------------------------------------------
    // Equation 6: t(x,y) = (h3x3 * g(x,y)) / max{ h3x3 * g(x,y) }
    // -----------------------------------------------------------

    // h3x3 represents the 3 x 3 average filter
    static const int8_t h3x3[] =
    { 1, 1, 1,
      1, 1, 1,
      1, 1, 1};

    // compute h3x3 * g(x,y)
    // - apply 3x3 average filter to input
    uint8_t* h3x3_g = (uint8_t*)malloc(L);
    convolve(g, h3x3_g, x, y, h3x3, 1.0/9.0);

    // compute max{ h3x3 * g(x,y) }
    // - select the maximum value
    double max=0;
    for (int i=0; i<L; ++i) if (h3x3_g[i] > max) max = h3x3_g[i];

    // compute t(x,y)
    // - divide each pixel by the maximum value
    double* t = (double*)malloc(L*sizeof(double));
    for (int i=0; i<L; ++i) t[i] = h3x3_g[i] / max;

    free(h3x3_g);

    // ---------------------------------------------
    // Equation 7: r(x,y) = 1 + (1 - t(x,y)/(Rb - 1)
    // ---------------------------------------------

    // Rb represents the maximum binning ratio
    // - see function argument (int Rb)

    // compute r(x,y)
    // - calculate optimal binning ratio for each pixel
    double* r = (double*)malloc(L*sizeof(double));
    for (int i=0; i<L; ++i) r[i] = 1 + (1 - t[i]) * (Rb - 1);

    free(t);


    // =============================
    // 3.2. Context-Adaptive Binning
    // =============================

    // Calculate a weighted sum of pixels based on the relationship
    // between each pixel and it's neighbours


    // also...


    // =================================
    // 3.3. Noise-Adaptive Pixel Binning
    // =================================

    // Uniform binning is effective in a noise region.
 

    // identify buffer for output of the entire algorithm
    uint8_t* output = reinterpret_cast<uint8_t*>(nextBuffer());

    // iterate through all pixels
    for (int i=0+1; i<x-1; ++i)
    for (int j=0+1; j<y-1; ++j)
    {
        // --------------------------------------------------------------------------------
        // Equation 8: dx,y = sort{g(x, y) − g(x + i, y + j)},  for i, j = −⌊p/2⌋, …, ⌊p/2⌋
        // --------------------------------------------------------------------------------

        // declare a sorted structure to hold:
        // - the difference of each pixel from the center pixel
        // - the position of the pixel in the original image
        // keys are sorted according to "abs(a) < abs(b)" and may be repeated
        // FIXME C++11 allows AbsCompare to be declared here, locally
        typedef std::multimap<int16_t, uint8_t, ::AbsCompare> sorted_diff_vec;
        sorted_diff_vec dxy;

        // current pixel value
        uint8_t gxy = g[j*x + i];
       
        // generate the sorted differences from a 3x3 window
        dxy.insert(std::make_pair(gxy - g[(j-1)*x + (i-1)], 0));
        dxy.insert(std::make_pair(gxy - g[(j-1)*x +  i   ], 1)); 
        dxy.insert(std::make_pair(gxy - g[(j-1)*x + (i+1)], 2)); 
        dxy.insert(std::make_pair(gxy - g[ j   *x + (i-1)], 3)); 
        dxy.insert(std::make_pair(gxy - g[ j   *x +  i   ], 4)); 
        dxy.insert(std::make_pair(gxy - g[ j   *x + (i+1)], 5)); 
        dxy.insert(std::make_pair(gxy - g[(j+1)*x + (i-1)], 6)); 
        dxy.insert(std::make_pair(gxy - g[(j+1)*x +  i   ], 7)); 
        dxy.insert(std::make_pair(gxy - g[(j+1)*x + (i+1)], 8)); 

        // ----------------------------
        // Equation 10: bc(x,y)=rTssx,y
        // ----------------------------

        // result of context-adaptive binning is a weightes sum of similar pixels

        // compute weighting vector rs
        // - actually unused, here for clarity
        double rs[9];

        // declare accumulator for weighted sums
        double bcxy = 0;
        double buxy = 0;

        // declare variable to store mean
        double s_bar = 0;

        // optimal binning ratio for this pixel
        double rxy = r[j*x + i];

        // pixel order q represents each sorted pixel in the 3x3 window
        int q = 1;
        for (sorted_diff_vec::const_iterator iter = dxy.begin();
                iter != dxy.end(); ++iter, ++q)
        {
            int d = iter->first;
            uint8_t pos = iter->second;
            int s = gxy - d;
           
            // accumulate for mean
            s_bar += s/9.0;

            // -----------------------------------------------------
            // Equation 11: rs(q)=⎧ 1,             r(x,y)−(q−1)>1
            //                    ⎨ r(x,y)−(q−1),  0<r(x,y)−(q−1) ≤1
            //                    ⎩ 0,             otherwise
            // -----------------------------------------------------

            // calculate weighting for each pixel in sorted vector
            if (rxy - (q - 1) > 1) {
                rs[pos] = 1;
            } else if (rxy - (q - 1) > 0) {
                rs[pos] = rxy - (q - 1);
            } else {
                rs[pos] = 0;
            }

            // accumulate the weighted sum
            bcxy += rs[pos] * s;

            

            // ---------------------------------------------------
            // Equation 17: ru(q)={ 1,                   if q=1
            //                    { 1/(p2−1){r(x,y)−1},  otherwise
            // ---------------------------------------------------

            // rxy is the optimal binning ratio for this pixel
            // center pixel has weight 1
            double ru = (rxy-1.0)/8.0;
            buxy += (pos == 4) ? s : (ru * s); 

        }

        // pixels may saturate, so clip
        bcxy = std::min(255.0,bcxy);
        buxy = std::min(255.0,buxy);

        // this has been manual convolution of the adaptive binning kernel
        // this has also been manual convolution of the uniform binning kernel


        // ---------------------------------------------------------------------------
        // Equation 19: b(x,y)=(1−γ) * bc(x,y) + γ * bu(x,y), for γ=1/λ * |s¯(q)−s(1)|
        // ---------------------------------------------------------------------------

        // λ is a constant for the sensitivity of noise suppression
        // s¯(q) is the mean value of the local window
        // s(1) is the center pixel

        // calculate gamma
        // - abs() was giving me integers, hence the ternary below
        double gamma = ((s_bar < gxy) ? (gxy - s_bar) : (s_bar - gxy))/16.0;//abs(s_bar - gxy)/1.0;
        
        // combine the result of adaptive binning and uniform binning as a function of gamma 
        double bxy = ((1.0-gamma) * bcxy) + (gamma * buxy);
        //b[j*x + i] = std::max(0.0,std::min(255.0, bxy));


        // =======================================
        // 3.4. Image Blending for Anti-Saturation
        // =======================================

        // Combine the amplified image (b) and the input (g) in order to prevent saturation

        // ----------------------------------------------------
        // Equation 21: w(x,y)= (1/μ){b(x,y)/(Rb−1) + g(x,y)/2}
        // ----------------------------------------------------

        // Compute the blending coefficient
        // - μ is the maximum bit depth of the image for normalisation
        
        double mu = 255;
        double wxy = (1/mu) * (( bxy / (Rb - 1.0) ) + ( gxy / 2.0 ));

        // ----------------------------------------------------
        // Equation 20: f^(x,y)=(1−w(x,y))⋅b(x,y)+w(x,y)⋅g(x,y)
        // ----------------------------------------------------

        double fxy = (1.0-wxy)*(bxy) + wxy*(gxy);
        output[j*x + i] = std::max(0.0,std::min(255.0, 1.2*fxy));
    }
    free(r);
    current = output;
}
