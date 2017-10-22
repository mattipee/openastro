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
void ImageBuffer::boost( bool stretch, int multiply, int algorithm )
{
    // Multiply works ok on RGB24, but my binning doesn't so don't bother
    // Restrict to GREY8 for now.
    if (pixelFormat != OA_PIX_FMT_GREY8) {
        convert(OA_PIX_FMT_GREY8);
    }

    // ALGORITHM
    {
        switch (algorithm) {
            case CONFIG::boost::ALGO_NONE:
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
                    memset(next, 0, length*2);
                    for (int i = 0; i < x; ++i) {
                        for (int j = 0; j < y; ++j) {
                            next[(j/bin)*(x/bin) + (i/bin)] += source[j*x + i];
                        }
                    }
                    x = x/bin;
                    y = y/bin;
                    length = x*y;

                    for (int i=0; i<length; ++i)
                    {
                        reinterpret_cast<uint8_t*>(next)[i] = avg ? next[i]/(bin*bin) : std::min(0x00ff,static_cast<int>(next[i]));
                    }
                    current = next;
                }
                break;

            case CONFIG::boost::ALGO_CUSTOM1:
                {
                    const uint8_t* source = reinterpret_cast<const uint8_t*>(current);
                    uint16_t* next = reinterpret_cast<uint16_t*>(nextBuffer());

                    memset(next, 0, length*2);
                    for (int i = 0; i < x-1; ++i)
                    {
                        for (int j = 0; j < y-1; ++j)
                        {
                            uint8_t p = source[j*x + i];
                            next[j*x + i] += p;
                            next[j*x + i+1] += p;
                            next[(j+1)*x + i] += p;
                            next[(j+1)*x + (i+1)] += p;
                        }
                    }

                    // loop through 16-bit values and convert to 8
                    for (int i=0; i<length; ++i)
                    {
                        reinterpret_cast<uint8_t*>(next)[i] = next[i]/4.0;
                    }
                    current = next;
                }
                break;

            case CONFIG::boost::ALGO_CUSTOM2:
                {
                    const uint8_t* source = reinterpret_cast<const uint8_t*>(current);
                    uint16_t* next = reinterpret_cast<uint16_t*>(nextBuffer());

                    memset(next, 0, length*2);
                    for (int i = 0; i < x-2; ++i)
                    {
                        for (int j = 0; j < y-2; ++j)
                        {
                            uint8_t p = source[j*x + i];
                            next[j*x + i] += p;
                            next[j*x + i+1] += p;
                            next[j*x + i+2] += p;
                            next[(j+1)*x + i] += p;
                            next[(j+1)*x + (i+1)] += p;
                            next[(j+1)*x + (i+2)] += p;
                            next[(j+2)*x + i] += p;
                            next[(j+2)*x + (i+1)] += p;
                            next[(j+2)*x + (i+2)] += p;
                        }
                    }

                    // loop through 16-bit values and convert to 8
                    for (int i=0; i<length; ++i)
                    {
                        reinterpret_cast<uint8_t*>(next)[i] = next[i]/9.0;
                    }

                    current = next;
                }
                break;
        }
    }


    // MULTIPLY
    {
        const uint8_t* source = reinterpret_cast<const uint8_t*>(current);
        uint8_t* next = reinterpret_cast<uint8_t*>(nextBuffer());
        int mul = std::max(1,multiply);
        for (int c = 0; c < length; ++c)
        {
            next[c] = std::min(255, source[c] * mul);
        }
        current = next;
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
}
