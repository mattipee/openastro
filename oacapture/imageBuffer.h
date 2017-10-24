/*****************************************************************************
 *
 * imageBuffer.h -- class declaration
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

#pragma once
#include <iostream>
#include <stdint.h>

class ImageBuffer
{
public:
  ImageBuffer();
  ~ImageBuffer();

  //ImageBuffer clone() const;

  void reset(void* img, int fmt, int width, int height);


  int getPixelFormat() const { return pixelFormat; }
  bool is8bit() const;
  int width() const { return x; }
  int height() const { return y; }
  int frameLength() const { return length; }

  const void* read_buffer() const { return current; }
  void* write_buffer();

  void ensure8BitGreyOrRaw();
  void ensure24BitRGB();
  void convert(int newPixelFormat);

  void demosaic(int pattern, int method);
  void greyscale(int targetPixelFormat = -1);
  void flip(bool flipX, bool flipY);

  void boost(bool stretch, int sharpen, int multiply, int algorithm );
private:
  bool reserve(int newBufferLength);
  void* nextBuffer();

  template <size_t D>
  void convolve(const int8_t (&kernel)[D], double factor = 1.0, double bias = 0.0)
  {
      const uint8_t* source = reinterpret_cast<const uint8_t*>(current);
      uint8_t* next = reinterpret_cast<uint8_t*>(nextBuffer());
      convolve(source, next, x, y, kernel, factor, bias);
      current = next;
  }
  static void convolve(const uint8_t* source, uint8_t* target, int x, int y, const int8_t (&kernel)[1], double factor = 1.0, double bias = 0.0);
  static void convolve(const uint8_t* source, uint8_t* target, int x, int y, const int8_t (&kernel)[9], double factor = 1.0, double bias = 0.0);
  static void convolve(const uint8_t* source, uint8_t* target, int x, int y, const int8_t (&kernel)[25], double factor = 1.0, double bias = 0.0);

  void adpb(int);

  static void processFlip8Bit(uint8_t* imageData, int x, int y, bool flipX, bool flipY);
  static void processFlip16Bit(uint8_t* imageData, int x, int y, bool flipX, bool flipY);
  static void processFlip24BitColour(uint8_t* imageData, int x, int y, bool flipX, bool flipY);


  const void* imageData;
  const void* current;
  void*		  buffer[2];
  int		  nBuffer;
  int         bufferLength;

  int         pixelFormat;
  int         x;
  int         y;
  int         length;
  bool        isDemosaicked; 
};

