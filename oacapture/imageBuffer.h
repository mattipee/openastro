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
        int getPixelFormat() const { return pixelFormat; }
        bool is8bit() const;
        const void* read_buffer() const { return current; }
        void* write_buffer();
        int frameLength() const { return x * y * depth / 8; }
        void reset(void* img, int fmt, int width, int height);
        void convert(int newPixelFormat);
        void ensure8BitGreyOrRaw();
        void ensure24BitRGB();
        void demosaic(int pattern, int method);
        void greyscale(int targetPixelFormat = -1);
        void flip(bool flipX, bool flipY);
    private:
        bool reserve(int newBufferLength);
        void* nextBuffer();

        static void processFlip8Bit(uint8_t* imageData, int x, int y, bool flipX, bool flipY);
        static void processFlip16Bit(uint8_t* imageData, int x, int y, bool flipX, bool flipY);
        static void processFlip24BitColour(uint8_t* imageData, int x, int y, bool flipX, bool flipY);


        const void* imageData;
        int         pixelFormat;
        int         x;
        int         y;
        int         depth;
        bool        isDemosaicked; 
        const void* current;
        int         bufferLength;
        void*		buffer[2];
        int			nBuffer;
};
