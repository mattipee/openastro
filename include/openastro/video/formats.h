/*****************************************************************************
 *
 * oacam.h -- camera API (sub)header for frame formats
 *
 * Copyright 2014 James Fidell (james@openastroproject.org)
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

#ifndef OPENASTRO_CAMERA_FORMATS_H
#define OPENASTRO_CAMERA_FORMATS_H

#include "../demosaic.h"
#include <math.h>

#define DEFINE_OA_PIX_FMT(OA_PIX_FMT) \
  OA_PIX_FMT(RGB24) \
  OA_PIX_FMT(BGR24) \
  OA_PIX_FMT(GREY8) \
  OA_PIX_FMT(GREY16BE) \
  OA_PIX_FMT(GREY16LE) \
  OA_PIX_FMT(BGGR8) \
  OA_PIX_FMT(RGGB8) \
  OA_PIX_FMT(GBRG8) \
  OA_PIX_FMT(GRBG8) \
  OA_PIX_FMT(GRBG10) \
  OA_PIX_FMT(GRBG10P) \
  OA_PIX_FMT(BGGR16LE) \
  OA_PIX_FMT(BGGR16BE) \
  OA_PIX_FMT(RGGB16LE) \
  OA_PIX_FMT(RGGB16BE) \
  OA_PIX_FMT(GBRG16LE) \
  OA_PIX_FMT(GBRG16BE) \
  OA_PIX_FMT(GRBG16LE) \
  OA_PIX_FMT(GRBG16BE) \
  OA_PIX_FMT(RGB48BE) \
  OA_PIX_FMT(RGB48LE) \
  OA_PIX_FMT(BGR48BE) \
  OA_PIX_FMT(BGR48LE) \
  \
  OA_PIX_FMT(YUV444P) \
  OA_PIX_FMT(YUV422P) \
  OA_PIX_FMT(YUV420P) \
  OA_PIX_FMT(YUV410P) \
  OA_PIX_FMT(YUYV) \
  OA_PIX_FMT(UYVY) \
  OA_PIX_FMT(YUV420) \
  OA_PIX_FMT(YUV411) \
  OA_PIX_FMT(YUV410) \


#define OA_PIX_FMT_GRAY8    OA_PIX_FMT_GREY8 
#define OA_PIX_FMT_GRAY16BE OA_PIX_FMT_GREY16BE
#define OA_PIX_FMT_GRAY16LE OA_PIX_FMT_GREY16LE


#define ENUM(FMT) OA_PIX_FMT_##FMT,
enum { OA_PIX_FMT_NONE = 0,
       DEFINE_OA_PIX_FMT(ENUM)
       OA_PIX_FMT_MAX };

/*
#define STRING(FMT) #FMT,
static const char *oa_pix_fmt_strings[] = {
    "error", DEFINE_OA_PIX_FMT(STRING)
};

#define OA_PIX_FMT_STRING(x) \
    oa_pix_fmt_strings[x>0 && x<OA_PIX_FMT_MAX ? x : 0]
*/

static const char* OA_PIX_FMT_STRING(int x) {
    #define STRING(FMT) #FMT,
    static const char *oa_pix_fmt_strings[] = {
        "error", DEFINE_OA_PIX_FMT(STRING)
    };
    return oa_pix_fmt_strings[x>0 && x<OA_PIX_FMT_MAX ? x : 0];
}

static int OA_ISGREYSCALE(int x) {
    switch(x) {     
        case OA_PIX_FMT_GREY8:
        case OA_PIX_FMT_GREY16BE:
        case OA_PIX_FMT_GREY16LE:
            return 1;
        default:
            return 0;
    }
}

static int OA_ISLITTLE_ENDIAN(int x)
{     
    switch(x) {
        case OA_PIX_FMT_BGGR16LE:
        case OA_PIX_FMT_RGGB16LE:
        case OA_PIX_FMT_GBRG16LE:
        case OA_PIX_FMT_GRBG16LE:
        case OA_PIX_FMT_GREY16LE:
        case OA_PIX_FMT_RGB48LE:
        case OA_PIX_FMT_BGR48LE:
            return 1;
        default:
            return 0;
    }
}

static int OA_ISBAYER8(int x) {
    switch(x) {
        case OA_PIX_FMT_BGGR8:
        case OA_PIX_FMT_RGGB8:
        case OA_PIX_FMT_GBRG8:
        case OA_PIX_FMT_GRBG8:
            return 1;
        default:
            return 0;
    }
}

static int OA_ISBAYER10(int x) {
    switch (x) {
        case OA_PIX_FMT_GRBG10:
        case OA_PIX_FMT_GRBG10P:
            return 1;
        default:
            return 0;
    }
}

static int OA_ISBAYER16(int x) {
    switch (x) {
        case OA_PIX_FMT_BGGR16LE:
        case OA_PIX_FMT_BGGR16BE:

        case OA_PIX_FMT_RGGB16LE:
        case OA_PIX_FMT_RGGB16BE:

        case OA_PIX_FMT_GBRG16LE:
        case OA_PIX_FMT_GBRG16BE:

        case OA_PIX_FMT_GRBG16LE:
        case OA_PIX_FMT_GRBG16BE:
            return 1;
        default:
            return 0;
    }
}

static int OA_ISBAYER(int x)
{
    return OA_ISBAYER8(x)
        || OA_ISBAYER10(x)
        || OA_ISBAYER16(x);
}

static double OA_BYTES_PER_PIXEL(int x) {
    switch(x) {
        case OA_PIX_FMT_BGGR16LE:
        case OA_PIX_FMT_BGGR16BE:
        case OA_PIX_FMT_RGGB16LE:
        case OA_PIX_FMT_RGGB16BE:
        case OA_PIX_FMT_GBRG16LE:
        case OA_PIX_FMT_GBRG16BE:
        case OA_PIX_FMT_GRBG16LE:
        case OA_PIX_FMT_GRBG16BE:
        case OA_PIX_FMT_GREY16BE:
        case OA_PIX_FMT_GREY16LE:
        case OA_PIX_FMT_YUYV:
        case OA_PIX_FMT_YUV422P:
        case OA_PIX_FMT_UYVY:
        case OA_PIX_FMT_GRBG10:
            return 2;

        case OA_PIX_FMT_RGB24:
        case OA_PIX_FMT_BGR24:
        case OA_PIX_FMT_YUV444P:
            return 3;

        case OA_PIX_FMT_YUV411:
            return 4.0/6.0;

        case OA_PIX_FMT_RGB48BE:
        case OA_PIX_FMT_RGB48LE:
        case OA_PIX_FMT_BGR48BE:
        case OA_PIX_FMT_BGR48LE:
            return 6;

        case OA_PIX_FMT_GRBG10P:
            return 1.25;

        default:
            return 1;
    }
}

static int OA_IS_MULTIBYTE_PER_PIXEL(int x)
{
    if (OA_BYTES_PER_PIXEL(x) == 2 ||
        OA_BYTES_PER_PIXEL(x) == 6 ||
        OA_BYTES_PER_PIXEL(x) == 1.25)
      return 1;
    return 0;
}

static int OA_IS_LUM_CHROM(int x) {
    return ( x >= OA_PIX_FMT_YUV444P && x <= OA_PIX_FMT_YUV410 );
}

static int OA_ISRGB(int x) {
    switch(x) {
        case OA_PIX_FMT_RGB24:
        case OA_PIX_FMT_BGR24:
        case OA_PIX_FMT_RGB48BE:
        case OA_PIX_FMT_BGR48BE:
        case OA_PIX_FMT_RGB48LE:
        case OA_PIX_FMT_BGR48LE:
            return 1;
        default:
            return 0;
    }
}

static int OA_ISRB_SWAPPED(int x) {
    switch(x) {
        case OA_PIX_FMT_BGR24:
        case OA_PIX_FMT_BGR48BE:
        case OA_PIX_FMT_BGR48LE:
            return 1;
        default:
            return 0;
    }
}

static int OA_GREYSCALE_FMT(int x) {
  return (OA_BYTES_PER_PIXEL(x) == 1)
        ? OA_PIX_FMT_GREY8
        : OA_PIX_FMT_GREY16LE;
}

static int OA_DEMOSAIC_FMT(int x) {
    switch(x) {
        case OA_PIX_FMT_BGGR8:
        case OA_PIX_FMT_RGGB8:
        case OA_PIX_FMT_GBRG8:
        case OA_PIX_FMT_GRBG8:
            return OA_PIX_FMT_RGB24;
        case OA_PIX_FMT_BGGR16LE:
        case OA_PIX_FMT_RGGB16LE:
        case OA_PIX_FMT_GBRG16LE:
        case OA_PIX_FMT_GRBG16LE:
        case OA_PIX_FMT_GRBG10:
        case OA_PIX_FMT_GRBG10P:
            return OA_PIX_FMT_RGB48LE;
        case OA_PIX_FMT_BGGR16BE:
        case OA_PIX_FMT_RGGB16BE:
        case OA_PIX_FMT_GBRG16BE:
        case OA_PIX_FMT_GRBG16BE:
            return OA_PIX_FMT_RGB48BE;
        default:
            return OA_PIX_FMT_NONE;
    }
}

static int OA_CFA_PATTERN(int x) {
    switch(x) {
        case OA_PIX_FMT_RGGB8:
        case OA_PIX_FMT_RGGB16LE:
        case OA_PIX_FMT_RGGB16BE:
            return OA_DEMOSAIC_RGGB;
        case OA_PIX_FMT_BGGR8:
        case OA_PIX_FMT_BGGR16LE:
        case OA_PIX_FMT_BGGR16BE:
            return OA_DEMOSAIC_BGGR;
        case OA_PIX_FMT_GRBG8:
        case OA_PIX_FMT_GRBG10:
        case OA_PIX_FMT_GRBG10P:
        case OA_PIX_FMT_GRBG16LE:
        case OA_PIX_FMT_GRBG16BE:
            return OA_DEMOSAIC_GRBG;
        case OA_PIX_FMT_GBRG8:
        case OA_PIX_FMT_GBRG16LE:
        case OA_PIX_FMT_GBRG16BE:
            return OA_DEMOSAIC_GBRG;
        default:
            return OA_DEMOSAIC_NONE;
    }
}

static int OA_TO_BAYER8(int x) {
    switch(OA_CFA_PATTERN(x)) {
        case OA_DEMOSAIC_RGGB:
            return OA_PIX_FMT_RGGB8;
        case OA_DEMOSAIC_BGGR:
            return OA_PIX_FMT_BGGR8;
        case OA_DEMOSAIC_GRBG:
            return OA_PIX_FMT_GRBG8;
        case OA_DEMOSAIC_GBRG:
            return OA_PIX_FMT_GBRG8;
        default:
            return OA_PIX_FMT_NONE;
    }
}

static int OA_TO_BAYER16(int x) {
    switch(OA_CFA_PATTERN(x)) {
        case OA_DEMOSAIC_RGGB:
            return OA_PIX_FMT_RGGB16LE;
        case OA_DEMOSAIC_BGGR:
            return OA_PIX_FMT_BGGR16LE;
        case OA_DEMOSAIC_GRBG:
            return OA_PIX_FMT_GRBG16LE;
        case OA_DEMOSAIC_GBRG:
            return OA_PIX_FMT_GBRG16LE;
        default:
            return OA_PIX_FMT_NONE;
    }
}

static int OA_SHOULD_CONVERT_PIX_FMT(int I, int O)
{
    // OUT         IN
    //       RAW  GREY RGB
    // RAW    1    0    1
    // GREY   0    1    0
    // RGB    0    1    1

    if (!O || !I)
        return 0; // error

    if (OA_IS_LUM_CHROM(O))
        return 0; // can't output LUM_CHROME

    if (OA_ISGREYSCALE(O)) {
        if (OA_BYTES_PER_PIXEL(O) == 2 && !OA_ISLITTLE_ENDIAN(O))
            return 0; // simple always outputs LE

        if (OA_ISBAYER(I) || OA_ISRGB(I)) {
            return 1; // can always output GREY, 8 or 16
        }
        if (OA_ISGREYSCALE(I))
            return OA_BYTES_PER_PIXEL(I) >= OA_BYTES_PER_PIXEL(O); // shouldn't output 16 bit grey for 8 bit grey input
    }

    if (OA_ISBAYER(O)) {
        if (OA_BYTES_PER_PIXEL(O) == 2 && !OA_ISLITTLE_ENDIAN(O))
            return 0; // simple always outputs LE, not BE

        if (!OA_ISBAYER(I))
            return 0; // can't output BAYER if input was not BAYER

        if (OA_ISBAYER10(O))
            return 0; // can't output BAYER10

        return (OA_CFA_PATTERN(I) == OA_CFA_PATTERN(O)) // shouldn't convert cfa pattern
            && (ceil(OA_BYTES_PER_PIXEL(I)) >= OA_BYTES_PER_PIXEL(O)); // shouldn't upscale bit depth
    }

    if (OA_ISRGB(O)) {
        if (OA_ISRB_SWAPPED(O))
            return 0; // simple always outputs RGB, not BGR

        if (OA_BYTES_PER_PIXEL(O) == 6 && !OA_ISLITTLE_ENDIAN(O))
            return 0; // simple always outputs LE

        if (OA_ISGREYSCALE(I))
            return 0; // shouldn't convert GREY to RGB

        if (OA_ISBAYER(I)) {
          return (6 == OA_BYTES_PER_PIXEL(O) && (ceil(OA_BYTES_PER_PIXEL(I)) > 1)) // shouldn't upscale bit depth
               || (3 == OA_BYTES_PER_PIXEL(O));
        }
        return OA_BYTES_PER_PIXEL(I) >= OA_BYTES_PER_PIXEL(O); // shouldn't upscale bit depth
    }

    return 0;
}

static int OA_CAN_CONVERT_PIX_FMT(int I, int O)
{
    // FIXME - this might be useful to reduce logic in this function
    // FIXME - might make maintenance easier, oth perhaps more obscure
    //if (OA_SHOULD_CONVERT_PIX_FMT(I,O))
    //    return 1; // recommended conversion is ok
    
    if (!O || !I)
        return 0; // error

    if (O == I)
        return 1; // can always output the input format

    if (OA_IS_LUM_CHROM(O))
        return 0; // can't output LUM_CHROME FIXME

    if (OA_ISGREYSCALE(O))
        return 1; // can always output GREY

    if (OA_ISBAYER(O)) {
        if (!OA_ISBAYER(I))
            return 0; // can't output BAYER if input was not BAYER
            // FIXME is it just a case of subsampling a colour image??

        if (OA_ISBAYER10(O))
            return 0; // can't output BAYER10 FIXME write the transformation

        return 1; // FIXME write the transformation
        // naive transformation is just to pix-shift, perhaps wraparound, duplicate or blank?
    }

    if (OA_ISRGB(O))
        return 1; // FIXME - write GREYtoRGB and XXXtoRGB48

    // should never reach here
    return 0;
}
#endif	/* OPENASTRO_CAMERA_FORMATS_H */
