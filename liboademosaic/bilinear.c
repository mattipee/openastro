/*****************************************************************************
 *
 * bilinear.c -- bilinear demosaic method
 *
 * Copyright 2013,2014 James Fidell (james@openastroproject.org)
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

#include <oa_common.h>
#include <openastro/demosaic.h>

#include "bilinear.h"


static void
_bilinearRGGB8 ( const void* source, void* target, int xSize, int ySize )
{
  int row, col, lastX, lastY;
  unsigned char* s;
  unsigned char* t;

  // FIX ME -- handle first row/column
  // FIX ME -- handle last row/column

  // odd rows
  // odd pixels, R avg(diagonals), G avg(vert/horiz), B direct copy
  // even pixels, R avg(verticals), G direct copy, B avg(horizontals)
  
  lastX = xSize - 2;
  lastY = ySize - 1;
  for ( row = 1; row < lastY; row += 2 ) {
    s = ( unsigned char* ) source + row * xSize + 1;
    t = ( unsigned char* ) target + ( row * xSize + 1 ) * 3;
    for ( col = 1; col < lastX; col += 2 ) {
      *t++ = ( *( s - xSize - 1 ) + *( s - xSize + 1 ) + *( s + xSize - 1 ) +
          *( s + xSize + 1 )) / 4;  // R
      *t++ = ( *( s - xSize ) + *( s - 1 ) + *( s + 1 ) +
          *( s + xSize )) / 4;  // G
      *t++ = *s;  // B
      s++;
      *t++ = ( *( s - xSize ) + *( s + xSize )) / 2;  // R
      *t++ = *s;  // G
      *t++ = ( *( s - 1 ) + *( s + 1 )) / 2;  // B
      s++;
    }
  }

  // even rows
  // odd pixels, R avg(horizontals), G direct copy, B avg(verticals)
  // even pixels, R direct copy, G avg(vert/horiz), B avg(diagonals)

  for ( row = 2; row < lastY; row += 2 ) {
    s = ( unsigned char* ) source + row * xSize + 1;
    t = ( unsigned char* ) target + ( row * xSize + 1 ) * 3;
    for ( col = 1; col < lastX; col += 2 ) {
      *t++ = ( *( s - 1 ) + *( s + 1 )) / 2;  // R
      *t++ = *s;  // G
      *t++ = ( *( s - xSize ) + *( s + xSize )) / 2;  // B
      s++;
      *t++ = *s;  // R
      *t++ = ( *( s - xSize ) + *( s - 1 ) + *( s + 1 ) +
          *( s + xSize )) / 4;  // G
      *t++ = ( *( s - xSize - 1 ) + *( s - xSize + 1 ) + *( s + xSize - 1 ) +
          *( s + xSize + 1 )) / 4;  // B
      s++;
    }
  }
}


static void
_bilinearBGGR8 ( const void* source, void* target, int xSize, int ySize )
{
  int row, col, lastX, lastY;
  unsigned char* s;
  unsigned char* t;

  // FIX ME -- handle first row/column
  // FIX ME -- handle last row/column

  // odd rows
  // odd pixels, R direct copy, G avg(vert/horiz), B avg(diagonals)
  // even pixels, R avg(horizontals), G direct copy, B avg(verticals)
  
  lastX = xSize - 2;
  lastY = ySize - 1;
  for ( row = 1; row < lastY; row += 2 ) {
    s = ( unsigned char* ) source + row * xSize + 1;
    t = ( unsigned char* ) target + ( row * xSize + 1 ) * 3;
    for ( col = 1; col < lastX; col += 2 ) {
      *t++ = *s;  // R
      *t++ = ( *( s - xSize ) + *( s - 1 ) + *( s + 1 ) +
          *( s + xSize )) / 4;  // G
      *t++ = ( *( s - xSize - 1 ) + *( s - xSize + 1 ) + *( s + xSize - 1 ) +
          *( s + xSize + 1 )) / 4;  // B
      s++;
      *t++ = ( *( s - 1 ) + *( s + 1 )) / 2;  // R
      *t++ = *s;  // G
      *t++ = ( *( s - xSize ) + *( s + xSize )) / 2;  // B
      s++;
    }
  }

  // even rows
  // odd pixels, R avg(verticals), G direct copy, B avg(horizontals)
  // even pixels, R avg(diagonals), G avg(vert/horiz), B direct copy

  for ( row = 2; row < lastY; row += 2 ) {
    s = ( unsigned char* ) source + row * xSize + 1;
    t = ( unsigned char* ) target + ( row * xSize + 1 ) * 3;
    for ( col = 1; col < lastX; col += 2 ) {
      *t++ = ( *( s - xSize ) + *( s + xSize )) / 2;  // R
      *t++ = *s;  // G
      *t++ = ( *( s - 1 ) + *( s + 1 )) / 2;  // B
      s++;
      *t++ = ( *( s - xSize - 1 ) + *( s - xSize + 1 ) + *( s + xSize - 1 ) +
          *( s + xSize + 1 )) / 4;  // R
      *t++ = ( *( s - xSize ) + *( s - 1 ) + *( s + 1 ) +
          *( s + xSize )) / 4;  // G
      *t++ = *s;  // B
      s++;
    }
  }
}


static void
_bilinearGRBG8 ( const void* source, void* target, int xSize, int ySize )
{
  int row, col, lastX, lastY;
  unsigned char* s;
  unsigned char* t;

  // FIX ME -- handle first row/column
  // FIX ME -- handle last row/column

  // odd rows
  // odd pixels, R avg(verticals), G direct copy, B avg(horizontals)
  // even pixels, R avg(diagonals), G avg(vert/horiz), B direct copy
  
  lastX = xSize - 2;
  lastY = ySize - 1;
  for ( row = 1; row < lastY; row += 2 ) {
    s = ( unsigned char* ) source + row * xSize + 1;
    t = ( unsigned char* ) target + ( row * xSize + 1 ) * 3;
    for ( col = 1; col < lastX; col += 2 ) {
      *t++ = ( *( s - xSize ) + *( s + xSize )) / 2;  // R
      *t++ = *s;  // G
      *t++ = ( *( s - 1 ) + *( s + 1 )) / 2;  // B
      s++;
      *t++ = ( *( s - xSize - 1 ) + *( s - xSize + 1 ) + *( s + xSize - 1 ) +
          *( s + xSize + 1 )) / 4;  // R
      *t++ = ( *( s - xSize ) + *( s - 1 ) + *( s + 1 ) +
          *( s + xSize )) / 4;  // G
      *t++ = *s;  // B
      s++;
    }
  }

  // even rows
  // odd pixels, R direct copy, G avg(vert/horiz), B avg(diagonals)
  // even pixels, R avg(horizontals), G direct copy, B avg(verticals)

  for ( row = 2; row < lastY; row += 2 ) {
    s = ( unsigned char* ) source + row * xSize + 1;
    t = ( unsigned char* ) target + ( row * xSize + 1 ) * 3;
    for ( col = 1; col < lastX; col += 2 ) {
      *t++ = *s;  // R
      *t++ = ( *( s - xSize ) + *( s - 1 ) + *( s + 1 ) +
          *( s + xSize )) / 4;  // G
      *t++ = ( *( s - xSize - 1 ) + *( s - xSize + 1 ) + *( s + xSize - 1 ) +
          *( s + xSize + 1 )) / 4;  // B
      s++;
      *t++ = ( *( s - 1 ) + *( s + 1 )) / 2;  // R
      *t++ = *s;  // G
      *t++ = ( *( s - xSize ) + *( s + xSize )) / 2;  // B
      s++;
    }
  }
}


static void
_bilinearGBRG8 ( const void* source, void* target, int xSize, int ySize )
{
  int row, col, lastX, lastY;
  unsigned char* s;
  unsigned char* t;

  // FIX ME -- handle first row/column
  // FIX ME -- handle last row/column

  // odd rows
  // odd pixels, R avg(horizontals), G direct copy, B avg(verticals)
  // even pixels, R direct copy, G avg(vert/horiz), B avg(diagonals)
  
  lastX = xSize - 2;
  lastY = ySize - 1;
  for ( row = 1; row < lastY; row += 2 ) {
    s = ( unsigned char* ) source + row * xSize + 1;
    t = ( unsigned char* ) target + ( row * xSize + 1 ) * 3;
    for ( col = 1; col < lastX; col += 2 ) {
      *t++ = ( *( s - 1 ) + *( s + 1 )) / 2;  // R
      *t++ = *s;  // G
      *t++ = ( *( s - xSize ) + *( s + xSize )) / 2;  // B
      s++;
      *t++ = *s;  // R
      *t++ = ( *( s - xSize ) + *( s - 1 ) + *( s + 1 ) +
          *( s + xSize )) / 4;  // G
      *t++ = ( *( s - xSize - 1 ) + *( s - xSize + 1 ) + *( s + xSize - 1 ) +
          *( s + xSize + 1 )) / 4;  // B
      s++;
    }
  }

  // even rows
  // odd pixels, R avg(diagonals), G avg(vert/horiz), B direct copy
  // even pixels, R avg(verticals), G direct copy, B avg(horizontals)

  for ( row = 2; row < lastY; row += 2 ) {
    s = ( unsigned char* ) source + row * xSize + 1;
    t = ( unsigned char* ) target + ( row * xSize + 1 ) * 3;
    for ( col = 1; col < lastX; col += 2 ) {
      *t++ = ( *( s - xSize - 1 ) + *( s - xSize + 1 ) + *( s + xSize - 1 ) +
          *( s + xSize + 1 )) / 4;  // R
      *t++ = ( *( s - xSize ) + *( s - 1 ) + *( s + 1 ) +
          *( s + xSize )) / 4;  // G
      *t++ = *s;  // B
      s++;
      *t++ = ( *( s - xSize ) + *( s + xSize )) / 2;  // R
      *t++ = *s;  // G
      *t++ = ( *( s - 1 ) + *( s + 1 )) / 2;  // B
      s++;
    }
  }
}


void
oadBilinear ( const void* source, void* target, int xSize, int ySize,
    int bitDepth, int format )
{
  // FIX ME
  if ( bitDepth != 8 ) {
    fprintf ( stderr, "demosaic: %s can only handle 8-bit data\n",
        __FUNCTION__ );
    return;
  }

  switch ( format ) {
    case OA_DEMOSAIC_RGGB:
      _bilinearRGGB8 ( source, target, xSize, ySize );
      break;
    case OA_DEMOSAIC_BGGR:
      _bilinearBGGR8 ( source, target, xSize, ySize );
      break;
    case OA_DEMOSAIC_GRBG:
      _bilinearGRBG8 ( source, target, xSize, ySize );
      break;
    case OA_DEMOSAIC_GBRG:
      _bilinearGBRG8 ( source, target, xSize, ySize );
      break;
  }
}
