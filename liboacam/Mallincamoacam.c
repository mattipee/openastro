/*****************************************************************************
 *
 * Mallincamoacam.c -- main entrypoint for Mallincam cameras
 *
 * Copyright 2016,2017 James Fidell (james@openastroproject.org)
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

#if HAVE_LIBDL
#if HAVE_DLFCN_H
#include <dlfcn.h>
#endif
#endif
#include <openastro/camera.h>
#include <toupcam.h>

#include "oacamprivate.h"
#include "unimplemented.h"
#include "Mallincamoacam.h"

// Pointers to libmallincam functions so we can use them via libdl.

const char*	( *p_Mallincam_Version )();
unsigned	( *p_Mallincam_Enum )( ToupcamInst* );
HToupCam	( *p_Mallincam_Open )( const char* );
HToupCam	( *p_Mallincam_OpenByIndex )( unsigned );
void		( *p_Mallincam_Close )( HToupCam );
HRESULT		( *p_Mallincam_StartPullModeWithCallback )( HToupCam,
		    PTOUPCAM_EVENT_CALLBACK, void* );
HRESULT		( *p_Mallincam_PullImage )( HToupCam, void*, int, unsigned*,
		    unsigned* );
HRESULT		( *p_Mallincam_PullStillImage )( HToupCam, void*, int,
		    unsigned*, unsigned* );
HRESULT		( *p_Mallincam_StartPushMode )( HToupCam,
		    PTOUPCAM_DATA_CALLBACK, void* );
HRESULT		( *p_Mallincam_Stop )( HToupCam );
HRESULT		( *p_Mallincam_Pause )( HToupCam, BOOL );
HRESULT		( *p_Mallincam_Snap )( HToupCam, unsigned );
HRESULT		( *p_Mallincam_Trigger )( HToupCam );
HRESULT		( *p_Mallincam_get_Size )( HToupCam, int*, int* );
HRESULT		( *p_Mallincam_put_Size )( HToupCam, int, int );
HRESULT		( *p_Mallincam_get_eSize )( HToupCam, unsigned* );
HRESULT		( *p_Mallincam_put_eSize )( HToupCam, unsigned );
HRESULT		( *p_Mallincam_get_Resolution )( HToupCam, unsigned, int*,
		    int* );
HRESULT		( *p_Mallincam_get_ResolutionNumber )( HToupCam );
HRESULT		( *p_Mallincam_get_ResolutionRatio )( HToupCam, unsigned, int*,
		    int* );
HRESULT		( *p_Mallincam_get_RawFormat )( HToupCam, unsigned*,
		    unsigned* );
HRESULT		( *p_Mallincam_get_AutoExpoEnable )( HToupCam, BOOL* );
HRESULT		( *p_Mallincam_get_AutoExpoTarget )( HToupCam,
		    unsigned short* );
HRESULT		( *p_Mallincam_put_AutoExpoEnable )( HToupCam, BOOL );
HRESULT		( *p_Mallincam_put_AutoExpoTarget )( HToupCam, unsigned short );
HRESULT		( *p_Mallincam_get_ExpoTime )( HToupCam, unsigned* );
HRESULT		( *p_Mallincam_get_ExpTimeRange )( HToupCam, unsigned*,
		    unsigned*, unsigned* );
HRESULT		( *p_Mallincam_put_ExpoTime )( HToupCam, unsigned );
HRESULT		( *p_Mallincam_put_MaxAutoExpoTimeAGain )( HToupCam, unsigned,
		    unsigned short );
HRESULT		( *p_Mallincam_get_ExpoAGain )( HToupCam, unsigned short* );
HRESULT		( *p_Mallincam_put_ExpoAGain )( HToupCam, unsigned short );
HRESULT		( *p_Mallincam_get_ExpoAGainRange )( HToupCam, unsigned short*,
		    unsigned short*, unsigned short* );
HRESULT		( *p_Mallincam_AwbInit )( HToupCam,
		    PITOUPCAM_WHITEBALANCE_CALLBACK, void* );
HRESULT		( *p_Mallincam_AwbOnePush )( HToupCam,
		    PITOUPCAM_TEMPTINT_CALLBACK, void* );
HRESULT		( *p_Mallincam_get_TempTint )( HToupCam, int*, int* );
HRESULT		( *p_Mallincam_put_TempTint )( HToupCam, int, int );
HRESULT		( *p_Mallincam_get_WhiteBalanceGain )( HToupCam, int[3] );
HRESULT		( *p_Mallincam_put_WhiteBalanceGain )( HToupCam, int[3] );
HRESULT		( *p_Mallincam_get_Hue )( HToupCam, int* );
HRESULT		( *p_Mallincam_put_Hue )( HToupCam, int );
HRESULT		( *p_Mallincam_get_Saturation )( HToupCam, int* );
HRESULT		( *p_Mallincam_put_Saturation )( HToupCam, int );
HRESULT		( *p_Mallincam_get_Brightness )( HToupCam, int* );
HRESULT		( *p_Mallincam_put_Brightness )( HToupCam, int );
HRESULT		( *p_Mallincam_get_Contrast )( HToupCam, int* );
HRESULT		( *p_Mallincam_put_Contrast )( HToupCam, int );
HRESULT		( *p_Mallincam_get_Gamma )( HToupCam, int* );
HRESULT		( *p_Mallincam_put_Gamma )( HToupCam, int );
HRESULT		( *p_Mallincam_get_Chrome )( HToupCam, BOOL* );
HRESULT		( *p_Mallincam_put_Chrome )( HToupCam, BOOL );
HRESULT		( *p_Mallincam_get_VFlip )( HToupCam, BOOL* );
HRESULT		( *p_Mallincam_put_VFlip )( HToupCam, BOOL );
HRESULT		( *p_Mallincam_get_HFlip )( HToupCam, BOOL* );
HRESULT		( *p_Mallincam_put_HFlip )( HToupCam, BOOL );
HRESULT		( *p_Mallincam_get_Negative )( HToupCam, BOOL* );
HRESULT		( *p_Mallincam_put_Negative )( HToupCam, BOOL );
HRESULT		( *p_Mallincam_get_MaxSpeed )( HToupCam );
HRESULT		( *p_Mallincam_get_Speed )( HToupCam, unsigned short* );
HRESULT		( *p_Mallincam_put_Speed )( HToupCam, unsigned short );
HRESULT		( *p_Mallincam_get_MaxBitDepth )( HToupCam );
HRESULT		( *p_Mallincam_get_HZ )( HToupCam, int* );
HRESULT		( *p_Mallincam_put_HZ )( HToupCam, int );
HRESULT		( *p_Mallincam_get_Mode )( HToupCam, BOOL* );
HRESULT		( *p_Mallincam_put_Mode )( HToupCam, BOOL );
HRESULT		( *p_Mallincam_get_AWBAuxRect )( HToupCam, RECT* );
HRESULT		( *p_Mallincam_put_AWBAuxRect )( HToupCam, const RECT* );
HRESULT		( *p_Mallincam_get_AEAuxRect )( HToupCam, RECT* );
HRESULT		( *p_Mallincam_put_AEAuxRect )( HToupCam, const RECT* );
HRESULT		( *p_Mallincam_get_MonoMode )( HToupCam );
HRESULT		( *p_Mallincam_get_StillResolution )( HToupCam, unsigned, int*,
		    int* );
HRESULT		( *p_Mallincam_get_StillResolutionNumber )( HToupCam );
HRESULT		( *p_Mallincam_get_RealTime )( HToupCam, BOOL* );
HRESULT		( *p_Mallincam_put_RealTime )( HToupCam, BOOL );
HRESULT		( *p_Mallincam_Flush )( HToupCam );
HRESULT		( *p_Mallincam_get_Temperature )( HToupCam, short* );
HRESULT		( *p_Mallincam_put_Temperature )( HToupCam, short );
HRESULT		( *p_Mallincam_get_SerialNumber )( HToupCam, char[32] );
HRESULT		( *p_Mallincam_get_FwVersion )( HToupCam, char[16] );
HRESULT		( *p_Mallincam_get_HwVersion )( HToupCam, char[16] );
HRESULT		( *p_Mallincam_get_ProductionDate )( HToupCam, char[10] );
HRESULT		( *p_Mallincam_get_LevelRange )( HToupCam, unsigned short[4],
		    unsigned short[4] );
HRESULT		( *p_Mallincam_put_LevelRange )( HToupCam, unsigned short[4],
		    unsigned short[4] );
HRESULT		( *p_Mallincam_put_ExpoCallback )( HToupCam,
		    PITOUPCAM_EXPOSURE_CALLBACK, void* );
HRESULT		( *p_Mallincam_put_ChromeCallback )( HToupCam,
		    PITOUPCAM_CHROME_CALLBACK, void* );
HRESULT		( *p_Mallincam_LevelRangeAuto )( HToupCam );
HRESULT		( *p_Mallincam_GetHistogram )( HToupCam,
		    PITOUPCAM_HISTOGRAM_CALLBACK, void* );
HRESULT		( *p_Mallincam_put_LEDState )( HToupCam, unsigned short,
		    unsigned short, unsigned short );
HRESULT		( *p_Mallincam_read_EEPROM )( HToupCam, unsigned,
		    unsigned char*, unsigned );
HRESULT		( *p_Mallincam_write_EEPROM )( HToupCam, unsigned,
		    const unsigned char*, unsigned );
HRESULT		( *p_Mallincam_get_Option )( HToupCam, unsigned, unsigned* );
HRESULT		( *p_Mallincam_put_Option )( HToupCam, unsigned, unsigned );
HRESULT		( *p_Mallincam_get_Roi )( HToupCam, unsigned*, unsigned* );
// This one doesn't exist in libmallincam, even thought get_Roi does
//HRESULT		( *p_Mallincam_put_Roi )( HToupCam, unsigned, unsigned,
//		    unsigned, unsigned );
HRESULT		( *p_Mallincam_ST4PlusGuide )( HToupCam, unsigned, unsigned );
HRESULT		( *p_Mallincam_ST4PlusGuideState )( HToupCam );
double		( *p_Mallincam_calc_ClarityFactor )( const void*, int,
		    unsigned, unsigned );
void		( *p_Mallincam_deBayer )( unsigned, int, int, const void*,
		    void*, unsigned char );
void		( *p_Mallincam_HotPlug )( PTOUPCAM_HOTPLUG, void* );

// These are apparently obsolete
//
// Mallincam_get_RoiMode
// Mallincam_put_RoiMode
// Mallincam_get_VignetAmountInt
// Mallincam_get_VignetEnable
// Mallincam_get_VignetMidPointInt
// Mallincam_put_VignetAmountInt
// Mallincam_put_VignetEnable
// Mallincam_put_VignetMidPointInt

// And these are not documented as far as I can see
// Mallincam_get_FanMaxSpeed
// Mallincam_get_Field
// Mallincam_get_PixelSize
// Mallincam_read_UART
// Mallincam_write_UART

static void*		_getDLSym ( void*, const char* );

/**
 * Cycle through the list of cameras returned by the mallincam library
 */

int
oaMallincamGetCameras ( CAMERA_LIST* deviceList, int flags )
{
  ToupcamInst		devList[ TOUPCAM_MAX ];
  unsigned int		numCameras;
  unsigned int		i;
  oaCameraDevice*       dev;
  DEVICE_INFO*		_private;
  int                   ret;
  char			libraryPath[ PATH_MAX+1 ];
#ifdef DYNLIB_EXT_DYLIB
  const char*		libName = "MacOS/libmallincam.dylib";
#else
  const char*		libName = "libmallincam.so";
#endif

  static void*		libHandle = 0;

  *libraryPath = 0;
  if ( installPathRoot ) {
    ( void ) strncpy ( libraryPath, installPathRoot, PATH_MAX );
    ( void ) strncat ( libraryPath, "/", PATH_MAX );
  }
  ( void ) strncat ( libraryPath, libName, PATH_MAX );

  if ( !libHandle ) {
    if (!( libHandle = dlopen ( libraryPath, RTLD_LAZY ))) {
#ifndef DYNLIB_EXT_DYLIB
      // We can try the library installed with the Mallincam binaries directly,
      // just to see if it is there
      if (!( libHandle = dlopen ( "/usr/local/MALLINCAMLITE/libmallincam.so",
          RTLD_LAZY ))) 
#endif
        // fprintf ( stderr, "can't load %s\n", libraryPath );
        return 0;
    }
  }

  dlerror();

  if (!( *( void** )( &p_Mallincam_AwbInit ) = _getDLSym ( libHandle,
      "Toupcam_AwbInit" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_AwbOnePush ) = _getDLSym ( libHandle,
      "Toupcam_AwbOnePush" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_calc_ClarityFactor ) = _getDLSym ( libHandle,
      "Toupcam_calc_ClarityFactor" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_Close ) = _getDLSym ( libHandle,
      "Toupcam_Close" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_deBayer ) = _getDLSym ( libHandle,
      "Toupcam_deBayer" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_Enum ) = _getDLSym ( libHandle,
      "Toupcam_Enum" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_Flush ) = _getDLSym ( libHandle,
      "Toupcam_Flush" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_AEAuxRect ) = _getDLSym ( libHandle,
      "Toupcam_get_AEAuxRect" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_AutoExpoEnable ) = _getDLSym ( libHandle,
      "Toupcam_get_AutoExpoEnable" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_AutoExpoTarget ) = _getDLSym ( libHandle,
      "Toupcam_get_AutoExpoTarget" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_AWBAuxRect ) = _getDLSym ( libHandle,
      "Toupcam_get_AWBAuxRect" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Brightness ) = _getDLSym ( libHandle,
      "Toupcam_get_Brightness" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Chrome ) = _getDLSym ( libHandle,
      "Toupcam_get_Chrome" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Contrast ) = _getDLSym ( libHandle,
      "Toupcam_get_Contrast" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_eSize ) = _getDLSym ( libHandle,
      "Toupcam_get_eSize" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_ExpoAGain ) = _getDLSym ( libHandle,
      "Toupcam_get_ExpoAGain" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_ExpoAGainRange ) = _getDLSym ( libHandle,
      "Toupcam_get_ExpoAGainRange" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_ExpoTime ) = _getDLSym ( libHandle,
      "Toupcam_get_ExpoTime" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_ExpTimeRange ) = _getDLSym ( libHandle,
      "Toupcam_get_ExpTimeRange" ))) {
    return 0;
  }

  /*
  if (!( *( void** )( &p_Mallincam_get_FanMaxSpeed ) = _getDLSym ( libHandle,
      "Toupcam_get_FanMaxSpeed" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Field ) = _getDLSym ( libHandle,
      "Toupcam_get_Field" ))) {
    return 0;
  }
   */

  if (!( *( void** )( &p_Mallincam_get_FwVersion ) = _getDLSym ( libHandle,
      "Toupcam_get_FwVersion" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Gamma ) = _getDLSym ( libHandle,
      "Toupcam_get_Gamma" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_HFlip ) = _getDLSym ( libHandle,
      "Toupcam_get_HFlip" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_GetHistogram ) = _getDLSym ( libHandle,
      "Toupcam_GetHistogram" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Hue ) = _getDLSym ( libHandle,
      "Toupcam_get_Hue" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_HwVersion ) = _getDLSym ( libHandle,
      "Toupcam_get_HwVersion" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_HZ ) = _getDLSym ( libHandle,
      "Toupcam_get_HZ" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_LevelRange ) = _getDLSym ( libHandle,
      "Toupcam_get_LevelRange" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_MaxBitDepth ) = _getDLSym ( libHandle,
      "Toupcam_get_MaxBitDepth" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_MaxSpeed ) = _getDLSym ( libHandle,
      "Toupcam_get_MaxSpeed" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Mode ) = _getDLSym ( libHandle,
      "Toupcam_get_Mode" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_MonoMode ) = _getDLSym ( libHandle,
      "Toupcam_get_MonoMode" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Negative ) = _getDLSym ( libHandle,
      "Toupcam_get_Negative" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Option ) = _getDLSym ( libHandle,
      "Toupcam_get_Option" ))) {
    return 0;
  }

  /*
  if (!( *( void** )( &p_Mallincam_get_PixelSize ) = _getDLSym ( libHandle,
      "Toupcam_get_PixelSize" ))) {
    return 0;
  }
   */

  if (!( *( void** )( &p_Mallincam_get_ProductionDate ) = _getDLSym ( libHandle,
      "Toupcam_get_ProductionDate" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_RawFormat ) = _getDLSym ( libHandle,
      "Toupcam_get_RawFormat" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_RealTime ) = _getDLSym ( libHandle,
      "Toupcam_get_RealTime" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Resolution ) = _getDLSym ( libHandle,
      "Toupcam_get_Resolution" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_ResolutionNumber ) = _getDLSym ( libHandle,
      "Toupcam_get_ResolutionNumber" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_ResolutionRatio ) = _getDLSym ( libHandle,
      "Toupcam_get_ResolutionRatio" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Roi ) = _getDLSym ( libHandle,
      "Toupcam_get_Roi" ))) {
    return 0;
  }

  /*
  if (!( *( void** )( &p_Mallincam_get_RoiMode ) = _getDLSym ( libHandle,
      "Toupcam_get_RoiMode" ))) {
    return 0;
  }
   */

  if (!( *( void** )( &p_Mallincam_get_Saturation ) = _getDLSym ( libHandle,
      "Toupcam_get_Saturation" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_SerialNumber ) = _getDLSym ( libHandle,
      "Toupcam_get_SerialNumber" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Size ) = _getDLSym ( libHandle,
      "Toupcam_get_Size" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Speed ) = _getDLSym ( libHandle,
      "Toupcam_get_Speed" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_StillResolution ) = _getDLSym ( libHandle,
      "Toupcam_get_StillResolution" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_StillResolutionNumber ) = _getDLSym (
      libHandle, "Toupcam_get_StillResolutionNumber" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_Temperature ) = _getDLSym ( libHandle,
      "Toupcam_get_Temperature" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_TempTint ) = _getDLSym ( libHandle,
      "Toupcam_get_TempTint" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_VFlip ) = _getDLSym ( libHandle,
      "Toupcam_get_VFlip" ))) {
    return 0;
  }

  /*
  if (!( *( void** )( &p_Mallincam_get_VignetAmountInt ) = _getDLSym ( libHandle,
      "Toupcam_get_VignetAmountInt" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_VignetEnable ) = _getDLSym ( libHandle,
      "Toupcam_get_VignetEnable" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_get_VignetMidPointInt ) = _getDLSym (
      libHandle, "Toupcam_get_VignetMidPointInt" ))) {
    return 0;
  }
   */

  if (!( *( void** )( &p_Mallincam_get_WhiteBalanceGain ) = _getDLSym ( libHandle,
      "Toupcam_get_WhiteBalanceGain" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_HotPlug ) = _getDLSym ( libHandle,
      "Toupcam_HotPlug" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_LevelRangeAuto ) = _getDLSym ( libHandle,
      "Toupcam_LevelRangeAuto" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_Open ) = _getDLSym ( libHandle,
      "Toupcam_Open" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_OpenByIndex ) = _getDLSym ( libHandle,
      "Toupcam_OpenByIndex" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_Pause ) = _getDLSym ( libHandle,
      "Toupcam_Pause" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_PullImage ) = _getDLSym ( libHandle,
      "Toupcam_PullImage" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_PullStillImage ) = _getDLSym ( libHandle,
      "Toupcam_PullStillImage" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_AEAuxRect ) = _getDLSym ( libHandle,
      "Toupcam_put_AEAuxRect" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_AutoExpoEnable ) = _getDLSym ( libHandle,
      "Toupcam_put_AutoExpoEnable" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_AutoExpoTarget ) = _getDLSym ( libHandle,
      "Toupcam_put_AutoExpoTarget" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_AWBAuxRect ) = _getDLSym ( libHandle,
      "Toupcam_put_AWBAuxRect" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Brightness ) = _getDLSym ( libHandle,
      "Toupcam_put_Brightness" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Chrome ) = _getDLSym ( libHandle,
      "Toupcam_put_Chrome" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_ChromeCallback ) = _getDLSym ( libHandle,
      "Toupcam_put_ChromeCallback" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Contrast ) = _getDLSym ( libHandle,
      "Toupcam_put_Contrast" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_eSize ) = _getDLSym ( libHandle,
      "Toupcam_put_eSize" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_ExpoAGain ) = _getDLSym ( libHandle,
      "Toupcam_put_ExpoAGain" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_ExpoCallback ) = _getDLSym ( libHandle,
      "Toupcam_put_ExpoCallback" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_ExpoTime ) = _getDLSym ( libHandle,
      "Toupcam_put_ExpoTime" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Gamma ) = _getDLSym ( libHandle,
      "Toupcam_put_Gamma" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_HFlip ) = _getDLSym ( libHandle,
      "Toupcam_put_HFlip" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Hue ) = _getDLSym ( libHandle,
      "Toupcam_put_Hue" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_HZ ) = _getDLSym ( libHandle,
      "Toupcam_put_HZ" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_LEDState ) = _getDLSym ( libHandle,
      "Toupcam_put_LEDState" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_LevelRange ) = _getDLSym ( libHandle,
      "Toupcam_put_LevelRange" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_MaxAutoExpoTimeAGain ) = _getDLSym (
      libHandle, "Toupcam_put_MaxAutoExpoTimeAGain" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Mode ) = _getDLSym ( libHandle,
      "Toupcam_put_Mode" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Negative ) = _getDLSym ( libHandle,
      "Toupcam_put_Negative" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Option ) = _getDLSym ( libHandle,
      "Toupcam_put_Option" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_RealTime ) = _getDLSym ( libHandle,
      "Toupcam_put_RealTime" ))) {
    return 0;
  }

  /*
  if (!( *( void** )( &p_Mallincam_put_Roi ) = _getDLSym ( libHandle,
      "Toupcam_put_Roi" ))) {
    return 0;
  }
   */

  /*
  if (!( *( void** )( &p_Mallincam_put_RoiMode ) = _getDLSym ( libHandle,
      "Toupcam_put_RoiMode" ))) {
    return 0;
  }
   */

  if (!( *( void** )( &p_Mallincam_put_Saturation ) = _getDLSym ( libHandle,
      "Toupcam_put_Saturation" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Size ) = _getDLSym ( libHandle,
      "Toupcam_put_Size" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Speed ) = _getDLSym ( libHandle,
      "Toupcam_put_Speed" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_Temperature ) = _getDLSym ( libHandle,
      "Toupcam_put_Temperature" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_TempTint ) = _getDLSym ( libHandle,
      "Toupcam_put_TempTint" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_VFlip ) = _getDLSym ( libHandle,
      "Toupcam_put_VFlip" ))) {
    return 0;
  }

  /*
  if (!( *( void** )( &p_Mallincam_put_VignetAmountInt ) = _getDLSym ( libHandle,
      "Toupcam_put_VignetAmountInt" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_VignetEnable ) = _getDLSym ( libHandle,
      "Toupcam_put_VignetEnable" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_put_VignetMidPointInt ) = _getDLSym (
      libHandle, "Toupcam_put_VignetMidPointInt" ))) {
    return 0;
  }
   */

  if (!( *( void** )( &p_Mallincam_put_WhiteBalanceGain ) = _getDLSym ( libHandle,
      "Toupcam_put_WhiteBalanceGain" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_read_EEPROM ) = _getDLSym ( libHandle,
      "Toupcam_read_EEPROM" ))) {
    return 0;
  }

  /*
  if (!( *( void** )( &p_Mallincam_read_UART ) = _getDLSym ( libHandle,
      "Toupcam_read_UART" ))) {
    return 0;
  }
   */

  if (!( *( void** )( &p_Mallincam_Snap ) = _getDLSym ( libHandle,
      "Toupcam_Snap" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_ST4PlusGuide ) = _getDLSym ( libHandle,
      "Toupcam_ST4PlusGuide" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_ST4PlusGuideState ) = _getDLSym ( libHandle,
      "Toupcam_ST4PlusGuideState" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_StartPullModeWithCallback ) = _getDLSym (
      libHandle, "Toupcam_StartPullModeWithCallback" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_StartPushMode ) = _getDLSym ( libHandle,
      "Toupcam_StartPushMode" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_Stop ) = _getDLSym ( libHandle,
      "Toupcam_Stop" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_Trigger ) = _getDLSym ( libHandle,
      "Toupcam_Trigger" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_Version ) = _getDLSym ( libHandle,
      "Toupcam_Version" ))) {
    return 0;
  }

  if (!( *( void** )( &p_Mallincam_write_EEPROM ) = _getDLSym ( libHandle,
      "Toupcam_write_EEPROM" ))) {
    return 0;
  }

  /*
  if (!( *( void** )( &p_Mallincam_write_UART ) = _getDLSym ( libHandle,
      "Mallincam_write_UART" ))) {
    return 0;
  }
   */

  numCameras = ( p_Mallincam_Enum )( devList );
  if ( numCameras < 1 ) {
    return 0;
  }

  for ( i = 0; i < numCameras; i++ ) {

    if (!( dev = malloc ( sizeof ( oaCameraDevice )))) {
      return -OA_ERR_MEM_ALLOC;
    }

    if (!( _private = malloc ( sizeof ( DEVICE_INFO )))) {
      free (( void* ) dev );
      return -OA_ERR_MEM_ALLOC;
    }

    _oaInitCameraDeviceFunctionPointers ( dev );
    dev->interface = OA_CAM_IF_MALLINCAM;
    ( void ) strncpy ( dev->deviceName, devList[i].displayname,
        OA_MAX_NAME_LEN+1 );
    _private->devIndex = i;
    ( void ) strcpy ( _private->toupcamId, devList[i].id );
    dev->_private = _private;
    dev->initCamera = oaMallincamInitCamera;
    dev->hasLoadableFirmware = 0;
    if (( ret = _oaCheckCameraArraySize ( deviceList )) < 0 ) {
      free (( void* ) dev );
      free (( void* ) _private );
      return ret;
    }
    deviceList->cameraList[ deviceList->numCameras++ ] = dev;
  }

  return numCameras;
}


static void*
_getDLSym ( void* libHandle, const char* symbol )
{
  void* addr;
  char* error;

  addr = dlsym ( libHandle, symbol );
  if (( error = dlerror())) {
    fprintf ( stderr, "libmallincam DL error: %s\n", error );
    addr = 0;
  }

  return addr;
}
