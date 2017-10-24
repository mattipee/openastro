/*****************************************************************************
 *
 * config.h -- declaration of data structures for configuration data
 *
 * Copyright 2013,2014,2015,2016,2017 James Fidell (james@openastroproject.org)
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

#if HAVE_LIMITS_H
#include <limits.h>
#endif

extern "C" {
#include <openastro/camera.h>
#include <openastro/filterwheel.h>
#include <openastro/userConfig.h>
}

#define	CONFIG_VERSION	7

typedef struct {
  QString	filterName;
  int		controls[OA_CAM_CTRL_MODIFIERS_P1][ OA_CAM_CTRL_LAST_P1 ];
  int		intervalMenuOption;
} FILTER_PROFILE;

typedef struct {
  QString       profileName;
  int           sixteenBit;
  int           binning2x2;
  int           rawMode;
  int           colourise;
  int           useROI;
  unsigned int  imageSizeX;
  unsigned int  imageSizeY;
  QList<FILTER_PROFILE> filterProfiles;
  int           frameRateNumerator;
  int           frameRateDenominator;
  int           fileTypeOption;
  int           filterOption;
  int           limitEnabled;
  int           secondsLimitValue;
  int           framesLimitValue;
  int           limitType;
  QString       fileNameTemplate;
  int		target;
} PROFILE;


// overkill, but i may want to expand this later
typedef struct {
  QString	filterName;
} FILTER;

typedef QList<userDeviceConfig> userConfigList;

typedef struct
{
  // general
  int			saveSettings;
  int			tempsInC;
  int			connectSoleCamera;
  int			dockableControls;
  int			controlsOnRight;
  int			separateControls;

  // settings from device menu
  int			cameraDevice;

  // options menu
  int			showHistogram;
  int			autoAlign;
  int			alignBox;
  int			autoGuide;
  int			showReticle;
  int			showFocusAid;
  int			cutout;
  int			darkFrame;
  int			derotate;
  int			flipX;
  int			flipY;
  int			demosaic;
  int			greyscale;
  struct boost {
    enum { MUL_X1 = 1, MUL_X2 = 2, MUL_X4 = 4, MUL_X8 = 8,
           MUL_X16 = 16, MUL_X32 = 32};
    enum { GAMMA_0_25 = 25, GAMMA_0_5 = 50, GAMMA_0_75 = 75, GAMMA_1_0 = 100,
           GAMMA_1_5 = 150, GAMMA_2_0 = 200 };
    enum { ALGO_NONE = 0,
           ALGO_BIN2X2, ALGO_BIN3X3, ALGO_BIN4X4,
           ALGO_AVG2X2, ALGO_AVG3X3, ALGO_AVG4X4,
           ALGO_dBIN2X2, ALGO_dBIN3X3, ALGO_dBIN4X4,
           ALGO_dAVG2X2, ALGO_dAVG3X3, ALGO_dAVG4X4,
           ALGO_ADPB4,
           ALGO_ADPB6,
           ALGO_ADPB8,
           NUM_ALGOS };
    enum { SHARPEN_NONE, SHARPEN_SOFT, SHARPEN_HARD };

    int         enable;
    int         stretch;
    int         sharpen;
    int         multiply;
    int         gamma;
    int         algorithm;
  }	boost;

  // camera config
  int			sixteenBit;
  int			binning2x2;
  int			rawMode;
  int			colourise;
  QColor		currentColouriseColour;
  int			numCustomColours;
  QList<QColor>		customColours;

  // image config
  int			useROI;
  unsigned int		imageSizeX;
  unsigned int		imageSizeY;

  // zoom config
  int			zoomButton1Option;
  int			zoomButton2Option;
  int			zoomButton3Option;
  int			zoomValue;

  // control config
  int64_t		controlValues[OA_CAM_CTRL_MODIFIERS_P1][ OA_CAM_CTRL_LAST_P1 ];
  int			exposureMenuOption;
  int			frameRateNumerator;
  int			frameRateDenominator;
  int			selectableControl[2];
  int			intervalMenuOption;

  // capture config
  int			profileOption;
  int			filterOption;
  int			fileTypeOption;
  int			limitEnabled;
  int			secondsLimitValue;
  int			framesLimitValue;
  int			limitType;
  QString		fileNameTemplate;
  QString		captureDirectory;
  int			autorunCount;
  int			autorunDelay;
  int			saveCaptureSettings;
  int			windowsCompatibleAVI;
  int			useUtVideo;
  int			indexDigits;

  // display config
  int			preview;
  int			nightMode;
  int			displayFPS;

  // reticle config
  int			reticleStyle;

  // histogram config
  int			splitHistogram;
  int			histogramOnTop;

  // demosaic config
  int			demosaicPreview;
  int			demosaicOutput;
  int			cfaPattern;
  int			demosaicMethod;

  // saved profiles
  int			numProfiles;
  QList<PROFILE>	profiles;

  // filters
  int			numFilters;
  QList<FILTER>		filters;
  int			filterSlots[MAX_FILTER_SLOTS];
  QList<int>		autorunFilterSequence;
  int			promptForFilterChange;
  int			interFilterDelay;

  // advanced user configuration

  QList<userConfigList>	filterWheelConfig;
  QList<userConfigList>	timerConfig;

  // FITS keyword data
  QString               fitsObserver;
  QString               fitsTelescope;
  QString               fitsInstrument;
  QString               fitsObject;
  QString               fitsComment;
  QString               fitsFocalLength;
  QString               fitsApertureDia;
  QString               fitsApertureArea;
  QString               fitsPixelSizeX;
  QString               fitsPixelSizeY;
  QString               fitsSubframeOriginX;
  QString               fitsSubframeOriginY;
  QString               fitsSiteLatitude;
  QString               fitsSiteLongitude;
  QString               fitsFilter;

  // Timer configuration
  int			timerEnabled;
  int			timerMode;
  int			triggerInterval;
  int			userDrainDelayEnabled;
  int			drainDelay;
  int			timestampDelay;
  int			queryGPSForEachCapture;

} CONFIG;

extern CONFIG		config;

#define CONTROL_VALUE(c)	controlValues[OA_CAM_CTRL_MODIFIER(c)][OA_CAM_CTRL_MODE_BASE(c)]

#define	SET_PROFILE_CONTROL(c,v) if ( config.profileOption >= 0 ) config.profiles[ config.profileOption ].filterProfiles[ config.filterOption ].controls[OA_CAM_CTRL_MODIFIER(c)][OA_CAM_CTRL_MODE_BASE(c)] = v

#define	SET_PROFILE_INTERVAL(v) if ( config.profileOption >= 0 ) config.profiles[ config.profileOption ].filterProfiles[ config.filterOption ].intervalMenuOption = v

#define SET_PROFILE_CONFIG(n,v) if ( config.profileOption >= 0 ) config.profiles[config.profileOption].n = v
