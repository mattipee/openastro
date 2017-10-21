/*****************************************************************************
 *
 * previewWidget.h -- class declaration
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
#include "imageBuffer.h"

class PreviewWidget : public QFrame
{
  Q_OBJECT

  public:
    			PreviewWidget ( QWidget* parent = 0 );
    			~PreviewWidget();
    void		updatePreviewSize ( void );
    void		configure ( void );
//  void		setBuffer ( QImage* );
    void		setCapturedFramesDisplayInterval ( int );
    void		setEnabled ( int );
    void		setVideoFramePixelFormat ( int );
    void		enableTempDisplay ( int );
    void		enableDroppedDisplay ( int );
    void		enableFlipX ( int );
    void		enableFlipY ( int );
    void		enableDemosaic ( int );
    void		enableScreenUpdates ( int );
    void		setDisplayFPS ( int );
    static void*	updatePreview ( void*, void*, int );
    void		setFirstFrameTime ( void );
    void		beginRecording ( void );
    void		forceRecordingStop ( void );

  public slots:
    void		recentreReticle ( void );
    void		derotateReticle ( void );
    void		setMonoPalette ( QColor );

  protected:
    void		paintEvent ( QPaintEvent* );

  signals:
    void		updateFrameCount ( unsigned int );
    void		updateActualFrameRate ( double );
    void		updateTemperature ( void );
    void		updateDroppedFrames ( void );
    void		updateProgress ( unsigned int );
    void		updateHistogram ( void );
    void		updateDisplay ( void );
    void		stopRecording ( void );
    void		frameWriteFailed ( void );

  private:
    ImageBuffer previewBuffer;
    ImageBuffer writeBuffer;
    QImage		image;
    int			currentZoom;
    int			currentZoomX;
    int			currentZoomY;
    int			capturedFramesDisplayInterval;
    unsigned long	lastCapturedFramesUpdateTime;
    int			frameDisplayInterval;
    unsigned long	lastDisplayUpdateTime;
    int			previewEnabled;
    int			videoFramePixelFormat;
    int			framesInFpsCalcPeriod;
    unsigned long	fpsCalcPeriodStartTime;
    long		secondForTemperature;
    long		secondForDropped;
    int			hasTemp;
    int			hasDroppedFrames;
    int			reticleCentreX;
    int			reticleCentreY;
    int			flipX;
    int			flipY;
    int			demosaic;

    int                 expectedSize;
    int                 screenUpdatesEnabled;
    int                 savedXSize;
    int                 savedYSize;
    int                 lastPointerX;
    int                 lastPointerY;
    int                 movingReticle;
    int                 rotatingReticle;
    int                 diagonalLength;
    qreal		rotationAngle;
    QTransform		rotationTransform;
    int			setNewFirstFrameTime;
    pthread_mutex_t	imageMutex;
    int			recordingInProgress;
    int			manualStop;
    int			focusScore;

    void		mousePressEvent ( QMouseEvent* );
    void		mouseMoveEvent ( QMouseEvent* );
    void		mouseReleaseEvent ( QMouseEvent* );
    void		wheelEvent ( QWheelEvent* );
    void		recalculateDimensions ( int );
    int			formatToCfaPattern ( int );

    QVector<QRgb>	greyscaleColourTable;
    QVector<QRgb>	falseColourTable;
};
