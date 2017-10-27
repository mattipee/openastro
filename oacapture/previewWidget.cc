/*****************************************************************************
 *
 * previewWidget.cc -- class for the preview window in the UI (and more)
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

#include <oa_common.h>

#include <QtGui>
#include <cstdlib>
#include <math.h>
#include <pthread.h>

extern "C" {
#include <openastro/camera.h>
#include <openastro/demosaic.h>
#include <openastro/video.h>
#include <openastro/imgproc.h>
#include <openastro/video/formats.h>
}

#include "configuration.h"
#include "previewWidget.h"
#include "captureWidget.h"
#include "outputHandler.h"
#include "histogramWidget.h"
#include "focusOverlay.h"
#include "state.h"


// FIX ME -- Lots of this stuff needs refactoring or placing elsewhere
// as it's really not anything to do with the actual preview window
// any more

PreviewWidget::PreviewWidget ( QWidget* parent ) : QFrame ( parent )
{
  currentZoom = 100;
  int zoomFactor = state.zoomWidget->getZoomFactor();
  // setAttribute( Qt::WA_NoSystemBackground, true );
  lastCapturedFramesUpdateTime = 0;
  capturedFramesDisplayInterval = 200;
  lastDisplayUpdateTime = 0;
  frameDisplayInterval = 1000/15; // display frames per second
  previewEnabled = 1;
  videoFramePixelFormat = OA_PIX_FMT_RGB24;
  framesInFpsCalcPeriod = fpsCalcPeriodStartTime = 0;
  secondForTemperature = secondForDropped = 0;
  flipX = flipY = 0;
  movingReticle = rotatingReticle = rotationAngle = 0;
  savedXSize = savedYSize = 0;
  recalculateDimensions ( zoomFactor );
  expectedSize = config.imageSizeX * config.imageSizeY *
      OA_BYTES_PER_PIXEL( videoFramePixelFormat );
  demosaic = config.demosaic;

  int r = config.currentColouriseColour.red();
  int g = config.currentColouriseColour.green();
  int b = config.currentColouriseColour.blue();
  int cr, cg, cb;
  cr = cg = cb = 0;
  greyscaleColourTable.clear();
  falseColourTable.clear();
  for ( int i = 0; i < 256; i++ ) {
    greyscaleColourTable.append ( QColor ( i, i, i ).rgb());
    falseColourTable.append ( QColor ( cr / 256, cg / 256, cb / 256 ).rgb());
    cr += r;
    cg += g;
    cb += b;
  }

  pthread_mutex_init ( &imageMutex, 0 );
  recordingInProgress = 0;
  manualStop = 0;

  connect ( this, SIGNAL( updateDisplay ( void )),
      this, SLOT( update ( void )));
}


PreviewWidget::~PreviewWidget()
{
}


void
PreviewWidget::configure ( void )
{
  // setGeometry ( 0, 0, config.imageSizeX, config.imageSizeY );
}


void
PreviewWidget::updatePreviewSize ( void )
{
  int zoomFactor = state.zoomWidget->getZoomFactor();
  recalculateDimensions ( zoomFactor );
  expectedSize        = config.imageSizeX * config.imageSizeY *
      OA_BYTES_PER_PIXEL( videoFramePixelFormat ); // full depth
  diagonalLength = sqrt ( config.imageSizeX * config.imageSizeX +
      config.imageSizeY * config.imageSizeY );
}


void
PreviewWidget::recalculateDimensions ( int zoomFactor )
{
  currentZoom = zoomFactor;
  currentZoomX = config.imageSizeX * zoomFactor / 100;
  currentZoomY = config.imageSizeY * zoomFactor / 100;
  if ( savedXSize && savedYSize ) {
    reticleCentreX = ( reticleCentreX * currentZoomX ) / savedXSize;
    reticleCentreY = ( reticleCentreY * currentZoomY ) / savedYSize;
  } else {
    reticleCentreX = currentZoomX / 2;
    reticleCentreY = currentZoomY / 2;
  }
  setGeometry ( 0, 0, currentZoomX, currentZoomY );
  savedXSize = currentZoomX;
  savedYSize = currentZoomY;
  if ( 0 == rotationAngle ) {
    rotationTransform.reset();
  } else {
    rotationTransform = QTransform().translate ( reticleCentreX,
        reticleCentreY ).rotate ( rotationAngle ).translate ( -reticleCentreX,
        -reticleCentreY );
  }
}


void
PreviewWidget::paintEvent ( QPaintEvent* event )
{
  Q_UNUSED( event );

  QPainter painter ( this );

  pthread_mutex_lock ( &imageMutex );
  painter.drawImage ( 0, 0, image );
  pthread_mutex_unlock ( &imageMutex );

  painter.setTransform ( rotationTransform );
  painter.setRenderHint ( QPainter::Antialiasing, true );
  painter.setPen ( QPen ( Qt::red, 2, Qt::SolidLine, Qt::FlatCap ));

  if ( config.showReticle ) {
    switch ( config.reticleStyle ) {

      case RETICLE_CIRCLE:
        painter.drawEllipse ( reticleCentreX - 50, reticleCentreY - 50,
          100, 100 );
        painter.drawEllipse ( reticleCentreX - 100, reticleCentreY - 100,
          200, 200 );
        painter.drawEllipse ( reticleCentreX - 200, reticleCentreY - 200,
          400, 400 );
        painter.drawLine ( reticleCentreX, reticleCentreY - 5,
          reticleCentreX, reticleCentreY + 5 );
        painter.drawLine ( reticleCentreX - 5, reticleCentreY,
          reticleCentreX + 5, reticleCentreY );
        break;

      case RETICLE_CROSS:
        // drawing lines at least the diagonal length long will mean
        // they always run to the edge of the displayed image.  The
        // widget will crop the lines as appropriate
        painter.drawLine ( reticleCentreX, reticleCentreY - diagonalLength,
            reticleCentreX, reticleCentreY - 10 );
        painter.drawLine ( reticleCentreX, reticleCentreY + diagonalLength,
            reticleCentreX, reticleCentreY + 10 );
        painter.drawLine ( reticleCentreX - diagonalLength, reticleCentreY,
            reticleCentreX - 10, reticleCentreY );
        painter.drawLine ( reticleCentreX + diagonalLength, reticleCentreY,
            reticleCentreX + 10, reticleCentreY );
        break;

      case RETICLE_TRAMLINES:
        // see comments for cross
        painter.drawLine ( reticleCentreX - 10,
            reticleCentreY - diagonalLength, reticleCentreX - 10,
            reticleCentreY + diagonalLength );
        painter.drawLine ( reticleCentreX + 10,
            reticleCentreY - diagonalLength, reticleCentreX + 10,
            reticleCentreY + diagonalLength );
        painter.drawLine ( reticleCentreX - diagonalLength,
            reticleCentreY - 10, reticleCentreX + diagonalLength,
            reticleCentreY - 10 );
        painter.drawLine ( reticleCentreX - diagonalLength,
            reticleCentreY + 10, reticleCentreX + diagonalLength,
            reticleCentreY + 10 );
        break;
    }
  }
}


void
PreviewWidget::mousePressEvent ( QMouseEvent* event )
{
  if ( event->button() == Qt::LeftButton ) {
    lastPointerX = event->x();
    lastPointerY = event->y();
    if (( abs ( lastPointerX - reticleCentreX ) < 20 ) &&
        ( abs ( lastPointerY - reticleCentreY ) < 20 )) {
      movingReticle = 1;
    }
  }
}


void
PreviewWidget::mouseMoveEvent ( QMouseEvent* event )
{
  int x = event->x();
  int y = event->y();
  if ( movingReticle ) {
    reticleCentreX += ( x - lastPointerX );
    reticleCentreY += ( y - lastPointerY );
  }
  lastPointerX = x;
  lastPointerY = y;
}

void
PreviewWidget::mouseReleaseEvent ( QMouseEvent* event )
{
  if ( event->button() == Qt::LeftButton ) {
    movingReticle = rotatingReticle = 0;
  }
}


void
PreviewWidget::wheelEvent ( QWheelEvent* event )
{
  int change = event->delta();
  int direction = ( change > 0 ) - ( change < 0 );
  if ( !direction ) {
    event->ignore();
  } else {
    int x = event->x();
    qreal scale = ( qreal ) abs ( x - reticleCentreX ) / 50;
    rotationAngle += direction * scale;
    if ( 0 == rotationAngle ) {
      rotationTransform.reset();
    } else {
      rotationTransform = QTransform().translate ( reticleCentreX,
          reticleCentreY ).rotate ( rotationAngle ).translate ( -reticleCentreX,
          -reticleCentreY );
    }
    event->accept();
  }
}


void
PreviewWidget::recentreReticle ( void )
{
  reticleCentreX = currentZoomX / 2;
  reticleCentreY = currentZoomY / 2;
}


void
PreviewWidget::derotateReticle ( void )
{
  rotationTransform.reset();
  rotationAngle = 0;
}


void
PreviewWidget::setCapturedFramesDisplayInterval ( int millisecs )
{
  capturedFramesDisplayInterval = millisecs;
}


void
PreviewWidget::setEnabled ( int state )
{
  previewEnabled = state;
}


void
PreviewWidget::setVideoFramePixelFormat ( int format )
{
  videoFramePixelFormat = format;
  expectedSize = config.imageSizeX * config.imageSizeY *
      OA_BYTES_PER_PIXEL( videoFramePixelFormat );
  state.mainWindow->setPixelFormatValue( format );
  config.imagePixelFormat = format; // hack to communicate to test code in demosaicSettings

  int* allowed_output_formats = OA_ALLOWED_OUTPUT_PIX_FMT(format);
  std::cerr << "Input format: " << OA_PIX_FMT_STRING(format) << "\n";
  std::cerr << "Output formats: ";
  for (int i=OA_PIX_FMT_NONE; i<OA_PIX_FMT_MAX; ++i)
  {
    if (allowed_output_formats[i]) std::cerr << OA_PIX_FMT_STRING(i) << " ";
  }
  std::cerr << "\n";
}


void
PreviewWidget::enableTempDisplay ( int state )
{
  hasTemp = state;
}


void
PreviewWidget::enableDroppedDisplay ( int state )
{
  hasDroppedFrames = state;
}


void
PreviewWidget::enableFlipX ( int state )
{
  flipX = state;
}


void
PreviewWidget::enableFlipY ( int state )
{
  flipY = state;
}


void
PreviewWidget::enableDemosaic ( int state )
{
  demosaic = state;
}


void
PreviewWidget::setDisplayFPS ( int fps )
{
  frameDisplayInterval = 1000 / fps;
}


// FIX ME -- could combine this with beginRecording() ?
void
PreviewWidget::setFirstFrameTime ( void )
{
  setNewFirstFrameTime = 1;
}


void
PreviewWidget::beginRecording ( void )
{
  recordingInProgress = 1;
}


void
PreviewWidget::forceRecordingStop ( void )
{
  manualStop = 1;
}


void
PreviewWidget::setMonoPalette ( QColor colour )
{
  unsigned int r = colour.red();
  unsigned int g = colour.green();
  unsigned int b = colour.blue();
  unsigned int cr, cg, cb;
  cr = cg = cb = 0;
  QRgb rgb;
  for ( int i = 0; i < 256; i++ ) {
    // This looks odd, but the shifts make sense because we actually
    // want ( <colour-component> / 256 ) shifted into the right place
    rgb = 0xff000000 | ((  cr & 0xff00 ) << 8 ) | ( cg & 0xff00 ) | ( cb >> 8 );
    falseColourTable[i] = rgb;
    cr += r;
    cg += g;
    cb += b;
  } 
}


void*
PreviewWidget::updatePreview ( void* args, void* imageData, int length )
{
  STATE*		state = ( STATE* ) args;
  PreviewWidget*	self = state->previewWidget;
  struct timeval	t;
  int			doDisplay = 0;
  int			doHistogram = 0;

  // don't do anything if the length is not as expected
  if ( length != self->expectedSize ) {
    // qWarning() << "size mismatch.  have:" << length << " expected: "
    //    << self->expectedSize;
    return 0;
  }



  ( void ) gettimeofday ( &t, 0 );
  unsigned long now = ( unsigned long ) t.tv_sec * 1000 +
      ( unsigned long ) t.tv_usec / 1000;

  int cfaPattern = config.cfaPattern;
  if ( OA_ISBAYER ( self->videoFramePixelFormat )) {
    if ( OA_DEMOSAIC_AUTO == cfaPattern ) {
      cfaPattern = self->formatToCfaPattern ( self->videoFramePixelFormat );
    }
  }

  // Preview Image
  self->previewBuffer.reset(imageData, self->videoFramePixelFormat,
                            config.imageSizeX, config.imageSizeY);

  if ( self->previewEnabled ) {
    if (( self->lastDisplayUpdateTime + self->frameDisplayInterval ) < now ) {
      self->lastDisplayUpdateTime = now;
      doDisplay = 1;

      // FIXME: I've kept 16bit for testing boost at 16bit
      //        ensure 8/24 just before creating new QImage
      //        this might be a poor choice re: performance
      //        but it's just for testing...


      // we can flip the preview image here if required
      if (OA_BYTES_PER_PIXEL(self->previewBuffer.getPixelFormat()) == 1.25)
      {
        //FIXME... greyscale() will change pixelFormat to greyscale
        // actually just want to convert to the correct BAYER format
        self->previewBuffer.greyscale(OA_GREYSCALE_FMT(self->previewBuffer.getPixelFormat()));
      }


      self->previewBuffer.flip(self->flipX, self->flipY);

      // Demosaic the preview
      if ( self->demosaic && config.demosaicPreview ) {
        self->previewBuffer.ensure8BitGreyOrRaw(); // FIXME can't demosaic 10/16bit GREY
        self->previewBuffer.demosaic(cfaPattern, config.demosaicMethod);
      }

      // Convert to greyscale (either original, or demosaicked if that happened)
      if ( config.greyscale ) {
        self->previewBuffer.greyscale(OA_GREYSCALE_FMT(self->previewBuffer.getPixelFormat()));
      }

      // Boost preview image
      if ( config.boost.enable ) {
        self->previewBuffer.boost(config.boost.stretch, config.boost.sharpen,
            config.boost.multiply, config.boost.gamma, config.boost.algorithm );
      }

      // Show focus aid
      if ( config.showFocusAid ) {
        int fmt = OA_PIX_FMT_RGB24; // TODO check... ever RGB48?

        // This call should be thread-safe
        state->focusOverlay->addScore ( oaFocusScore ( self->previewBuffer.write_buffer(),
            0, config.imageSizeX, config.imageSizeY, fmt ));
      }

      
      // Convert GREY16 to GREY8 or BAYERXX to BAYER8 (or RGB48XX to RGB24??) for preview
      // Convert YUV to RGB24, (or RGB48XX to RGB24 TODO) for preview
      self->previewBuffer.ensure8BitGreyOrRaw();
      self->previewBuffer.ensure24BitRGB();


      QImage* newImage;
      QImage* swappedImage = 0;

      if ( self->previewBuffer.is8bit() ) {
        newImage = new QImage (( const uint8_t* ) self->previewBuffer.read_buffer(),
            self->previewBuffer.width(), self->previewBuffer.height(),
            self->previewBuffer.width(), QImage::Format_Indexed8 );

        newImage->setColorTable(( OA_PIX_FMT_GREY8 == self->previewBuffer.getPixelFormat() &&
                                  config.colourise ) ? self->falseColourTable
                                                     : self->greyscaleColourTable);
        swappedImage = newImage;
      } else {
        // Need the stride size here or QImage appears to "tear" the
        // right hand edge of the image when the X dimension is an odd
        // number of pixels
        newImage = new QImage (( const uint8_t* ) self->previewBuffer.read_buffer(),
            self->previewBuffer.width(), self->previewBuffer.height(),
            self->previewBuffer.width() * 3, QImage::Format_RGB888 );
        if ( OA_PIX_FMT_BGR24 == self->previewBuffer.getPixelFormat() ) {
          swappedImage = new QImage ( newImage->rgbSwapped());
        } else {
          swappedImage = newImage;
        }
      }

      // This call should be thread-safe
      int zoomFactor = state->zoomWidget->getZoomFactor();
      if ( zoomFactor && zoomFactor != self->currentZoom ) {
        self->recalculateDimensions ( zoomFactor );
      }

      if ( self->currentZoom != 100 || self->previewBuffer.width() != config.imageSizeX
                                    || self->previewBuffer.height() != config.imageSizeY ) {
        QImage scaledImage = swappedImage->scaled ( self->currentZoomX,
          self->currentZoomY );

        if ( config.showFocusAid ) {
        }

        pthread_mutex_lock ( &self->imageMutex );
        self->image = scaledImage.copy();
        pthread_mutex_unlock ( &self->imageMutex );
      } else {
        pthread_mutex_lock ( &self->imageMutex );
        self->image = swappedImage->copy();
        pthread_mutex_unlock ( &self->imageMutex );
      }
      if ( swappedImage != newImage ) {
        delete swappedImage;
      }
      delete newImage;
    }
  }


  // Output Image
  self->writeBuffer.reset(imageData, self->videoFramePixelFormat,
          config.imageSizeX, config.imageSizeY);

  OutputHandler* output = 0;
  if ( !state->pauseEnabled ) {
    // This should be thread-safe
    output = state->captureWidget->getOutputHandler();
    if ( output && self->recordingInProgress ) {
      if ( self->setNewFirstFrameTime ) {
        state->firstFrameTime = now;
        self->setNewFirstFrameTime = 0;
      }
      state->lastFrameTime = now;


      // Try flipping the image if required
      // this is going to make a mess for data we intend to demosaic.
      // the user will have to deal with that
      if ( self->flipX || self->flipY )
      {
        // We may not be able to flip the image that is going to be written out as
        // some formats don't have a flip routine written yet, eg 10bit bayer, YUV, RGB48
        // FIX ME - work this out some time
        self->writeBuffer.flip(self->flipX, self->flipY);
      }

      // Demosaic the output frame
      if ( config.demosaicOutput ) {
        // If it's possible that the write CFA pattern is not the same
        // as the preview one, this code will need fixing to reset
        // cfaPattern, but I can't see that such a thing is possible
        // at the moment
        self->writeBuffer.demosaic(cfaPattern, config.demosaicMethod);
      }

      // Convert output frame to greyscale
      if (config.greyscale) {
        self->writeBuffer.greyscale();
      }

      // These calls should be thread-safe
      const char* timestamp = ( state->timer->isInitialised()
                                && state->timer->isRunning() )
          ? state->timer->readTimestamp() : 0;

      // TODO TODO
      // stopped short of being a const crusader here... really should be
      // passing const pointer to addFrame, but it's a virtual function to
      // all of the output formats, so didn't bother.
      // here, we just get a non-const buffer... this is fine if we've already
      // meddled with it, otherwise it's going to incur a copy just to satisfy
      // the interface.
      // I am a big fan of const.
      if ( output->addFrame ( self->writeBuffer.write_buffer(), timestamp,
          // This call should be thread-safe
          state->controlWidget->getCurrentExposure()) < 0 ) {
        self->recordingInProgress = 0;
        self->manualStop = 0;
        state->autorunEnabled = 0;
        emit self->stopRecording();
        emit self->frameWriteFailed();
      } else {
        if (( self->lastCapturedFramesUpdateTime +
            self->capturedFramesDisplayInterval ) < now ) {
          emit self->updateFrameCount ( output->getFrameCount());
          self->lastCapturedFramesUpdateTime = now;
        }
      }
    }
  }

  self->framesInFpsCalcPeriod++;
  if ( self->framesInFpsCalcPeriod &&
      now - self->fpsCalcPeriodStartTime > 1000 ) {
    double fpsCalcPeriodDuration_s = (now - self->fpsCalcPeriodStartTime)/1000.0;
    emit self->updateActualFrameRate (self->framesInFpsCalcPeriod / fpsCalcPeriodDuration_s);

    self->fpsCalcPeriodStartTime = now;
    self->framesInFpsCalcPeriod = 0;

    if ( state->histogramOn ) {
      // This call should be thread-safe

      // If recording, output image will already have been modified
      // demosaic() will return without modification if not a bayer format
      if ( config.demosaicOutput ) {
        self->writeBuffer.demosaic(cfaPattern, config.demosaicMethod);
      }
      if (config.greyscale) {
        self->writeBuffer.greyscale();
      }

      state->histogramWidget->process ( self->writeBuffer.read_buffer(), self->writeBuffer.frameLength(),
          self->writeBuffer.getPixelFormat() );
      doHistogram = 1;
    }
  }

  if ( self->hasTemp && t.tv_sec != self->secondForTemperature &&
      t.tv_sec % 5 == 0 ) {
    emit self->updateTemperature();
    self->secondForTemperature = t.tv_sec;
  }
  if ( self->hasDroppedFrames && t.tv_sec != self->secondForDropped &&
      t.tv_sec % 2 == 0 ) {
    emit self->updateDroppedFrames();
    self->secondForTemperature = t.tv_sec;
  }

  if ( doDisplay ) {
    emit self->updateDisplay();
  }

  // check histogram control here just in case it got changed
  // this ought to be done rather more safely
  if ( state->histogramOn && state->histogramWidget && doHistogram ) {
    emit self->updateHistogram();
  }

  if ( self->manualStop ) {
    self->recordingInProgress = 0;
    emit self->stopRecording();
    self->manualStop = 0;
  }

  if ( output && self->recordingInProgress ) {
    if ( config.limitEnabled ) {
      int finished = 0;
      float percentage = 0;
      int frames = output->getFrameCount();
      switch ( config.limitType ) {
        case 0: // FIX ME -- nasty magic number
          // start and current times here are in ms, but the limit value is in
          // secs, so rather than ( current - start ) / time * 100 to get the
          // %age, we do ( current - start ) / time / 10
          percentage = ( now - state->captureWidget->recordingStartTime ) /
              ( config.secondsLimitValue * 1000.0 +
              state->captureWidget->totalTimePaused ) * 100.0;
          if ( now > state->captureWidget->recordingEndTime ) {
            finished = 1;
          }
          break;
        case 1: // FIX ME -- nasty magic number
          percentage = ( 100.0 * frames ) / config.framesLimitValue;
          if ( frames >= config.framesLimitValue ) {
            finished = 1;
          }
          break;
      }
      if ( finished ) {
        // need to stop now, even if we don't know what's happening
        // with the UI
        self->recordingInProgress = 0;
        self->manualStop = 0;
        emit self->stopRecording();
        // these two are really just tidying up the display
        emit self->updateProgress ( 100 );
        emit self->updateFrameCount ( frames );
        if ( state->autorunEnabled ) {
          // returns non-zero if more runs are left
          // This call is thread-safe because the called function is aware
          // that the GUI changes it makes must be done indirectly
          // FIX ME -- doing it with invokeMethod would be nicer though
          if ( state->captureWidget->singleAutorunFinished()) {
            state->autorunStartNext = now + 1000 * config.autorunDelay;
          }
        }
      } else {
        emit self->updateProgress (( unsigned int ) percentage );
      }
    }
  }

  if ( state->autorunEnabled && state->autorunStartNext &&
      now > state->autorunStartNext ) {
    state->autorunStartNext = 0;
    // Have to do it this way rather than calling direct to ensure
    // thread-safety
    QMetaObject::invokeMethod ( state->captureWidget, "startNewAutorun",
        Qt::BlockingQueuedConnection );
  }

  return 0;
}


int
PreviewWidget::formatToCfaPattern ( int format )
{
  switch ( format ) {
    case OA_PIX_FMT_BGGR8:
    case OA_PIX_FMT_BGGR16LE:
    case OA_PIX_FMT_BGGR16BE:
      return OA_DEMOSAIC_BGGR;
      break;
    case OA_PIX_FMT_RGGB8:
    case OA_PIX_FMT_RGGB16LE:
    case OA_PIX_FMT_RGGB16BE:
      return OA_DEMOSAIC_RGGB;
      break;
    case OA_PIX_FMT_GBRG8:
    case OA_PIX_FMT_GBRG16LE:
    case OA_PIX_FMT_GBRG16BE:
      return OA_DEMOSAIC_GBRG;
      break;
    case OA_PIX_FMT_GRBG8:
    case OA_PIX_FMT_GRBG10:
    case OA_PIX_FMT_GRBG10P:
    case OA_PIX_FMT_GRBG16LE:
    case OA_PIX_FMT_GRBG16BE:
      return OA_DEMOSAIC_GRBG;
      break;
  }
  qWarning() << "Invalid format (" << OA_PIX_FMT_STRING(format) <<
      ") in" << __FUNCTION__;
  return 0;
}
