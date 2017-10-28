/*****************************************************************************
 *
 * outputSettings.cc -- class for output format settings in the settings UI
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

extern "C" {
#include <openastro/demosaic.h>
}

#include "outputSettings.h"

#include "configuration.h"
#include "state.h"

OutputSettings::OutputSettings ( QWidget* parent ) : QWidget ( parent )
{
  inputFormatLabel = new QLabel("Input format");
  inputFormatValue = new QLabel(OA_PIX_FMT_STRING(config.imagePixelFormat));
  outputFormatLabel = new QLabel("Select output format");

  outputFormatMenu = new QComboBox(this);
  int* allowed_output_formats = OA_ALLOWED_OUTPUT_PIX_FMT(config.imagePixelFormat);
  bool target_allowed = false;
  for (int i=OA_PIX_FMT_NONE+1; i<OA_PIX_FMT_MAX; ++i)
  {
      if (allowed_output_formats[i]) {
          target_allowed |= (i == config.targetPixelFormat);
          outputFormatMenu->addItem ( OA_PIX_FMT_STRING(i), QVariant ( i ) );
      }
  }
  doDemosaicCheckbox = new QCheckBox("Do demosaic?", this);
  doGreyscaleCheckbox = new QCheckBox("Convert to greyscale?", this);
  dummyForceDataChanged = new QCheckBox("HACK!!!!");

  cfaLabel = new QLabel ( tr ( "Bayer format" ));
  cfaPatternMenu = new QComboBox(this);
  cfaPatternMenu->addItem( tr ( "Auto" ), QVariant ( OA_DEMOSAIC_AUTO ));
  cfaPatternMenu->addItem( QIcon ( ":/qt-icons/RGGB.png" ), tr ( "RGGB" ), QVariant ( OA_DEMOSAIC_RGGB ));
  cfaPatternMenu->addItem( QIcon ( ":/qt-icons/BGGR.png" ), tr ( "BGGR" ), QVariant ( OA_DEMOSAIC_BGGR ));
  cfaPatternMenu->addItem( QIcon ( ":/qt-icons/GRBG.png" ), tr ( "GRBG" ), QVariant ( OA_DEMOSAIC_GRBG ));
  cfaPatternMenu->addItem( QIcon ( ":/qt-icons/GBRG.png" ), tr ( "GBRG" ), QVariant ( OA_DEMOSAIC_GRBG ));

  methodLabel = new QLabel ( tr ( "Demosaic method" ));
  methodMenu = new QComboBox(this);
  methodMenu->addItem( tr ( "Nearest Neighbour" ), QVariant ( OA_DEMOSAIC_NEAREST_NEIGHBOUR ));
  methodMenu->addItem( tr ( "Bilinear" ), QVariant ( OA_DEMOSAIC_BILINEAR ));
  methodMenu->addItem( tr ( "Smooth Hue" ), QVariant ( OA_DEMOSAIC_SMOOTH_HUE ));
  methodMenu->addItem( tr ( "VNG" ), QVariant ( OA_DEMOSAIC_VNG ));



  box = new QVBoxLayout ( this );
  box->addWidget(inputFormatLabel);
  box->addWidget(inputFormatValue);
  box->addSpacing ( 15 );
  box->addWidget(outputFormatLabel);
  box->addWidget(outputFormatMenu);
  box->addSpacing ( 15 );
  box->addWidget(doDemosaicCheckbox);
  box->addWidget(doGreyscaleCheckbox);
  box->addSpacing ( 15 );
  box->addWidget ( cfaLabel );
  box->addWidget ( cfaPatternMenu );
  box->addSpacing ( 15 );
  box->addWidget ( methodLabel );
  box->addWidget ( methodMenu );
  box->addStretch ( 1 );

  setLayout ( box );


  // set values
  outputFormatMenu->setCurrentIndex ( outputFormatMenu->findData(
      target_allowed ? config.targetPixelFormat : config.imagePixelFormat ) );
  updateDoProcessing(-1); // don't call SettingsWidget::dataChanged



  connect ( outputFormatMenu, SIGNAL ( currentIndexChanged ( int )), this,
      SLOT ( updateDoProcessing(int)));
  connect ( doDemosaicCheckbox, SIGNAL ( stateChanged ( int )), this,
      SLOT ( selectivelyControlDemosaic()));
  connect ( cfaPatternMenu, SIGNAL ( currentIndexChanged ( int )), parent,
      SLOT ( dataChanged()));
  connect ( methodMenu, SIGNAL ( currentIndexChanged ( int )), parent,
      SLOT ( dataChanged()));
  connect ( dummyForceDataChanged, SIGNAL ( stateChanged ( int )), parent,
      SLOT ( dataChanged()));


}


OutputSettings::~OutputSettings()
{
  state.mainWindow->destroyLayout (( QLayout* ) box );
}


void
OutputSettings::storeSettings ( void )
{
  config.targetPixelFormat = outputFormatMenu->itemData(
      outputFormatMenu->currentIndex()).toInt();
  state.mainWindow->updateImagePixelFormat();

  config.demosaic.demosaicOutput = doDemosaicCheckbox->isChecked() ? 1 : 0;
  //config.greyscale = doGreyscaleCheckbox->isChecked() ? 1 : 0;

  config.demosaic.cfaPattern = cfaPatternMenu->itemData(
      cfaPatternMenu->currentIndex()).toInt();

  config.demosaic.method = methodMenu->itemData(
      methodMenu->currentIndex()).toInt();
}


void OutputSettings::updateDoProcessing(int index)
{
    const bool dataChanged = -1 != index;

    int output_format = outputFormatMenu->itemData(outputFormatMenu->currentIndex()).toInt();

    const bool canDoDemosaic = OA_ISBAYER(config.imagePixelFormat) && !OA_ISBAYER(output_format);
    const bool mustDoDemosaic = canDoDemosaic && OA_ISRGB(output_format);
    const bool doDemosaic = mustDoDemosaic ? true : (canDoDemosaic ? config.demosaic.demosaicOutput : false);
  
    const bool mustDoGreyscale = OA_ISGREYSCALE(output_format) && !OA_ISGREYSCALE(config.imagePixelFormat);

    cfaPatternMenu->setEnabled(doDemosaic);
    
    cfaPatternMenu->setCurrentIndex ( cfaPatternMenu->findData(
        (!OA_ISBAYER(config.imagePixelFormat) || OA_ISBAYER(output_format))
        ? OA_DEMOSAIC_AUTO
        : config.demosaic.cfaPattern ));
    /*
    if (!OA_ISBAYER(config.imagePixelFormat) || OA_ISBAYER(output_format)) {
      cfaPatternMenu->setCurrentIndex ( cfaPatternMenu->findData( OA_DEMOSAIC_AUTO ) )
    } else {
      cfaPatternMenu->setCurrentIndex ( cfaPatternMenu->findData( config.demosaic.cfaPattern ) )
    }
    */

    methodMenu->setEnabled(doDemosaic);
    methodMenu->setCurrentIndex ( methodMenu->findData( config.demosaic.method ) );

    doDemosaicCheckbox->setChecked(doDemosaic);
    doDemosaicCheckbox->setEnabled(canDoDemosaic && !mustDoDemosaic);

    doGreyscaleCheckbox->setChecked(mustDoGreyscale);
    doGreyscaleCheckbox->setEnabled(false);

    if (dataChanged)
        dummyForceDataChanged->setChecked(!dummyForceDataChanged->isChecked());
}

void OutputSettings::selectivelyControlDemosaic(void)
{
    int output_format = outputFormatMenu->itemData(outputFormatMenu->currentIndex()).toInt();

    const bool doDemosaic = doDemosaicCheckbox->isChecked();
  
    cfaPatternMenu->setEnabled( doDemosaic );
    methodMenu->setEnabled( doDemosaic ); 

    config.demosaic.demosaicOutput = doDemosaic;

    dummyForceDataChanged->setChecked(!dummyForceDataChanged->isChecked());
}
