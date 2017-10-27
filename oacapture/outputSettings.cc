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
  cfaButtons = new QButtonGroup ( this );
  rggbButton = new QRadioButton ( tr ( "RGGB" ));
  bggrButton = new QRadioButton ( tr ( "BGGR" ));
  grbgButton = new QRadioButton ( tr ( "GRBG" ));
  gbrgButton = new QRadioButton ( tr ( "GBRG" ));
  autoButton = new QRadioButton ( tr ( "Auto" ));
  rggbButton->setIcon ( QIcon ( ":/qt-icons/RGGB.png" ));
  bggrButton->setIcon ( QIcon ( ":/qt-icons/BGGR.png" ));
  grbgButton->setIcon ( QIcon ( ":/qt-icons/GRBG.png" ));
  gbrgButton->setIcon ( QIcon ( ":/qt-icons/GBRG.png" ));
  autoButton->setChecked(true);

  cfaButtons->addButton ( rggbButton );
  cfaButtons->addButton ( bggrButton );
  cfaButtons->addButton ( grbgButton );
  cfaButtons->addButton ( gbrgButton );
  cfaButtons->addButton ( autoButton );

  methodLabel = new QLabel ( tr ( "Demosaic method" ));
  methodButtons = new QButtonGroup ( this );
  nnButton = new QRadioButton ( tr ( "Nearest Neighbour" ));
  bilinearButton = new QRadioButton ( tr ( "Bilinear" ));
  smoothHueButton = new QRadioButton ( tr ( "Smooth Hue" ));
  vngButton = new QRadioButton ( tr ( "VNG" ));
  nnButton->setChecked ( config.demosaic.method ==
      OA_DEMOSAIC_NEAREST_NEIGHBOUR ? 1 : 0 );
  bilinearButton->setChecked ( config.demosaic.method ==
      OA_DEMOSAIC_BILINEAR ? 1 : 0 );
  smoothHueButton->setChecked ( config.demosaic.method ==
      OA_DEMOSAIC_SMOOTH_HUE ? 1 : 0 );
  vngButton->setChecked ( config.demosaic.method ==
      OA_DEMOSAIC_VNG ? 1 : 0 );

  methodButtons->addButton ( nnButton );
  methodButtons->addButton ( bilinearButton );
  methodButtons->addButton ( smoothHueButton );
  methodButtons->addButton ( vngButton );




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
  box->addWidget ( rggbButton );
  box->addWidget ( bggrButton );
  box->addWidget ( grbgButton );
  box->addWidget ( gbrgButton );
  box->addWidget ( autoButton );
  box->addSpacing ( 15 );
  box->addWidget ( methodLabel );
  box->addWidget ( nnButton );
  box->addWidget ( bilinearButton );
  box->addWidget ( smoothHueButton );
  box->addWidget ( vngButton );
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
  connect ( cfaButtons, SIGNAL ( buttonClicked ( int )), parent,
      SLOT ( dataChanged()));
  connect ( methodButtons, SIGNAL ( buttonClicked ( int )), parent,
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

  if ( rggbButton->isChecked()) {
    config.demosaic.cfaPattern = OA_DEMOSAIC_RGGB;
  }
  if ( bggrButton->isChecked()) {
    config.demosaic.cfaPattern = OA_DEMOSAIC_BGGR;
  }
  if ( grbgButton->isChecked()) {
    config.demosaic.cfaPattern = OA_DEMOSAIC_GRBG;
  }
  if ( gbrgButton->isChecked()) {
    config.demosaic.cfaPattern = OA_DEMOSAIC_GBRG;
  }
  if ( autoButton->isChecked()) {
    config.demosaic.cfaPattern = OA_DEMOSAIC_AUTO;
  }
  if ( nnButton->isChecked()) {
    config.demosaic.method = OA_DEMOSAIC_NEAREST_NEIGHBOUR;
  }
  if ( bilinearButton->isChecked()) {
    config.demosaic.method = OA_DEMOSAIC_BILINEAR;
  }
  if ( smoothHueButton->isChecked()) {
    config.demosaic.method = OA_DEMOSAIC_SMOOTH_HUE;
  }
  if ( vngButton->isChecked()) {
    config.demosaic.method = OA_DEMOSAIC_VNG;
  }
}


void
OutputSettings::loadSettings ( void )
{
  
  rggbButton->setChecked ( config.demosaic.cfaPattern == OA_DEMOSAIC_RGGB ? 1 : 0 );
  bggrButton->setChecked ( config.demosaic.cfaPattern == OA_DEMOSAIC_BGGR ? 1 : 0 );
  grbgButton->setChecked ( config.demosaic.cfaPattern == OA_DEMOSAIC_GRBG ? 1 : 0 );
  gbrgButton->setChecked ( config.demosaic.cfaPattern == OA_DEMOSAIC_GBRG ? 1 : 0 );
  autoButton->setChecked ( config.demosaic.cfaPattern == OA_DEMOSAIC_AUTO ? 1 : 0 );
}

void OutputSettings::updateDoProcessing(int index)
{
    const bool dataChanged = -1 != index;

    int output_format = outputFormatMenu->itemData(outputFormatMenu->currentIndex()).toInt();

    const bool canDoDemosaic = OA_ISBAYER(config.imagePixelFormat) && !OA_ISBAYER(output_format);
    const bool mustDoDemosaic = canDoDemosaic && OA_ISRGB(output_format);
    const bool doDemosaic = mustDoDemosaic ? true : (canDoDemosaic ? config.demosaic.demosaicOutput : false);
  
    const bool mustDoGreyscale = OA_ISGREYSCALE(output_format) && !OA_ISGREYSCALE(config.imagePixelFormat);

    rggbButton->setEnabled(doDemosaic);
    bggrButton->setEnabled(doDemosaic);
    grbgButton->setEnabled(doDemosaic);
    gbrgButton->setEnabled(doDemosaic);
    autoButton->setEnabled(doDemosaic);
    rggbButton->setChecked(false);
    bggrButton->setChecked(false);
    grbgButton->setChecked(false);
    gbrgButton->setChecked(false);
/*
    if (dataChanged && OA_ISBAYER(config.imagePixelFormat) && !OA_ISBAYER(output_format))
    {
        switch(OA_CFA_PATTERN(config.imagePixelFormat)) {
            case OA_DEMOSAIC_RGGB:
                rggbButton->setChecked(true);
                break;
            case OA_DEMOSAIC_BGGR:
                bggrButton->setChecked(true);
                break;
            case OA_DEMOSAIC_GRBG:
                grbgButton->setChecked(true);
                break;
            case OA_DEMOSAIC_GBRG:
                gbrgButton->setChecked(true);
                break;
            default:
                autoButton->setChecked(true);
        }
    } else */if (!OA_ISBAYER(config.imagePixelFormat) || OA_ISBAYER(output_format)) {
      autoButton->setChecked(true);
    } else {
        switch(config.demosaic.cfaPattern) {
            case OA_DEMOSAIC_RGGB:
                rggbButton->setChecked(true);
                break;
            case OA_DEMOSAIC_BGGR:
                bggrButton->setChecked(true);
                break;
            case OA_DEMOSAIC_GRBG:
                grbgButton->setChecked(true);
                break;
            case OA_DEMOSAIC_GBRG:
                gbrgButton->setChecked(true);
                break;
            default:
                autoButton->setChecked(true);
        }
    }

    methodLabel->setEnabled(doDemosaic);
    nnButton->setEnabled(doDemosaic);
    bilinearButton->setEnabled(doDemosaic);
    smoothHueButton->setEnabled(doDemosaic);
    vngButton->setEnabled(doDemosaic);

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
   
    rggbButton->setEnabled(doDemosaic);
    bggrButton->setEnabled(doDemosaic);
    grbgButton->setEnabled(doDemosaic);
    gbrgButton->setEnabled(doDemosaic);
    autoButton->setEnabled(doDemosaic);
    methodLabel->setEnabled(doDemosaic);
    nnButton->setEnabled(doDemosaic);
    bilinearButton->setEnabled(doDemosaic);
    smoothHueButton->setEnabled(doDemosaic);
    vngButton->setEnabled(doDemosaic);

    config.demosaic.demosaicOutput = doDemosaic;

    dummyForceDataChanged->setChecked(!dummyForceDataChanged->isChecked());
}
