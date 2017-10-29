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
  colourModeLabel = new QLabel("Colour mode");
  colourModeButtons = new QButtonGroup ( this );
  colourButton = new QRadioButton ( tr ( "RGB" ));
  greyButton = new QRadioButton ( tr ( "Greyscale" ));
  rawButton = new QRadioButton ( tr ( "Raw bayer" ));
  colourModeButtons->addButton ( colourButton );
  colourModeButtons->addButton ( greyButton );
  colourModeButtons->addButton ( rawButton );

  bitDepthLabel = new QLabel("Bit depth");
  bitDepthButtons = new QButtonGroup ( this );
  eightBitButton = new QRadioButton ( tr ( "8bit (24bit RGB)" ));
  sixteenBitButton = new QRadioButton ( tr ( "16bit (48bit RGB)" ));
  bitDepthButtons->addButton ( eightBitButton );
  bitDepthButtons->addButton ( sixteenBitButton );

  advancedBox = new QCheckBox("Advanced");
  advancedBox->setChecked(true);
  colourButton->setEnabled(false);
  greyButton->setEnabled(false);
  rawButton->setEnabled(false);
  eightBitButton->setEnabled(false);
  sixteenBitButton->setEnabled(false);




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

  // set values
  int formatIndex = outputFormatMenu->findData(
      target_allowed ? config.targetPixelFormat : config.imagePixelFormat );
  outputFormatMenu->setCurrentIndex ( formatIndex );
  updateDoProcessing( formatIndex ); // set checkbox state and enable demosaic menus





  box = new QVBoxLayout ( this );
  box->addWidget(inputFormatLabel);
  box->addWidget(inputFormatValue);
  box->addSpacing ( 15 );
  box->addWidget(colourModeLabel);
  box->addWidget(colourButton);
  box->addWidget(greyButton);
  box->addWidget(rawButton);
  box->addSpacing ( 15 );
  box->addWidget(bitDepthLabel);
  box->addWidget(eightBitButton);
  box->addWidget(sixteenBitButton);
  box->addSpacing ( 15 );
  box->addWidget(advancedBox);
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


  connect ( colourModeButtons, SIGNAL ( buttonClicked ( int )), this,
      SLOT ( updateDoProcessingSimple(int)));
  connect ( bitDepthButtons, SIGNAL ( buttonClicked ( int )), this,
      SLOT ( updateDoProcessingSimple(int)));
  connect ( advancedBox, SIGNAL ( stateChanged ( int )), this,
      SLOT ( switchSimpleAdvanced(int)));
  connect ( outputFormatMenu, SIGNAL ( currentIndexChanged ( int )), this,
      SLOT ( updateDoProcessing(int)));
  connect ( doDemosaicCheckbox, SIGNAL ( stateChanged ( int )), this,
      SLOT ( selectivelyControlDemosaic( int )));
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

  config.demosaic.cfaPattern = cfaPatternMenu->itemData(
      cfaPatternMenu->currentIndex()).toInt();

  config.demosaic.method = methodMenu->itemData(
      methodMenu->currentIndex()).toInt();
}

void OutputSettings::switchSimpleAdvanced(int state)
{
    const bool switchToSimple = state==0;

    colourButton->setEnabled(switchToSimple);
    greyButton->setEnabled(switchToSimple);
    rawButton->setEnabled(switchToSimple);
    eightBitButton->setEnabled(switchToSimple);
    sixteenBitButton->setEnabled(switchToSimple);

    outputFormatLabel->setVisible(!switchToSimple);
    outputFormatMenu->setVisible(!switchToSimple);
    doDemosaicCheckbox->setVisible(!switchToSimple);
    doGreyscaleCheckbox->setVisible(!switchToSimple);
    cfaLabel->setVisible(!switchToSimple);
    cfaPatternMenu->setVisible(!switchToSimple);
    methodLabel->setVisible(!switchToSimple);
    methodMenu->setVisible(!switchToSimple);

    if (switchToSimple) {
        if (doDemosaicCheckbox->isChecked()) {
            const int current_cfa  = cfaPatternMenu->itemData(cfaPatternMenu->currentIndex()).toInt();
            const int current_method = methodMenu->itemData(methodMenu->currentIndex()).toInt();

            bool warn = false;
            std::string warning = "TODO";
            if (current_cfa != OA_DEMOSAIC_AUTO) {
                warn = true;
                warning += " CFA";
                cfaPatternMenu->setCurrentIndex(cfaPatternMenu->findData( OA_DEMOSAIC_AUTO ));
            }
            if (current_method != OA_DEMOSAIC_NEAREST_NEIGHBOUR) {
                warn = true;
                warning += " METHOD";
                methodMenu->setCurrentIndex(methodMenu->findData( OA_DEMOSAIC_NEAREST_NEIGHBOUR ));
            }

            if (warn) {
                QMessageBox::warning ( this, APPLICATION_NAME, warning.c_str());
            }
        }
    }
}

void OutputSettings::updateDoProcessingSimple(int index)
{
    if (colourButton->isChecked()) {
        if (sixteenBitButton->isChecked()) {
                QMessageBox::warning ( this, APPLICATION_NAME, "Can't output 16bit colour, reverting to 8bit");
                eightBitButton->setChecked(true);
        }
    }

    const bool colour = colourButton->isChecked();
    const bool grey = greyButton->isChecked();
    const bool raw = rawButton->isChecked();
    const bool eightBit = eightBitButton->isChecked();

    //TODO calculate correct output format
    int output_format = 0;
    if (colour && eightBit)
        output_format = OA_PIX_FMT_RGB24;
    if (grey && eightBit)
        output_format = OA_PIX_FMT_GREY8;
    if (grey && !eightBit)
        output_format = OA_PIX_FMT_GREY16LE;
    if (raw && eightBit)
        output_format = OA_TO_BAYER8(config.imagePixelFormat);
    if (raw && !eightBit)
        output_format = OA_TO_BAYER16(config.imagePixelFormat);

    int outputFormatIndex = outputFormatMenu->findData(output_format);
    std::cerr << "Simple implies output format " << OA_PIX_FMT_STRING(output_format) << "@" << outputFormatIndex;
    outputFormatMenu->setCurrentIndex(outputFormatIndex);
    updateDoProcessing(outputFormatIndex);
}

void OutputSettings::updateDoProcessing(int index)
{
    int output_format = outputFormatMenu->itemData(index).toInt();

    const bool canDoDemosaic = OA_ISBAYER(config.imagePixelFormat) && !OA_ISBAYER(output_format);
    const bool mustDoDemosaic = canDoDemosaic && OA_ISRGB(output_format);
    const bool doDemosaic = mustDoDemosaic ? true : (canDoDemosaic ? config.demosaic.demosaicOutput : false);
  
    const bool mustDoGreyscale = OA_ISGREYSCALE(output_format) && !OA_ISGREYSCALE(config.imagePixelFormat);

    cfaPatternMenu->setEnabled(doDemosaic);
    
    cfaPatternMenu->setCurrentIndex ( cfaPatternMenu->findData(
        (!OA_ISBAYER(config.imagePixelFormat) || OA_ISBAYER(output_format))
        ? OA_DEMOSAIC_AUTO
        : config.demosaic.cfaPattern ));

    methodMenu->setEnabled(doDemosaic);
    methodMenu->setCurrentIndex ( methodMenu->findData( config.demosaic.method ) );

    doDemosaicCheckbox->setChecked(doDemosaic);
    doDemosaicCheckbox->setEnabled(canDoDemosaic && !mustDoDemosaic);

    doGreyscaleCheckbox->setChecked(mustDoGreyscale);
    doGreyscaleCheckbox->setEnabled(false);



    colourButton->setEnabled(canDoDemosaic || OA_ISRGB(config.imagePixelFormat));
    colourButton->setChecked(OA_ISRGB(output_format));

    greyButton->setChecked(OA_ISGREYSCALE(output_format));
    greyButton->setEnabled(true);

    rawButton->setEnabled(canDoDemosaic);
    rawButton->setChecked(OA_ISBAYER(output_format));

    eightBitButton->setChecked(OA_BYTES_PER_PIXEL(output_format) == 1 ||
                               OA_BYTES_PER_PIXEL(output_format) == 3);
    sixteenBitButton->setChecked(OA_BYTES_PER_PIXEL(output_format) == 2 ||
                               OA_BYTES_PER_PIXEL(output_format) == 6);

    switchSimpleAdvanced(advancedBox->isChecked());



    // first time updateDoProcessing is called is to populate
    // after that, it gets called on change of output format
    // first time, dummyForceDataChanged hasn't been connected
    // so we don't call dataChanged, so next line is a nop.
    dummyForceDataChanged->setChecked(!dummyForceDataChanged->isChecked());
}

void OutputSettings::selectivelyControlDemosaic( int doDemosaic )
{
    config.demosaic.demosaicOutput = doDemosaic;

    cfaPatternMenu->setEnabled( doDemosaic );
    methodMenu->setEnabled( doDemosaic ); 

    dummyForceDataChanged->setChecked(!dummyForceDataChanged->isChecked());
}
