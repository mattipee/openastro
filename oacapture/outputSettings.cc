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
/*  advancedBox->setChecked(true);
  colourButton->setEnabled(false);
  greyButton->setEnabled(false);
  rawButton->setEnabled(false);
  eightBitButton->setEnabled(false);
  sixteenBitButton->setEnabled(false);
*/



  inputFormatLabel = new QLabel("Input/output format");
  inputFormatValue = new QLabel(QString(OA_PIX_FMT_STRING(config.imagePixelFormat)).append(" => ").append(OA_PIX_FMT_STRING(config.targetPixelFormat)));
  outputFormatLabel = new QLabel("Select output format");

  outputFormatMenu = new QComboBox(this);

  for (int i=OA_PIX_FMT_NONE+1; i<OA_PIX_FMT_MAX; ++i)
  {
      if (OA_CAN_CONVERT_PIX_FMT(config.imagePixelFormat, i))
          outputFormatMenu->addItem ( OA_PIX_FMT_STRING(i), QVariant ( i ) );
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
  bool canConvert    = OA_CAN_CONVERT_PIX_FMT   (config.imagePixelFormat, config.targetPixelFormat);
  bool shouldConvert = OA_SHOULD_CONVERT_PIX_FMT(config.imagePixelFormat, config.targetPixelFormat);

  int formatIndex = outputFormatMenu->findData(
      canConvert ? config.targetPixelFormat : config.imagePixelFormat );
  outputFormatMenu->setCurrentIndex ( formatIndex );
  advancedFormatChanged( formatIndex ); // set checkbox state and enable demosaic menus
  advancedBox->setChecked(canConvert && !shouldConvert);
  advancedCheckboxClicked(advancedBox->isChecked()); // choose simple/advanced





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
      SLOT ( simpleSettingsClicked(int)));
  connect ( bitDepthButtons, SIGNAL ( buttonClicked ( int )), this,
      SLOT ( simpleSettingsClicked(int)));
  connect ( advancedBox, SIGNAL ( stateChanged ( int )), this,
      SLOT ( advancedCheckboxClicked(int)));
  connect ( outputFormatMenu, SIGNAL ( currentIndexChanged ( int )), this,
      SLOT ( advancedFormatChanged(int)));
  connect ( doDemosaicCheckbox, SIGNAL ( stateChanged ( int )), this,
      SLOT ( advancedDemosaicClicked( int )));
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
  std::cerr << "in storeSettings()\n";

  config.targetPixelFormat = outputFormatMenu->itemData(
      outputFormatMenu->currentIndex()).toInt();
  state.mainWindow->updateImagePixelFormat();

  config.demosaic.demosaicOutput = doDemosaicCheckbox->isChecked() ? 1 : 0;

  config.demosaic.cfaPattern = cfaPatternMenu->itemData(
      cfaPatternMenu->currentIndex()).toInt();

  config.demosaic.method = methodMenu->itemData(
      methodMenu->currentIndex()).toInt();
}










void OutputSettings::advancedDemosaicClicked(int doDemosaic) {
    std::cerr << "in advancedDemosaicClicked " << doDemosaic << "\n";

    cfaPatternMenu->setEnabled( doDemosaic );
    methodMenu->setEnabled( doDemosaic ); 

    dummyForceDataChanged->setChecked(!dummyForceDataChanged->isChecked());
}

void OutputSettings::simpleSettingsClicked(int index) {
    std::cerr << "in simpleSettingsClicked " << index << "\n";

    if (colourButton->isChecked()) {
        if (sixteenBitButton->isChecked()) {
            if ((OA_ISRGB(config.imagePixelFormat) && OA_BYTES_PER_PIXEL(config.imagePixelFormat) == 3)
             || (OA_ISBAYER(config.imagePixelFormat) && OA_BYTES_PER_PIXEL(config.imagePixelFormat) == 1))
            {
                QMessageBox::warning ( this, APPLICATION_NAME, "Can't output 48bit colour, reverting to 24bit");
                eightBitButton->setChecked(true);
            }
        }
    }
    if (greyButton->isChecked()) {
        if (sixteenBitButton->isChecked()) {
            if (OA_ISGREYSCALE(config.imagePixelFormat) && OA_BYTES_PER_PIXEL(config.imagePixelFormat) == 1)
            {
                QMessageBox::warning ( this, APPLICATION_NAME, "Can't output 16bit greyscale, reverting to 8bit");
                eightBitButton->setChecked(true);
            }
        }
    }
    if (rawButton->isChecked()) {
        if (sixteenBitButton->isChecked()) {
            if (OA_BYTES_PER_PIXEL(config.imagePixelFormat) == 1)
            {
                QMessageBox::warning ( this, APPLICATION_NAME, "Can't output 16bit raw, reverting to 8bit");
                eightBitButton->setChecked(true);
            }
        }
    }
    inferPixelFormatFromSimpleSettings(); // triggers advancedFormatChanged()???
}

void OutputSettings::inferPixelFormatFromSimpleSettings(void) {
    std::cerr << "in inferPixelFormatFromSimpleSettings()\n";

    const bool colour = colourButton->isChecked();
    const bool grey = greyButton->isChecked();
    const bool raw = rawButton->isChecked();
    const bool eightBit = eightBitButton->isChecked();

    //TODO calculate correct output format
    int output_format = 0;
    if (colour && eightBit)
        output_format = OA_PIX_FMT_RGB24;
    if (colour && !eightBit)
        output_format = OA_PIX_FMT_RGB48LE;
    if (grey && eightBit)
        output_format = OA_PIX_FMT_GREY8;
    if (grey && !eightBit)
        output_format = OA_PIX_FMT_GREY16LE;
    if (raw && eightBit)
        output_format = OA_TO_BAYER8(config.imagePixelFormat);
    if (raw && !eightBit)
        output_format = OA_TO_BAYER16(config.imagePixelFormat);

    int outputFormatIndex = outputFormatMenu->findData(output_format);
    std::cerr << "Simple implies output format " << OA_PIX_FMT_STRING(output_format)
              << "@" << outputFormatIndex << "\n";
    outputFormatMenu->setCurrentIndex(outputFormatIndex); // triggers advancedFormatChanged???
}

void OutputSettings::advancedFormatChanged(int index) {
    std::cerr << "in advancedFormatChanged " << index << "\n";

    int output_format = outputFormatMenu->itemData(index).toInt();
    inputFormatValue->setText(QString(OA_PIX_FMT_STRING(config.imagePixelFormat)).append(" -> ").append(OA_PIX_FMT_STRING(output_format)));

    if (!OA_SHOULD_CONVERT_PIX_FMT(config.imagePixelFormat, output_format))
        advancedBox->setText("Advanced (warning: non standard)");
    else
        advancedBox->setText("Advanced");


    std::cerr << "constructing conversion from changed format\n";
    const Conversion convert(config.imagePixelFormat, output_format, config.demosaic.cfaPattern, config.demosaic.method);

    cfaPatternMenu->setEnabled(convert.doDemosaic());
    cfaPatternMenu->setCurrentIndex ( cfaPatternMenu->findData(convert.cfaFormat()) );
//    cfaPatternMenu->setCurrentIndex ( cfaPatternMenu->findData(
//        (!OA_ISBAYER(config.imagePixelFormat) || OA_ISBAYER(output_format))
//        ? OA_DEMOSAIC_AUTO
//        : config.demosaic.cfaPattern ));

    methodMenu->setEnabled(convert.doDemosaic());
    methodMenu->setCurrentIndex ( methodMenu->findData( convert.demosaicMethod() ) );//config.demosaic.method ) );

    doDemosaicCheckbox->setChecked(convert.doDemosaic());
    doDemosaicCheckbox->setEnabled(convert.canDoDemosaic() && !convert.mustDoDemosaic());

    doGreyscaleCheckbox->setChecked(convert.doGreyscale());
    doGreyscaleCheckbox->setEnabled(false);

    inferSimpleSettingsFromAdvanced();

    // first time advancedFormatChanged is called is to populate
    // after that, it gets called on change of output format
    // first time, dummyForceDataChanged hasn't been connected
    // so we don't call dataChanged, so next line is a nop.
    dummyForceDataChanged->setChecked(!dummyForceDataChanged->isChecked());
}

void OutputSettings::advancedCheckboxClicked(int state) {
    std::cerr << "in advancedCheckboxClicked " << state << "\n";

    const bool switchToSimple = state==0;

    enableOrDisableSimpleSettings(switchToSimple);
    showOrHideAdvancedSettings(!switchToSimple);

    if (switchToSimple)
        simplifyAdvancedSettings();
}

void OutputSettings::enableOrDisableSimpleSettings(int simple) {
    std::cerr << "in enableOrDisableSimpleSettings()\n";
    colourButton->setEnabled(simple);
    greyButton->setEnabled(simple);
    rawButton->setEnabled(simple);
    eightBitButton->setEnabled(simple);
    sixteenBitButton->setEnabled(simple);
}

void OutputSettings::showOrHideAdvancedSettings(int advanced) {
    std::cerr << "in showOrHideAdvancedSettings()\n";
    outputFormatLabel->setVisible(advanced);
    outputFormatMenu->setVisible(advanced);
    doDemosaicCheckbox->setVisible(advanced);
    doGreyscaleCheckbox->setVisible(advanced);
    cfaLabel->setVisible(advanced);
    cfaPatternMenu->setVisible(advanced);
    methodLabel->setVisible(advanced);
    methodMenu->setVisible(advanced);

}

void OutputSettings::simplifyAdvancedSettings() {
    std::cerr << "in simplifyAdvancedSettings()\n";
    bool warn = false;
    std::string warning = "The following settings have been simplified:\n";

    const int current_format = outputFormatMenu->itemData(outputFormatMenu->currentIndex()).toInt();
    const int current_cfa  = cfaPatternMenu->itemData(cfaPatternMenu->currentIndex()).toInt();
    const int current_method = methodMenu->itemData(methodMenu->currentIndex()).toInt();

    std::cerr << "constructing advanced\n";
    const Conversion advanced(config.imagePixelFormat, current_format, current_cfa, current_method);
    std::cerr << "constructing simple from advanced\n";
    const Conversion simple = advanced.toSimple();
    const int new_format = simple.outputFormat();
    const int new_cfa = simple.cfaFormat();
    const int new_method = simple.demosaicMethod();

    if (new_format != current_format)
    {
        warn = true;

        warning += "\tPixel format changed from ";
        warning += OA_PIX_FMT_STRING(current_format);
        warning += " to ";
        warning += OA_PIX_FMT_STRING(new_format);
        warning += "\n"; // FIXME one line

        outputFormatMenu->setCurrentIndex(outputFormatMenu->findData(new_format));
    }

    if (simple.doDemosaic() && !doDemosaicCheckbox->isChecked())
    {
        warn = true;
        warning += "\tDemosaic enabled (greyscale output)\n";
        doDemosaicCheckbox->setChecked(true);
    }

    if (doDemosaicCheckbox->isChecked()) {
        if (new_cfa != current_cfa) {
            warn = true;

            warning += "\tCFA pattern changed from ";
            warning += "TODO"; // FIXME "RGGB" etc
            warning += " to AUTO\n"; // FIXME one line

            cfaPatternMenu->setCurrentIndex(cfaPatternMenu->findData( new_cfa ));
        }
        if (new_method != current_method) {
            warn = true;

            warning += "\tDemosaic method changed from \"";
            warning += oademosaicMethodName(current_method);
            warning += "\" to \"";
            warning += oademosaicMethodName(new_method);
            warning += "\"\n"; // FIXME one line

            methodMenu->setCurrentIndex(methodMenu->findData( new_method ));
        }

    }

    if (warn) {
        QMessageBox::warning ( this, APPLICATION_NAME, warning.c_str());
    }

    inferSimpleSettingsFromAdvanced();
}

void OutputSettings::inferSimpleSettingsFromAdvanced() {
    const bool simpleEnabled = !advancedBox->isChecked();

    std::cerr << "in inferSimpleSettingsFromAdvanced()\n";
    int output_format = outputFormatMenu->itemData(outputFormatMenu->currentIndex()).toInt();

    std::cerr << "construction conversion from current\n";
    const Conversion convert(config.imagePixelFormat, output_format);

    colourButton->setEnabled(simpleEnabled && convert.canDoSimpleRGB());
    colourButton->setChecked(convert.doSimpleRGB());

    greyButton->setEnabled(simpleEnabled && convert.canDoSimpleGrey());
    greyButton->setChecked(convert.doSimpleGrey());

    rawButton->setEnabled(simpleEnabled && convert.canDoSimpleRaw());
    rawButton->setChecked(convert.doSimpleRaw());

    eightBitButton->setEnabled(simpleEnabled && convert.canDoSimple8Bit());
    eightBitButton->setChecked(convert.doSimple8Bit());
    sixteenBitButton->setEnabled(simpleEnabled && convert.canDoSimple16Bit());
    sixteenBitButton->setChecked(convert.doSimple16Bit());
}
