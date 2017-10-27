/*****************************************************************************
 *
 * demosaicSettings.cc -- class for the demosaic settings in the settings UI
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

#include "boostSettings.h"

#include "configuration.h"
#include "state.h"

BoostSettings::BoostSettings ( QWidget* parent ) : QWidget ( parent )
{
  stretchLabel = new QLabel ( tr ( "Stretch histogram" ));
  stretchBox = new QCheckBox ( tr ( "Stretch histogram" ), this );
  stretchBox->setChecked ( config.boost.stretch );

  sharpenLabel = new QLabel ( tr ( "Sharpen" ));
  sharpenMenu = new QComboBox( this );
  sharpenMenu->addItem ( tr("None"), QVariant ( CONFIG::boost::SHARPEN_NONE ) );
  sharpenMenu->addItem ( tr("Soft"), QVariant ( CONFIG::boost::SHARPEN_SOFT ) );
  sharpenMenu->addItem ( tr("Hard"), QVariant ( CONFIG::boost::SHARPEN_HARD ) );
  sharpenMenu->setCurrentIndex ( sharpenMenu->findData(config.boost.sharpen) );

  multiplyLabel = new QLabel ( tr ( "Multiply" ));
  multiplyMenu = new QComboBox ( this );
  multiplyMenu->addItem ( QString::number ( CONFIG::boost::MUL_X1 ), QVariant ( CONFIG::boost::MUL_X1 ) );
  multiplyMenu->addItem ( QString::number ( CONFIG::boost::MUL_X2 ), QVariant ( CONFIG::boost::MUL_X2 ) );
  multiplyMenu->addItem ( QString::number ( CONFIG::boost::MUL_X4 ), QVariant ( CONFIG::boost::MUL_X4 ) );
  multiplyMenu->addItem ( QString::number ( CONFIG::boost::MUL_X8 ), QVariant ( CONFIG::boost::MUL_X8 ) );
  multiplyMenu->addItem ( QString::number ( CONFIG::boost::MUL_X16 ), QVariant ( CONFIG::boost::MUL_X16 ) );
  multiplyMenu->addItem ( QString::number ( CONFIG::boost::MUL_X32 ), QVariant ( CONFIG::boost::MUL_X32 ) );
  multiplyMenu->setCurrentIndex ( multiplyMenu->findData(config.boost.multiply) );

  gammaLabel = new QLabel ( tr ( "Gamma" ));
  gammaMenu = new QComboBox ( this );
  gammaMenu->addItem ( "0.25", QVariant ( CONFIG::boost::GAMMA_0_25 ) );
  gammaMenu->addItem ( "0.5", QVariant ( CONFIG::boost::GAMMA_0_5 ) );
  gammaMenu->addItem ( "0.75", QVariant ( CONFIG::boost::GAMMA_0_75 ) );
  gammaMenu->addItem ( "1", QVariant ( CONFIG::boost::GAMMA_1_0 ) );
  gammaMenu->addItem ( "1.5", QVariant ( CONFIG::boost::GAMMA_1_5 ) );
  gammaMenu->addItem ( "2", QVariant ( CONFIG::boost::GAMMA_2_0 ) );
  gammaMenu->setCurrentIndex ( gammaMenu->findData(config.boost.gamma) );

  algorithmLabel = new QLabel ( tr ( "Algorithm" ));
  algorithmMenu = new QComboBox ( this );
  algorithmMenu->addItem ( tr("None"), QVariant ( CONFIG::boost::ALGO_NONE ) );
  algorithmMenu->addItem ( tr("Bin 2x2"), QVariant ( CONFIG::boost::ALGO_BIN2X2 ) );
  algorithmMenu->addItem ( tr("Bin 3x3"), QVariant ( CONFIG::boost::ALGO_BIN3X3 ) );
  algorithmMenu->addItem ( tr("Bin 4x4"), QVariant ( CONFIG::boost::ALGO_BIN4X4 ) );
  algorithmMenu->addItem ( tr("Avg 2x2"), QVariant ( CONFIG::boost::ALGO_AVG2X2 ) );
  algorithmMenu->addItem ( tr("Avg 3x3"), QVariant ( CONFIG::boost::ALGO_AVG3X3 ) );
  algorithmMenu->addItem ( tr("Avg 4x4"), QVariant ( CONFIG::boost::ALGO_AVG4X4 ) );
  algorithmMenu->addItem ( tr("Digital Bin 2x2"), QVariant ( CONFIG::boost::ALGO_dBIN2X2 ) );
  algorithmMenu->addItem ( tr("Digital Bin 3x3"), QVariant ( CONFIG::boost::ALGO_dBIN3X3 ) );
  algorithmMenu->addItem ( tr("Digital Bin 4x4"), QVariant ( CONFIG::boost::ALGO_dBIN4X4 ) );
  algorithmMenu->addItem ( tr("Digital Avg 2x2"), QVariant ( CONFIG::boost::ALGO_dAVG2X2 ) );
  algorithmMenu->addItem ( tr("Digital Avg 3x3"), QVariant ( CONFIG::boost::ALGO_dAVG3X3 ) );
  algorithmMenu->addItem ( tr("Digital Avg 4x4"), QVariant ( CONFIG::boost::ALGO_dAVG4X4 ) );
  algorithmMenu->addItem ( tr("Adaptive Digital Pixel Binning (gain=2)"), QVariant ( CONFIG::boost::ALGO_ADPB2 ) );
  algorithmMenu->addItem ( tr("Adaptive Digital Pixel Binning (gain=4)"), QVariant ( CONFIG::boost::ALGO_ADPB4 ) );
  algorithmMenu->addItem ( tr("Adaptive Digital Pixel Binning (gain=6)"), QVariant ( CONFIG::boost::ALGO_ADPB6 ) );
  algorithmMenu->addItem ( tr("Adaptive Digital Pixel Binning (gain=8)"), QVariant ( CONFIG::boost::ALGO_ADPB8 ) );
  algorithmMenu->setCurrentIndex ( algorithmMenu->findData(config.boost.algorithm) );

  box = new QVBoxLayout ( this );
  box->addWidget ( stretchLabel );
  box->addWidget ( stretchBox );
  box->addWidget ( sharpenLabel );
  box->addWidget ( sharpenMenu );
  box->addSpacing ( 15 );
  box->addWidget ( multiplyLabel );
  box->addWidget ( multiplyMenu );
  box->addWidget ( gammaLabel );
  box->addWidget ( gammaMenu );
  box->addSpacing ( 15 );
  box->addWidget ( algorithmLabel );
  box->addWidget ( algorithmMenu );
  box->addStretch ( 1 );
  setLayout ( box );

  connect ( stretchBox, SIGNAL ( stateChanged ( int )), parent,
      SLOT ( dataChanged()));
  connect ( sharpenMenu, SIGNAL( currentIndexChanged ( int )), parent,
      SLOT( dataChanged()));
  connect ( multiplyMenu, SIGNAL( currentIndexChanged ( int )), parent,
      SLOT( dataChanged()));
  connect ( gammaMenu, SIGNAL( currentIndexChanged ( int )), parent,
      SLOT( dataChanged()));
  connect ( algorithmMenu, SIGNAL( currentIndexChanged ( int )), parent,
      SLOT( dataChanged()));
}


BoostSettings::~BoostSettings()
{
  state.mainWindow->destroyLayout (( QLayout* ) box );
}


void
BoostSettings::storeSettings ( void )
{
  config.boost.stretch = stretchBox->isChecked() ? 1 : 0;
  config.boost.sharpen =
      sharpenMenu->itemData(sharpenMenu->currentIndex()).toInt();
  config.boost.multiply =
      multiplyMenu->itemData(multiplyMenu->currentIndex()).toInt();
  config.boost.gamma =
      gammaMenu->itemData(gammaMenu->currentIndex()).toInt();
  config.boost.algorithm =
      algorithmMenu->itemData(algorithmMenu->currentIndex()).toInt();
}
