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

  multiplyLabel = new QLabel ( tr ( "Multiply" ));
  multiplyMenu = new QComboBox ( this );
  multiplyMenu->addItem ( QString::number ( CONFIG::boost::MUL_X1 ), QVariant ( CONFIG::boost::MUL_X1 ) );
  multiplyMenu->addItem ( QString::number ( CONFIG::boost::MUL_X2 ), QVariant ( CONFIG::boost::MUL_X2 ) );
  multiplyMenu->addItem ( QString::number ( CONFIG::boost::MUL_X4 ), QVariant ( CONFIG::boost::MUL_X4 ) );
  multiplyMenu->addItem ( QString::number ( CONFIG::boost::MUL_X8 ), QVariant ( CONFIG::boost::MUL_X8 ) );
  multiplyMenu->setCurrentIndex ( 0 );

  algorithmLabel = new QLabel ( tr ( "Algorithm" ));
  algorithmMenu = new QComboBox ( this );
  algorithmMenu->addItem ( tr("None"), QVariant ( CONFIG::boost::ALGO_NONE ) );
  algorithmMenu->addItem ( tr("Bin 2x2"), QVariant ( CONFIG::boost::ALGO_BIN2X2 ) );
  algorithmMenu->addItem ( tr("Bin 3x3"), QVariant ( CONFIG::boost::ALGO_BIN3X3 ) );
  algorithmMenu->addItem ( tr("Bin 4x4"), QVariant ( CONFIG::boost::ALGO_BIN4X4 ) );
  algorithmMenu->addItem ( tr("Avg 2x2"), QVariant ( CONFIG::boost::ALGO_AVG2X2 ) );
  algorithmMenu->addItem ( tr("Avg 3x3"), QVariant ( CONFIG::boost::ALGO_AVG3X3 ) );
  algorithmMenu->addItem ( tr("Avg 4x4"), QVariant ( CONFIG::boost::ALGO_AVG4X4 ) );
  algorithmMenu->addItem ( tr("Custom1"), QVariant ( CONFIG::boost::ALGO_CUSTOM1 ) );
  algorithmMenu->addItem ( tr("Custom2"), QVariant ( CONFIG::boost::ALGO_CUSTOM2 ) );
  algorithmMenu->setCurrentIndex ( 0 );

  box = new QVBoxLayout ( this );
  box->addWidget ( stretchLabel );
  box->addWidget ( stretchBox );
  box->addSpacing ( 15 );
  box->addWidget ( multiplyLabel );
  box->addWidget ( multiplyMenu );
  box->addSpacing ( 15 );
  box->addWidget ( algorithmLabel );
  box->addWidget ( algorithmMenu );
  box->addStretch ( 1 );
  setLayout ( box );
  connect ( stretchBox, SIGNAL ( stateChanged ( int )), parent,
      SLOT ( dataChanged()));
  connect ( multiplyMenu, SIGNAL( currentIndexChanged ( int )), parent,
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
  config.boost.multiply =
      multiplyMenu->itemData(multiplyMenu->currentIndex()).toInt();
  config.boost.algorithm =
      algorithmMenu->itemData(algorithmMenu->currentIndex()).toInt();
}
