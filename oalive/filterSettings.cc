/*****************************************************************************
 *
 * filterSettings.cc -- class for the filter settings tab in the settings UI
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

#include <oa_common.h>

#include <QtGui>

extern "C" {
#include <openastro/filterwheel.h>
}

#include "configuration.h"
#include "state.h"

#include "filterSettings.h"


FilterSettings::FilterSettings ( QWidget* parent ) : QWidget ( parent )
{
  filterWheelSlots = 0;
  if ( state.filterWheel && state.filterWheel->isInitialised()) {
    filterWheelSlots = state.filterWheel->numSlots();
  }
  list = new QListWidget ( this );
  if ( config.numFilters ) {
    for ( int i = 0; i < config.numFilters; i++ ) {
      list->addItem ( config.filters[i].filterName );
      QListWidgetItem* entry = list->item ( i );
      entry->setFlags ( entry->flags() | Qt :: ItemIsEditable );
    }
  }
  addButton = new QPushButton ( QIcon ( ":/qt-icons/list-add-4.png" ),
      tr ( "Add Item" ));
  addButton->setStyleSheet("Text-align:left");
  connect ( addButton, SIGNAL ( clicked()), this, SLOT ( addEntry()));
  connect ( addButton, SIGNAL ( clicked()), parent, SLOT ( dataChanged()));
  removeButton = new QPushButton ( QIcon ( ":/qt-icons/list-remove-4.png" ),
      tr ( "Remove Item" ));
  removeButton->setStyleSheet("Text-align:left");
  connect ( removeButton, SIGNAL ( clicked()), this, SLOT ( removeEntry()));
  connect ( removeButton, SIGNAL ( clicked()), parent, SLOT ( dataChanged()));
  /*
  connect ( list, SIGNAL ( currentItemChanged ( QListWidgetItem*,
      QListWidgetItem* )), this, SLOT ( focusChanged ( QListWidgetItem*,
      QListWidgetItem* )));
   */
  connect ( list, SIGNAL ( itemChanged ( QListWidgetItem* )), this,
      SLOT ( itemChanged ( QListWidgetItem* )));
  connect ( list, SIGNAL ( itemChanged ( QListWidgetItem* )), parent,
      SLOT ( dataChanged()));

  slotGrid = new QGridLayout();

  slotChangedMapper = new QSignalMapper ( this );
  connect ( slotChangedMapper, SIGNAL( mapped ( int )), this,
      SLOT ( filterSlotChanged ( int )));

  for ( int i = 0; i < MAX_FILTER_SLOTS; i++ ) {
    char t[20];
    snprintf ( t, 15, "Slot %d: ", i + 1 );
    slotLabels[i] = new QLabel ( tr ( t ), this );
    slotMenus[i] = new QComboBox ( this );
    slotGrid->addWidget ( slotLabels[i], i / 2, ( i % 2 ) * 2 );
    slotGrid->addWidget ( slotMenus[i], i / 2, ( i % 2 ) * 2 + 1 );
    for ( int j = 0; j < config.numFilters; j++ ) {
      slotMenus[i]->addItem ( config.filters[j].filterName );
    }
    if ( config.filterSlots[i] >= 0 ) {
      slotMenus[i]->setCurrentIndex ( config.filterSlots[i] );
    }
    connect ( slotMenus[i], SIGNAL ( currentIndexChanged ( int )),
        slotChangedMapper, SLOT ( map()));
    slotChangedMapper->setMapping ( slotMenus[i], i );
    connect ( slotMenus[i], SIGNAL ( currentIndexChanged ( int )), parent,
        SLOT ( dataChanged()));
    if ( i >= filterWheelSlots ) {
      slotLabels[i]->hide();
      slotMenus[i]->hide();
    }
  }

  vbox = new QVBoxLayout();
  vbox->addStretch ( 3 );
  vbox->addWidget ( addButton );
  vbox->addWidget ( removeButton );
  vbox->addStretch ( 3 );

  hbox = new QHBoxLayout ( this );
  hbox->addWidget ( list );
  hbox->addLayout ( vbox );
  hbox->addStretch ( 1 );
  hbox->addLayout ( slotGrid );
  hbox->addStretch ( 1 );
  setLayout ( hbox );
  listChanged = slotsChanged = 0;
}


FilterSettings::~FilterSettings()
{
  state.mainWindow->destroyLayout (( QLayout* ) hbox );
}


void
FilterSettings::storeSettings ( void )
{
  int oldFilterCount, newFilterCount;
  int compareOldPosn, compareNewPosn;
  int deleteOldFilter, totalFilters;

  if ( !listChanged && !slotsChanged ) {
    return;
  }

  // Additions to the filter list can only have been made at the end,
  // whereas deletions can come anywhere.
  //
  // So, step through the new list of filters.  If the name of the current
  // new one is the same as the name of the current old one then move on.
  //
  // If the names don't match, the existing filter at this position must have
  // been deleted, so delete the current one, delete the entry associated
  // with it from the profile filter settings list and compare the same
  // new one with the next old one.  Repeat as above.
  //
  // We've reached the end when either there are no more old filters,
  // in which case the new ones need to be added, together with their
  // profile filter settings, or there are no more new ones, in which
  // case the remaining old ones need to be deleted together with their
  // profile filter settings
  //
  // In fact we should check if we've reached the end of either list
  // first. as one or the other list may actually be empty


  if ( listChanged ) {
    newFilterCount = list->count();
    totalFilters = oldFilterCount = config.numFilters;
    // handle the degenerate case
    if ( !newFilterCount && !oldFilterCount ) {
qWarning() << "state.captureWidget->reloadFilters();";
      // state.captureWidget->reloadFilters();
      return;
    }

    compareOldPosn = compareNewPosn = 0;
    while ( compareOldPosn != oldFilterCount || compareNewPosn !=
        newFilterCount ) {

      deleteOldFilter = 0;

      if ( compareOldPosn == oldFilterCount ) {
        // We know we're not at the end of the new list as the while loop test
        // would have failed, so add the filter at the current new position
        FILTER f;
        QListWidgetItem* entry = list->item ( compareNewPosn );
        f.filterName = entry->text();
        config.filters.append ( f );
        // Add the profiles for the new filter
        for ( int i = 0; i < config.numProfiles; i++ ) {
          FILTER_PROFILE fp;
          fp.filterName = f.filterName;
          for ( int j = 1; j <  OA_CAM_CTRL_LAST_P1; j++ ) {
            for ( int k = 0; k <  OA_CAM_CTRL_MODIFIERS_P1; k++ ) {
              fp.controls[k][j] = config.controlValues[k][j];
            }
          }
          config.profiles[i].filterProfiles.append ( fp );
        }
        totalFilters++;
        compareNewPosn++;

      } else {
        if ( compareNewPosn == newFilterCount ) {
          // We're not at the end of the old list otherwise we'd have hit the
          // previous test, so delete the filter in the current old position
          deleteOldFilter = 1;
        } else {
          QListWidgetItem* entry = list->item ( compareNewPosn );
          if ( config.filters[ compareOldPosn ].filterName != entry->text()) {
            // The name of the new one doesn't match the current old one, so we
            // need to delete the old one
            deleteOldFilter = 1;
          } else {
            // The filter names match, so we keep both
            compareOldPosn++;
            compareNewPosn++;
          }
        }

        if ( deleteOldFilter ) {
          // remove the filter
          config.filters.removeAt ( compareOldPosn );
          // remove the filters profiles
          for ( int i = 0; i < config.numProfiles; i++ ) {
            config.profiles[i].filterProfiles.removeAt ( compareOldPosn );
          }
          totalFilters--;
          oldFilterCount--;
        }
      }
    }
    config.numFilters = totalFilters;
  }

  if ( slotsChanged ) {
    for ( int i = 0; i < MAX_FILTER_SLOTS; i++ ) {
      config.filterSlots[i] = slotMenus[i]->currentIndex();
    }
  }
qWarning() << "state.captureWidget->reloadFilters();";
  // state.captureWidget->reloadFilters();
}


void
FilterSettings::addEntry ( void )
{
  QListWidgetItem* entry = new QListWidgetItem ( "" );
  entry->setFlags ( entry->flags() | Qt :: ItemIsEditable );
  list->addItem ( entry );
  list->editItem ( entry );
  listChanged = 1;
}


void
FilterSettings::removeEntry ( void )
{
  int row = list->currentRow();

  QListWidgetItem* entry = list->takeItem ( row );
  if ( entry ) {
    delete entry;
    for ( int i = 0; i < MAX_FILTER_SLOTS; i++ ) {
      slotMenus[i]->removeItem ( row );
    }
    listChanged = slotsChanged = 1;
    // FIX ME -- remove item from filter sequence if present
  }
}


void
FilterSettings::focusChanged ( QListWidgetItem* curr, QListWidgetItem* old )
{
  Q_UNUSED ( curr )
  if ( old && old->text() == "" ) {
    int row = list->row ( old );
    ( void ) list->takeItem ( list->row ( old ));
    delete old;
    for ( int i = 0; i < MAX_FILTER_SLOTS; i++ ) {
      slotMenus[i]->removeItem ( row );
    }
    listChanged = slotsChanged = 1;
    // FIX ME -- remove item from filter sequence if present
  }
}


void
FilterSettings::itemChanged ( QListWidgetItem* item )
{
  QString itemText = item->text();
  int row = list->row ( item );

  if ( itemText == "" ) {
    ( void ) list->takeItem ( list->row ( item ));
    delete item;
    for ( int i = 0; i < MAX_FILTER_SLOTS; i++ ) {
      slotMenus[i]->removeItem ( row );
      // FIX ME -- remove item from filter sequence if present
    }
  } else {
    if ( row < slotMenus[0]->count()) { // not new
      for ( int i = 0; i < MAX_FILTER_SLOTS; i++ ) {
        slotMenus[i]->setItemText ( row, itemText );
        // FIX ME -- update filter sequence menus if item is selected for a slot
      }
    } else {
      for ( int i = 0; i < MAX_FILTER_SLOTS; i++ ) {
        slotMenus[i]->addItem ( itemText );
      }
    }
  }
  listChanged = 1;
}


void
FilterSettings::filterSlotChanged ( int slotIndex )
{
  // If this slot is being used in the autorun filter sequence then we
  // need to update that with the new text

  QString filterName = "";
  int currentFilter = slotMenus[ slotIndex ]->currentIndex();
  if ( currentFilter >= 0 ) {
    filterName = slotMenus[ slotIndex ]->itemText ( currentFilter );
  }
  state.settingsWidget->propagateNewSlotName ( slotIndex, filterName );
  slotsChanged = 1;
}


QString
FilterSettings::getSlotFilterName ( int slotIndex )
{
  QString filterName = "";
  int currentFilter = slotMenus[ slotIndex ]->currentIndex();
  if ( currentFilter >= 0 ) {
    filterName = slotMenus[ slotIndex ]->itemText ( currentFilter );
  }
  return filterName;
}


void
FilterSettings::setSlotCount ( int numSlots )
{
  for ( int i = 0; i < MAX_FILTER_SLOTS; i++ ) {
    if ( i < numSlots ) {
      slotLabels[i]->show();
      slotMenus[i]->show();
    } else {
      slotLabels[i]->hide();
      slotMenus[i]->hide();
    }
  }

  if ( numSlots > 0 ) {
    for ( int i = 0; i < numSlots; i++ ) {
      filterSlotChanged ( i );
    }
  }
}
