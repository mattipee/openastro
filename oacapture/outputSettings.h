/*****************************************************************************
 *
 * outputSettings.h -- class declaration
 *
 * Copyright 2013,2014,2016,2017 James Fidell (james@openastroproject.org)
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

#include <openastro/video/formats.h>
#include <iostream>
class Conversion
{
public:
    Conversion(int input_format, int output_format, bool optionalDemosaic = true, int cfaFormat = OA_DEMOSAIC_AUTO, int method = OA_DEMOSAIC_NEAREST_NEIGHBOUR)
      : I(input_format), O(output_format), od(optionalDemosaic),
        cfa((!OA_ISBAYER(I) || OA_ISBAYER(O)) ? OA_DEMOSAIC_AUTO : cfaFormat),
        m(method)
    {
        if (!OA_CAN_CONVERT_PIX_FMT(I,O))
        {
            std::cerr << "can't convert from " << OA_PIX_FMT_STRING(I) << " to " << OA_PIX_FMT_STRING(O) << "\n";
            throw 1;
        }
    }

    int outputFormat() const { return O; }
    int cfaFormat() const { return cfa; }
    int demosaicMethod() const { return m; }

    bool canDoSimpleRGB() const { return OA_ISBAYER(I) || OA_ISRGB(O); }
    bool canDoSimpleGrey() const { return true; }
    bool canDoSimpleRaw() const { return OA_ISBAYER(I); }
    bool doSimpleRGB() const { return OA_ISRGB(O); }
    bool doSimpleGrey() const { return OA_ISGREYSCALE(O); }
    bool doSimpleRaw() const { return OA_ISBAYER(O); }


    bool canDoSimple8Bit() const { return true; }
    bool canDoSimple16Bit() const { return OA_ISRGB(I) ? OA_BYTES_PER_PIXEL(I) == 6
                                                       : OA_ISGREYSCALE(O) && OA_ISBAYER(I) ? true : OA_BYTES_PER_PIXEL(I) > 1; } 
    bool doSimple8Bit() const { return doSimpleRGB() ? OA_BYTES_PER_PIXEL(O) == 3
                                                     : OA_BYTES_PER_PIXEL(I) == 1; }
    bool doSimple16Bit() const { return doSimpleRGB() ? OA_BYTES_PER_PIXEL(O) == 6
                                                      : OA_BYTES_PER_PIXEL(O) > 1; }


    bool canDoDemosaic() const { return OA_ISBAYER(I) && (doSimpleRGB() || doSimpleGrey()); }
    bool mustDoDemosaic() const { return OA_ISBAYER(I) && doSimpleRGB(); }
    bool doDemosaic() const { return mustDoDemosaic() ? true : canDoDemosaic() ? od : false; }
    bool doGreyscale() const { return !OA_ISGREYSCALE(I) && doSimpleGrey(); }

    Conversion toSimple() const {
        int newO = O;

        if (OA_ISBAYER(O)) {
            if (OA_BYTES_PER_PIXEL(I) == 1)
                newO = OA_TO_BAYER8(I);
            else
                newO = OA_TO_BAYER16(I);
        }

        if (OA_ISGREYSCALE(O)) {
            if ((doDemosaic() || OA_BYTES_PER_PIXEL(I) > 1) && O != OA_PIX_FMT_GREY8)
                newO = OA_PIX_FMT_GREY16LE;
            else
                newO = OA_PIX_FMT_GREY8;
        }

        if (OA_ISRGB(O)) {
            if (OA_ISRGB(I))
                newO = I;
            else if ((doDemosaic() || OA_BYTES_PER_PIXEL(I) > 1) && OA_BYTES_PER_PIXEL(O) != 3)
                newO = OA_PIX_FMT_RGB48LE;
            else
                newO = OA_PIX_FMT_RGB24;
        }

        std::cerr << "simplified " << OA_PIX_FMT_STRING(O) << " to " << OA_PIX_FMT_STRING(newO) << " ... " << OA_SHOULD_CONVERT_PIX_FMT(I,newO) << "\n";
        return Conversion(I,newO,true);
    }
private:
    const int I;
    const int O;
    const bool od;
    const int cfa;
    const int m;
};

class OutputSettings : public QWidget
{
  Q_OBJECT

  public:
    			OutputSettings ( QWidget* );
    			~OutputSettings();
    void		storeSettings ( void );

  public slots:
    void simpleSettingsClicked(int);
    void advancedCheckboxClicked(int);
    void advancedFormatChanged(int);
    void advancedDemosaicClicked(int);

  private:
    void enableOrDisableSimpleSettings(int);
    void showOrHideAdvancedSettings(int);
    void simplifyAdvancedSettings();
    void inferPixelFormatFromSimpleSettings(void);
    void inferSimpleSettingsFromAdvanced(void);

    QVBoxLayout*	box;

    QLabel*         colourModeLabel;
    QButtonGroup*	colourModeButtons;
    QRadioButton*	colourButton;
    QRadioButton*	greyButton;
    QRadioButton*	rawButton;
    QLabel*         bitDepthLabel;
    QButtonGroup*	bitDepthButtons;
    QRadioButton*	eightBitButton;
    QRadioButton*	sixteenBitButton;
    QCheckBox*      advancedBox;



    QLabel* inputFormatLabel;
    QLabel* inputFormatValue;
    QLabel* outputFormatLabel;
    QComboBox* outputFormatMenu;
    QCheckBox* doDemosaicCheckbox;
    QCheckBox* doGreyscaleCheckbox;
    QCheckBox* dummyForceDataChanged;// don't understand qt signals... this is a HACK!

    QLabel*             cfaLabel;
    QComboBox*          cfaPatternMenu;
    QLabel*             methodLabel;
    QComboBox*          methodMenu;

};
