/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#ifndef XCSOAR_ICON_LOOK_HPP
#define XCSOAR_ICON_LOOK_HPP

#include "Screen/Icon.hpp"

/**
 * This class manages the icons of various XCSoar dialogs.
 */
struct IconLook {
  /* can't draw icons until screen is initialized which initializes this IconLook */
  bool valid;
  // task dialog
  MaskedIcon hBmpTabTask;
  MaskedIcon hBmpTabWrench;
  MaskedIcon hBmpTabSettings;
  MaskedIcon hBmpTabSettingsNavBar;
  MaskedIcon hBmpTabCalculator;

  // status dialog
  MaskedIcon hBmpTabFlight;
  MaskedIcon hBmpTabSystem;
  MaskedIcon hBmpTabRules;
  MaskedIcon hBmpTabTimes;

  MaskedIcon hBmpMenuButton;
  MaskedIcon hBmpScreensButton;
  MaskedIcon hBmpCheckMark;
  MaskedIcon hBmpSearch;
  MaskedIcon hBmpSearchChecked;
  MaskedIcon hBmpZoomOutButton;
  MaskedIcon hBmpZoomInButton;
  MaskedIcon hBmpBearingLeftOne;
  MaskedIcon hBmpBearingLeftTwo;
  MaskedIcon hBmpBearingLeftThree;
  MaskedIcon hBmpBearingLeftFour;
  MaskedIcon hBmpBearingRightOne;
  MaskedIcon hBmpBearingRightTwo;
  MaskedIcon hBmpBearingRightThree;
  MaskedIcon hBmpBearingRightFour;
  MaskedIcon hBmpClose;
  MaskedIcon icon_home;

  void Initialise();
};

#endif
