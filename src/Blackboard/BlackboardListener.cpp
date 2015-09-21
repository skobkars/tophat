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

#include "BlackboardListener.hpp"
#include "Compiler.h"

void
NullBlackboardListener::OnGPSUpdate(gcc_unused const MoreData &basic)
{
}

void
NullBlackboardListener::OnCalculatedUpdate(gcc_unused const MoreData &basic,
                                           gcc_unused const DerivedInfo &calculated)
{
}

void
NullBlackboardListener::OnComputerSettingsUpdate(gcc_unused const ComputerSettings &settings)
{
}

void
NullBlackboardListener::OnUISettingsUpdate(gcc_unused const UISettings &settings)
{
}

void
NullBlackboardListener::OnUIStateUpdate()
{
}
