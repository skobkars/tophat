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

#ifndef XCSOAR_SYMBOL_BUTTON_RENDERER_HPP
#define XCSOAR_SYMBOL_BUTTON_RENDERER_HPP

#include "TextButtonRenderer.hpp"
#include "Util/StaticString.hxx"

#include <tchar.h>

/**
 * A #ButtonRenderer instance that renders a regular button frame and
 * a symbol.
 */
class SymbolButtonRenderer : public TextButtonRenderer {

public:
  SymbolButtonRenderer(const ButtonLook &_look,
                       StaticString<64>::const_pointer _caption)
    :TextButtonRenderer(_look, _caption) {}


private:
  virtual void DrawButton(Canvas &canvas, const PixelRect &rc,
                          bool enabled, bool focused, bool pressed) const;

  void DrawSymbol(Canvas &canvas, PixelRect rc,
                  bool enabled, bool focused, bool pressed) const;
};

#endif