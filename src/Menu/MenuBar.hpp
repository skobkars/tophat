/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2013 The XCSoar Project
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

#ifndef XCSOAR_MENU_BAR_HPP
#define XCSOAR_MENU_BAR_HPP

#include "Form/SymbolButton.hpp"

#include <tchar.h>

class ContainerWindow;

/**
 * A container for menu buttons.
 */
class MenuBar {
public:
  enum {
    MAX_BUTTONS = 32,
  };

protected:
  class Button : public WndSymbolButton {
    unsigned event;

  public:
    Button(ContainerWindow &parent, const ButtonLook &look,
           const TCHAR *Caption,
           const PixelRect &rc, const ButtonWindowStyle style)
    :WndSymbolButton(parent, look,
                     Caption,
                     rc, style, NULL) {};

    void SetEvent(unsigned _event) {
      event = _event;
    }

    virtual bool OnClicked();

    /** sets the down flag that renders the button as such.
     *  This method does not work with GDI
     * @param _down
     */
    virtual void SetDown(bool _down) {
#ifndef USE_GDI
      ButtonWindow::SetDown(_down);
#endif
    }

#ifdef USE_GDI
  protected:
    virtual LRESULT OnMessage(HWND hWnd, UINT message,
                               WPARAM wParam, LPARAM lParam);
#endif
  };

  Button *buttons[MAX_BUTTONS];

public:
  MenuBar(ContainerWindow &parent);

public:
  void SetFont(const Font &font);
  /**
   * @param i
   * @param enabled
   * @param text
   * @param event
   * @param down.  renders button as down
   */
  void ShowButton(unsigned i, bool enabled, const TCHAR *text,
                  unsigned event, bool down);
  void HideButton(unsigned i);

  bool IsButtonEnabled(unsigned i) const {
    return buttons[i]->IsEnabled();
  }

  /**
   * To be called when the parent's size changes.  Moves all buttons
   * to a new position.
   */
  void OnResize(const PixelRect &rc);
};

#endif
