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

#ifndef XCSOAR_SOCKET_THREAD_HPP
#define XCSOAR_SOCKET_THREAD_HPP

#include "Thread/StoppableThread.hpp"
#include "Net/SocketDescriptor.hpp"

#include <assert.h>

class SocketEventHandler;

/**
 * An adapter between a #SocketDescriptor and a #SocketEventHandler.
 */
class SocketThread : public StoppableThread {
  SocketDescriptor socket;
  SocketEventHandler &handler;

public:
  SocketThread(SocketEventHandler &_handler)
    :StoppableThread("SocketThread"), handler(_handler) {}

  bool Start(SocketDescriptor _socket) {
    assert(_socket.IsDefined());

    socket = _socket;
    return StoppableThread::Start();
  }

protected:
  /* virtual methods from class Thread */
  void Run() override;
};

#endif
