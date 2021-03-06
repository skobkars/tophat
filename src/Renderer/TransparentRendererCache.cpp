/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
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

#include "TransparentRendererCache.hpp"

#ifndef ENABLE_OPENGL
#include "Projection/WindowProjection.hpp"

#ifdef USE_GDI
#include "Screen/GDI/AlphaBlend.hpp"
#endif

bool
TransparentRendererCache::Check(const WindowProjection &projection) const
{
  assert(projection.IsValid());

  return buffer.IsDefined() &&
    buffer.GetWidth() == projection.GetScreenWidth() &&
    buffer.GetHeight() == projection.GetScreenHeight() &&
    compare_projection.Compare(projection);
}

Canvas &
TransparentRendererCache::Begin(Canvas &canvas,
                                const WindowProjection &projection)
{
  assert(canvas.IsDefined());
  assert(projection.IsValid());

  const PixelSize size(projection.GetScreenWidth(),
                       projection.GetScreenHeight());
  if (buffer.IsDefined())
    buffer.Resize(size);
  else
    buffer.Create(canvas, size);

  compare_projection = CompareProjection(projection);
  return buffer;
}

void
TransparentRendererCache::Commit(Canvas &canvas,
                                 const WindowProjection &projection)
{
  assert(canvas.IsDefined());
  assert(projection.IsValid());
  assert(buffer.IsDefined());

  /* if Invalidate() is called then compare_projection will be undefined */
  if (compare_projection.IsDefined() && Check(projection))
    empty = false;
}

void
TransparentRendererCache::CopyAndTo(Canvas &canvas,
                                    const WindowProjection &projection) const
{
  if (empty)
    return;

  canvas.CopyAnd(0, 0,
                 projection.GetScreenWidth(), projection.GetScreenHeight(),
                 buffer, 0, 0);
}

void
TransparentRendererCache::CopyTransparentWhiteTo(Canvas &canvas,
                                                 const WindowProjection &projection) const
{
  if (empty)
    return;

  canvas.CopyTransparentWhite(0, 0,
                              projection.GetScreenWidth(),
                              projection.GetScreenHeight(),
                              buffer, 0, 0);
}

#ifdef HAVE_ALPHA_BLEND

void
TransparentRendererCache::AlphaBlendTo(Canvas &canvas,
                                       const WindowProjection &projection,
                                       uint8_t alpha) const
{
  assert(AlphaBlendAvailable());
  assert(canvas.IsDefined());
  assert(buffer.IsDefined());
  assert(projection.IsValid());

  if (empty || !compare_projection.IsDefined() || !Check(projection))
    return;

  const unsigned width = projection.GetScreenWidth(),
    height = projection.GetScreenHeight();

#ifdef USE_MEMORY_CANVAS
  canvas.AlphaBlendNotWhite(0, 0, width, height,
                            buffer, 0, 0, width, height,
                            alpha);
#else
  canvas.AlphaBlend(0, 0, width, height,
                    buffer, 0, 0, width, height,
                    alpha);
#endif
}

#endif

#endif
