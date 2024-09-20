// Copyright (c) 2008  GeometryFactory Sarl (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
// You can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL$
// $Id$
// SPDX-License-Identifier: GPL-3.0+
// 
//
// Author(s)     : Andreas Fabri <Andreas.Fabri@geometryfactory.com>
//                 Laurent Rineau <Laurent.Rineau@geometryfactory.com>

#ifndef CGAL_QT_UTILITY_H
#define CGAL_QT_UTILITY_H

#include <CGAL/license/GraphicsView.h>


#include <QRectF>
#include <QRect>

#include <CGAL/auto_link/Qt.h>
#include <CGAL/export/Qt.h>

class QGraphicsScene;
class QGraphicsView;

namespace CGAL {
namespace Qt {

CGAL_QT_EXPORT QRectF mapToScene(const QGraphicsView* v, const QRect rect);
CGAL_QT_EXPORT QRectF viewportsBbox(const QGraphicsScene*);

} // namespace Qt
} // namespace CGAL

#ifdef CGAL_HEADER_ONLY
#include <CGAL/Qt/utility_impl.h>
#endif // CGAL_HEADER_ONLY

#endif // CGAL_QT_UTILITY_H
