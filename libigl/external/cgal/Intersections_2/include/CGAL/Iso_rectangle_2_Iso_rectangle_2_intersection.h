// Copyright (c) 2000  
// Utrecht University (The Netherlands),
// ETH Zurich (Switzerland),
// INRIA Sophia-Antipolis (France),
// Max-Planck-Institute Saarbruecken (Germany),
// and Tel-Aviv University (Israel).  All rights reserved. 
//
// This file is part of CGAL (www.cgal.org); you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation; either version 3 of the License,
// or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL$
// $Id$
// SPDX-License-Identifier: LGPL-3.0+
// 
//
// Author(s)     : Geert-Jan Giezeman


#ifndef CGAL_ISO_RECTANGLE_2_ISO_RECTANGLE_2_INTERSECTION_H
#define CGAL_ISO_RECTANGLE_2_ISO_RECTANGLE_2_INTERSECTION_H

#include <CGAL/Iso_rectangle_2.h>
#include <CGAL/Intersection_traits_2.h>

namespace CGAL {

namespace internal {

template <class K>
typename CGAL::Intersection_traits
<K, typename K::Iso_rectangle_2, typename K::Iso_rectangle_2>::result_type
intersection(
    const typename K::Iso_rectangle_2 &irect1,
    const typename K::Iso_rectangle_2 &irect2,
    const K&)
{
    typedef typename K::FT  FT;
    Rational_traits<FT>  rt;
    typename K::Construct_point_2 construct_point_2;
    typename K::Construct_iso_rectangle_2 construct_iso_rectangle_2;
    const typename K::Point_2 &min1 = (irect1.min)();
    const typename K::Point_2 &min2 = (irect2.min)();
    const typename K::Point_2 &max1 = (irect1.max)();
    const typename K::Point_2 &max2 = (irect2.max)();
    typename K::FT minx, miny, maxx, maxy;
    typename K::Point_2 newmin;
    typename K::Point_2 newmax;
    minx = (min1.x() >= min2.x()) ? min1.x() : min2.x();
    maxx = (max1.x() <= max2.x()) ? max1.x() : max2.x();
    if (maxx < minx)
        return intersection_return<typename K::Intersect_2, typename K::Iso_rectangle_2, typename K::Iso_rectangle_2>();
    miny = (min1.y() >= min2.y()) ? min1.y() : min2.y();
    maxy = (max1.y() <= max2.y()) ? max1.y() : max2.y();
    if (maxy < miny)
        return intersection_return<typename K::Intersect_2, typename K::Iso_rectangle_2, typename K::Iso_rectangle_2>();
    if (rt.denominator(minx) == rt.denominator(miny)) {
        newmin = construct_point_2(rt.numerator(minx), rt.numerator(miny),
				   rt.denominator(minx));
    } else {
        newmin = construct_point_2(rt.numerator(minx)   * rt.denominator(miny),
				   rt.numerator(miny)   * rt.denominator(minx),
				   rt.denominator(minx) * rt.denominator(miny));
    }
    if (rt.denominator(maxx) == rt.denominator(maxy)) {
        newmax = construct_point_2(rt.numerator(maxx), rt.numerator(maxy),
				   rt.denominator(maxx));
    } else {
        newmax = construct_point_2(rt.numerator(maxx)   * rt.denominator(maxy),
				   rt.numerator(maxy)   * rt.denominator(maxx),
				   rt.denominator(maxx) * rt.denominator(maxy));
    }
    return intersection_return<typename K::Intersect_2, typename K::Iso_rectangle_2, typename K::Iso_rectangle_2>(construct_iso_rectangle_2(newmin, newmax));
}

template<typename K>
inline bool
do_intersect(const typename K::Iso_rectangle_2 &irect1,
             const typename K::Iso_rectangle_2 &irect2,
             const K&) {
  return bool(intersection(irect1, irect2));
}

} // namespace internal


CGAL_INTERSECTION_FUNCTION_SELF(Iso_rectangle_2, 2)
CGAL_DO_INTERSECT_FUNCTION_SELF(Iso_rectangle_2, 2)

} //namespace CGAL

#endif
