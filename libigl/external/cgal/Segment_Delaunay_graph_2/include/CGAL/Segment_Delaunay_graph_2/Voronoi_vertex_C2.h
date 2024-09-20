// Copyright (c) 2003,2004,2005,2006  INRIA Sophia-Antipolis (France).
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
// Author(s)     : Menelaos Karavelas <mkaravel@iacm.forth.gr>



#ifndef CGAL_SEGMENT_DELAUNAY_GRAPH_2_VORONOI_VERTEX_C2_H
#define CGAL_SEGMENT_DELAUNAY_GRAPH_2_VORONOI_VERTEX_C2_H

#include <CGAL/license/Segment_Delaunay_graph_2.h>



#include <CGAL/Segment_Delaunay_graph_2/basic.h>

#include <CGAL/Segment_Delaunay_graph_2/Voronoi_vertex_ring_C2.h>
#include <CGAL/Segment_Delaunay_graph_2/Voronoi_vertex_sqrt_field_new_C2.h>

#ifdef CGAL_SDG_USE_OLD_INCIRCLE
#include <CGAL/Segment_Delaunay_graph_2/Voronoi_vertex_sqrt_field_C2.h>
#endif // CGAL_SDG_USE_OLD_INCIRCLE

namespace CGAL {

namespace SegmentDelaunayGraph_2 {

namespace Internal {

  template<class K,class M> struct Which_Voronoi_vertex_base_C2;

  template<class K>
  struct Which_Voronoi_vertex_base_C2<K,Integral_domain_without_division_tag>
  {
    typedef Voronoi_vertex_ring_C2<K>          Base;
  };

  template<class K>
  struct Which_Voronoi_vertex_base_C2<K,Field_with_sqrt_tag>
  {
#ifdef CGAL_SDG_USE_OLD_INCIRCLE
    typedef Voronoi_vertex_sqrt_field_C2<K>        Base;
#else
    typedef Voronoi_vertex_sqrt_field_new_C2<K>    Base;
#endif // CGAL_SDG_USE_OLD_INCIRCLE
  };
} // namespace Internal

//----------------------------------------------------------------------

template<class K, class M>
class Voronoi_vertex_C2
  : public Internal::Which_Voronoi_vertex_base_C2<K,M>::Base
{
private:
  typedef typename Internal::Which_Voronoi_vertex_base_C2<K,M>::Base  Base;

protected:
  typedef typename Base::Site_2   Site_2;
public:
  Voronoi_vertex_C2(const Site_2& p, const Site_2& q,
		    const Site_2& r)
    : Base(p, q, r) {}
};


} //namespace SegmentDelaunayGraph_2

} //namespace CGAL



#endif // CGAL_SEGMENT_DELAUNAY_GRAPH_2_VORONOI_VERTEX_C2_H
