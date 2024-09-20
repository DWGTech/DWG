// Copyright (c) 2005-2008 Max-Planck-Institute Saarbruecken (Germany).
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
// Author(s)     :  Peter Hachenberger <hachenberger@mpi-sb.mpg.de>

#ifndef CGAL_CD3_INSERT_VERTEX_INTO_EDGE_H
#define CGAL_CD3_INSERT_VERTEX_INTO_EDGE_H

#include <CGAL/license/Convex_decomposition_3.h>


#include <CGAL/Nef_3/SNC_constructor.h>

namespace CGAL {

template<class SNC_structure, class SNC_point_locator>
class Insert_vertex_into_edge {

  typedef typename SNC_structure::Vertex_handle Vertex_handle;
  typedef typename SNC_structure::SVertex_handle SVertex_handle;
  typedef typename SNC_structure::SVertex_iterator SVertex_iterator;
  typedef typename SNC_structure::Point_3 Point_3;
  typedef typename SNC_structure::Items Items;

  SNC_structure& snc;
  SNC_point_locator& pl;

 public:
  Insert_vertex_into_edge(SNC_structure& snc_,
			  SNC_point_locator& pl_) 
    : snc(snc_), pl(pl_) {}

  SVertex_handle operator() 
    (SVertex_handle e, const Point_3 ip) 
  {
    CGAL::SNC_constructor<Items, SNC_structure> C(snc);
    Vertex_handle v;
    v = C.create_from_edge(e, ip);
    
    pl.add_vertex(v);
    
    SVertex_iterator svi = v->svertices_begin();
    SVertex_handle svf, svb;
    if(svi->point() == e->point()) {
      svf = svi;
      svb = ++svi;
    } else {
      svb = svi;
      svf = ++svi;
    }
    
    svb->twin() = e;
    svf->twin() = e->twin();
    e->twin()->twin() = svf;
    e->twin() = svb;
    
    pl.add_edge(svf);
    pl.add_edge(svb);

    return svf;
  }

};

} //namespace CGAL
#endif // CGAL_CD3_INSERT_VERTEX_INTO_EDGE_H
