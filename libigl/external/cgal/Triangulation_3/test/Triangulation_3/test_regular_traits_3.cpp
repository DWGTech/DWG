// Copyright (c) 1998  INRIA Sophia-Antipolis (France).
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
// $Date$
// 
//
// Author(s)     : Mariette Yvinec

#define CGAL_NO_DEPRECATION_WARNINGS

#include <cassert>
#include <CGAL/Regular_triangulation_euclidean_traits_3.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <CGAL/_test_cls_regular_euclidean_traits_3.h>

//needs exact constructions as well to test the traits class

typedef CGAL::Exact_predicates_exact_constructions_kernel K;

// Explicit instantiation of the whole class :
template class CGAL::Regular_triangulation_euclidean_traits_3<K, K::FT>;

int main()
{
  typedef CGAL::Regular_triangulation_euclidean_traits_3<K> Traits;
  _test_cls_regular_euclidean_traits_3(Traits() );
  std::cerr << "done"<< std::endl;

  return 0;
}
