// Copyright (c) 2008  INRIA Sophia-Antipolis (France).
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
// Author(s) : Monique Teillaud, Sylvain Pion, Pedro Machado

// Partially supported by the IST Programme of the EU as a 
// STREP (FET Open) Project under Contract No  IST-006413 
// (ACS -- Algorithms for Complex Shapes)

#include <CGAL/Exact_spherical_kernel_3.h>
#include <CGAL/_test_sphere_predicates.h>
#include <CGAL/_test_sphere_constructions.h>
#include <CGAL/_test_sphere_compute.h>
#include <CGAL/_test_functionalities_on_sphere.h>

int main()
{ 
  typedef CGAL::Exact_spherical_kernel_3  SK;
  SK  sk1;
  _test_spherical_kernel_predicates(sk1);
  _test_spherical_kernel_construct(sk1); 
  _test_spherical_kernel_compute(sk1);
  test_functionalities_on_a_reference_sphere<CGAL::Exact_spherical_kernel_3>(SK::Point_3(4.5,1.2,0.7));
  return 0;
}

