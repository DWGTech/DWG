// Copyright (c) 2006-2008 Max-Planck-Institute Saarbruecken (Germany).
// All rights reserved.
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
// Author(s)     : Michael Hemmer   <hemmer@mpi-inf.mpg.de>


#ifndef CGAL_SQRT_EXTENSION_GET_ARITHMETIC_KERNEL_H
#define CGAL_SQRT_EXTENSION_GET_ARITHMETIC_KERNEL_H

#include <CGAL/basic.h>
#include <CGAL/Get_arithmetic_kernel.h>

namespace CGAL {

template <class COEFF, class ROOT, class ACDE_TAG,class FP_TAG>
struct Get_arithmetic_kernel<Sqrt_extension<COEFF,ROOT,ACDE_TAG,FP_TAG> >{
  typedef Get_arithmetic_kernel<COEFF> GET;
  typedef typename GET::Arithmetic_kernel Arithmetic_kernel;
};

} //namespace CGAL

#endif
