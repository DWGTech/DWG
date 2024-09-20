// Copyright (c) 2016 CNRS and LIRIS' Establishments (France).
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
// Author(s)     : Guillaume Damiand <guillaume.damiand@liris.cnrs.fr>
//
#ifndef CGAL_GMAP_DART_CONST_ITERATORS_HH
#define CGAL_GMAP_DART_CONST_ITERATORS_HH 1

#include <CGAL/GMap_dart_iterators.h>

namespace CGAL {

  /** @file GMap_dart_const_iterators.h
   * Definition of gmap dart const iterators.
   * There are 7 iterators:
   *  - GMap_dart_const_iterator_basic_of_orbit<Map,Alpha...>
   *  - GMap_dart_const_iterator_basic_of_cell<Map,i,d>
   *  - GMap_dart_const_iterator_basic_of_involution<Map,i,d>
   *  - GMap_dart_const_iterator_basic_of_all<Map>
   *  - GMap_dart_const_iterator_of_orbit<Map,Alpha...>
   *  - GMap_dart_const_iterator_of_cell<Map,i,d>
   *  - GMap_dart_const_iterator_of_involution<Map,i,d>
   */
  //****************************************************************************
#ifndef CGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES
  template<typename Map_,unsigned int...Alpha>
  class GMap_dart_const_iterator_basic_of_orbit:
    public GMap_dart_iterator_basic_of_orbit_generic<Map_,true,Alpha...>
  {
  public:
    typedef GMap_dart_const_iterator_basic_of_orbit<Map_,Alpha...> Self;
    typedef GMap_dart_iterator_basic_of_orbit_generic<Map_,true,Alpha...> Base;

    typedef typename Map_::Dart_const_handle Dart_const_handle;
    typedef typename Map_::size_type size_type;

    /// Main constructor.
    GMap_dart_const_iterator_basic_of_orbit(const Map_& amap,
                                            Dart_const_handle adart):
      Base(amap,adart)
    {}
    /// Main constructor.
    GMap_dart_const_iterator_basic_of_orbit(const Map_& amap,
                                            Dart_const_handle adart,
                                            size_type amark):
      Base(amap,adart,amark)
    {}
    /// Constructor from non const version.
    GMap_dart_const_iterator_basic_of_orbit
    (const GMap_dart_const_iterator_basic_of_orbit<Map_,Alpha...>& it):
      Base(*const_cast<const Map_*>(it.get_combinatorial_map()),
           it.get_first_dart(),
           it.mmark_number)
    {}
  };
  //****************************************************************************
  template<typename Map_,unsigned int...Alpha>
  class GMap_dart_const_iterator_of_orbit:
    public GMap_dart_iterator_of_orbit_generic<Map_,true,Alpha...>
  {
  public:
    typedef GMap_dart_const_iterator_of_orbit<Map_,Alpha...> Self;
    typedef GMap_dart_iterator_of_orbit_generic<Map_,true,Alpha...> Base;

    typedef typename Map_::Dart_const_handle Dart_const_handle;

    /// Main constructor.
    GMap_dart_const_iterator_of_orbit(const Map_& amap,
                                      Dart_const_handle adart):
      Base(amap,adart)
    {}
    /// Constructor from non const version.
    GMap_dart_const_iterator_of_orbit
    (const GMap_dart_iterator_of_orbit<Map_,Alpha...>& it):
      Base(*const_cast<const Map_*>(it.get_combinatorial_map()),
	   it.get_first_dart())
    {}
  };
#else
  //****************************************************************************
  template<typename Map_,int B1=-1,int B2=-1,int B3=-1,int B4=-1,int B5=-1,
	   int B6=-1,int B7=-1,int B8=-1,int B9=-1>
    class GMap_dart_const_iterator_basic_of_orbit:
    public GMap_dart_iterator_basic_of_orbit_generic<Map_,true,B1,B2,B3,B4,
                                                     B5,B6,B7,B8,B9>
    {
    public:
      typedef GMap_dart_const_iterator_basic_of_orbit<Map_,B1,B2,B3,B4,B5,B6,
                                                      B7,B8,B9> Self;
      typedef GMap_dart_iterator_basic_of_orbit_generic<Map_,true,B1,B2,B3,B4,
                                                        B5,B6,B7,B8,B9> Base;

      typedef typename Map_::Dart_const_handle Dart_const_handle;
      typedef typename Map_::size_type size_type;

      /// Main constructor.
      GMap_dart_const_iterator_basic_of_orbit(const Map_& amap,
                                              Dart_const_handle adart):
        Base(amap,adart)
      {}
      /// Main constructor.
      GMap_dart_const_iterator_basic_of_orbit(const Map_& amap,
                                              Dart_const_handle adart,
                                              size_type amark):
        Base(amap,adart,amark)
      {}
      /// Constructor from non const version.
      GMap_dart_const_iterator_basic_of_orbit
      (const GMap_dart_const_iterator_basic_of_orbit<Map_,B1,B2,B3,B4,B5,B6,
       B7,B8,B9>& it):
        Base(*const_cast<const Map_*>(it.get_combinatorial_map()),
             it.get_first_dart(),
             it.mmark_number)
      {}
    };
  //****************************************************************************
  template<typename Map_,int B1=-1,int B2=-1,int B3=-1,int B4=-1,int B5=-1,
	   int B6=-1,int B7=-1,int B8=-1,int B9=-1>
    class GMap_dart_const_iterator_of_orbit:
    public GMap_dart_iterator_of_orbit_generic<Map_,true,B1,B2,B3,B4,
                                               B5,B6,B7,B8,B9>
    {
    public:
      typedef GMap_dart_const_iterator_of_orbit<Map_,B1,B2,B3,B4,B5,B6,
                                                B7,B8,B9> Self;
      typedef GMap_dart_iterator_of_orbit_generic<Map_,true,B1,B2,B3,B4,
                                                  B5,B6,B7,B8,B9> Base;

      typedef typename Map_::Dart_const_handle Dart_const_handle;

      /// Main constructor.
      GMap_dart_const_iterator_of_orbit(const Map_& amap,
                                        Dart_const_handle adart):
        Base(amap,adart)
      {}
      /// Constructor from non const version.
      GMap_dart_const_iterator_of_orbit
      (const GMap_dart_iterator_of_orbit<Map_,B1,B2,B3,B4,B5,B6,B7,B8,B9>& it):
        Base(*const_cast<const Map_*>(it.get_combinatorial_map()),
             it.get_first_dart())
      {}
    };
#endif // CGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES
  //****************************************************************************
  template<typename Map_>
  class GMap_dart_const_iterator_basic_of_all:
    public GMap_dart_iterator_basic_of_all<Map_,true>
  {
  public:
    typedef GMap_dart_iterator_basic_of_all<Map_,true> Base;
    typedef typename Map_::Dart_const_handle Dart_const_handle;
    typedef typename Map_::size_type size_type;

    /* Main constructor. */
    GMap_dart_const_iterator_basic_of_all(const Map_& amap,
                                          Dart_const_handle adart):
      Base(amap,adart)
    {}
    /* Main constructor. */
    GMap_dart_const_iterator_basic_of_all(const Map_& amap,
                                          Dart_const_handle adart,
                                          size_type /*amark*/):
      Base(amap,adart)
    {}
    /// Constructor from non const version.
    GMap_dart_const_iterator_basic_of_all
    (const GMap_dart_iterator_basic_of_all<Map_,false>& it):
      Base(*const_cast<const Map_*>(it.get_combinatorial_map()),
           it.get_first_dart())
    {}
  };
  //****************************************************************************
  template<typename Map_,int i,int d=Map_::dimension>
  class GMap_dart_const_iterator_basic_of_cell:
    public GMap_dart_iterator_basic_of_cell<Map_,i,d,true>
  {
  public:
    typedef GMap_dart_iterator_basic_of_cell<Map_,i,d,true> Base;
    typedef typename Map_::Dart_const_handle Dart_const_handle;
    typedef typename Map_::size_type size_type;

    /* Main constructor. */
    GMap_dart_const_iterator_basic_of_cell(const Map_& amap,
                                           Dart_const_handle adart):
      Base(amap,adart)
    {}
    /* Main constructor. */
    GMap_dart_const_iterator_basic_of_cell(const Map_& amap,
                                           Dart_const_handle adart, 
                                           size_type amark):
      Base(amap,adart,amark)
    {}
    /// Constructor from non const version.
    GMap_dart_const_iterator_basic_of_cell
    (const GMap_dart_iterator_basic_of_cell<Map_,i,d,false>& it):
      Base(*const_cast<const Map_*>(it.get_combinatorial_map()),
           it.get_first_dart())
    {}
  };
  //****************************************************************************
  template<typename Map_, int i, int d=Map_::dimension>
  class GMap_dart_const_iterator_of_cell:
    public GMap_dart_iterator_of_cell<Map_,i,d,true>
  {
  public:
    typedef GMap_dart_iterator_of_cell<Map_,i,d,true> Base;
    typedef typename Map_::Dart_const_handle Dart_const_handle;

    /* Main constructor. */
    GMap_dart_const_iterator_of_cell(const Map_& amap,
				     Dart_const_handle adart):
      Base(amap,adart)
    {}
    /// Constructor from non const version.
    GMap_dart_const_iterator_of_cell
    (const GMap_dart_iterator_of_cell<Map_,i,d,false>& it):
      Base(*const_cast<const Map_*>(it.get_combinatorial_map()),
	   it.get_first_dart())
    {}
  };
  //****************************************************************************
  template<typename Map_,int i,int d=Map_::dimension>
  class GMap_dart_const_iterator_basic_of_involution:
    public GMap_dart_iterator_basic_of_involution<Map_,i,d,true>
  {
  public:
    typedef GMap_dart_iterator_basic_of_involution<Map_,i,d,true> Base;
    typedef typename Map_::Dart_const_handle Dart_const_handle;
    typedef typename Map_::size_type size_type;

    /* Main constructor. */
    GMap_dart_const_iterator_basic_of_involution(const Map_& amap,
						 Dart_const_handle adart):
      Base(amap,adart)
    {}
    /* Main constructor. */
    GMap_dart_const_iterator_basic_of_involution(const Map_& amap,
						 Dart_const_handle adart,
						 size_type amark):
      Base(amap,adart,amark)
    {}
    /// Constructor from non const version.
    GMap_dart_const_iterator_basic_of_involution
    (const GMap_dart_iterator_basic_of_involution<Map_,i,d,false>& it):
      Base(*const_cast<const Map_*>(it.get_combinatorial_map()),
	   it.get_first_dart(), it.mmark_number)
    {}
  };
  //****************************************************************************
  template<typename Map_,int i,int d=Map_::dimension>
  class GMap_dart_const_iterator_of_involution:
    public GMap_dart_iterator_of_involution<Map_,i,d,true>
  {
  public:
    typedef GMap_dart_iterator_of_involution<Map_,i,d,true> Base;
    typedef typename Map_::Dart_const_handle Dart_const_handle;
    typedef typename Map_::size_type size_type;

    /* Main constructor. */
    GMap_dart_const_iterator_of_involution(const Map_& amap,
					   Dart_const_handle adart):
      Base(amap,adart)
    {}
    /// Constructor from non const version.
    GMap_dart_const_iterator_of_involution
    (const GMap_dart_iterator_of_involution<Map_,i,d,false>& it):
      Base(*const_cast<const Map_*>(it.get_combinatorial_map()),
	   it.get_first_dart())
    {}
  };
  //****************************************************************************
} // namespace CGAL
//******************************************************************************
#endif // CGAL_GMAP_DART_CONST_ITERATORS_HH
//******************************************************************************
