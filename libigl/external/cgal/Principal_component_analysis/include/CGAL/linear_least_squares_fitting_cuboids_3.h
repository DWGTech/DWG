// Copyright (c) 2005  INRIA Sophia-Antipolis (France).
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
// Author(s) : Pierre Alliez and Sylvain Pion and Ankit Gupta

#ifndef CGAL_LINEAR_LEAST_SQUARES_FITTING_CUBOIDS_3_H
#define CGAL_LINEAR_LEAST_SQUARES_FITTING_CUBOIDS_3_H

#include <CGAL/license/Principal_component_analysis.h>


#include <CGAL/basic.h>
#include <CGAL/centroid.h>
#include <CGAL/PCA_util.h>
#include <CGAL/linear_least_squares_fitting_points_3.h>
#include <CGAL/linear_least_squares_fitting_segments_3.h>

#include <list>
#include <iterator>

namespace CGAL {

namespace internal {

// fits a plane to a 3D cuboid set
template < typename InputIterator, 
           typename K,
	   typename DiagonalizeTraits >
typename K::FT
linear_least_squares_fitting_3(InputIterator first,
                               InputIterator beyond, 
                               typename K::Plane_3& plane,   // best fit plane
                               typename K::Point_3& c,       // centroid
                               const typename K::Iso_cuboid_3*,  // used for indirection
                               const K& k,                   // kernel
			       const CGAL::Dimension_tag<3>& tag,
			       const DiagonalizeTraits& diagonalize_traits)
{
  typedef typename K::Iso_cuboid_3 Iso_cuboid;

  // precondition: at least one element in the container.
  CGAL_precondition(first != beyond);
  
  // compute centroid
  c = centroid(first,beyond,k,tag);

  // assemble covariance matrix
  typename DiagonalizeTraits::Covariance_matrix covariance = {{ 0., 0., 0., 0., 0., 0. }};
  assemble_covariance_matrix_3(first,beyond,covariance,c,k,(Iso_cuboid*) NULL,tag, diagonalize_traits);

  // compute fitting plane
  return fitting_plane_3(covariance,c,plane,k,diagonalize_traits);


} // end linear_least_squares_fitting_cuboids_3

// fits a plane to a 3D cuboid set
template < typename InputIterator, 
           typename K,
	   typename DiagonalizeTraits >
typename K::FT
linear_least_squares_fitting_3(InputIterator first,
                               InputIterator beyond, 
                               typename K::Plane_3& plane,   // best fit plane
                               typename K::Point_3& c,       // centroid
                               const typename K::Iso_cuboid_3*,  // used for indirection
                               const K& k,                   // kernel
			       const CGAL::Dimension_tag<2>& tag,
			       const DiagonalizeTraits& diagonalize_traits)
{
  typedef typename K::Iso_cuboid_3 Iso_cuboid;

  // precondition: at least one element in the container.
  CGAL_precondition(first != beyond);
  
  // compute centroid
  c = centroid(first,beyond,k,tag);

  // assemble covariance matrix
  typename DiagonalizeTraits::Covariance_matrix covariance = {{ 0., 0., 0., 0., 0., 0. }};
  assemble_covariance_matrix_3(first,beyond,covariance,c,k,(Iso_cuboid*) NULL,tag,diagonalize_traits);

  // compute fitting plane
  return fitting_plane_3(covariance,c,plane,k,diagonalize_traits);


} // end linear_least_squares_fitting_cuboids_3

// fits a plane to a 3D cuboid set
template < typename InputIterator, 
           typename K,
	   typename DiagonalizeTraits >
typename K::FT
linear_least_squares_fitting_3(InputIterator first,
                               InputIterator beyond, 
                               typename K::Plane_3& plane,   // best fit plane
                               typename K::Point_3& c,       // centroid
                               const typename K::Iso_cuboid_3*,  // used for indirection
                               const K& k,                   // kernel
			       const CGAL::Dimension_tag<1>& tag,
			       const DiagonalizeTraits& diagonalize_traits)
{
  typedef typename K::Segment_3 Segment;
  typedef typename K::Iso_cuboid_3 Iso_cuboid;

  // precondition: at least one element in the container.
  CGAL_precondition(first != beyond);
  
  std::list<Segment> segments;
  for(InputIterator it = first;
      it != beyond;
      it++)
  {
    const Iso_cuboid& t = *it;
    segments.push_back(Segment(t[0],t[1]));
    segments.push_back(Segment(t[1],t[2]));
    segments.push_back(Segment(t[2],t[3]));    
    segments.push_back(Segment(t[3],t[0]));
    segments.push_back(Segment(t[4],t[5]));
    segments.push_back(Segment(t[5],t[6]));
    segments.push_back(Segment(t[6],t[7]));
    segments.push_back(Segment(t[7],t[4]));
    segments.push_back(Segment(t[5],t[0]));
    segments.push_back(Segment(t[1],t[6]));
    segments.push_back(Segment(t[2],t[7]));
    segments.push_back(Segment(t[3],t[4]));
  }

  // compute fitting plane
  return linear_least_squares_fitting_3(segments.begin(),segments.end(),plane,c,(Segment*)NULL,k,tag,
					diagonalize_traits);

} // end linear_least_squares_fitting_cuboids_3

// fits a plane to a 3D cuboid set
template < typename InputIterator, 
           typename K,
	   typename DiagonalizeTraits >
typename K::FT
linear_least_squares_fitting_3(InputIterator first,
                               InputIterator beyond, 
                               typename K::Plane_3& plane,   // best fit plane
                               typename K::Point_3& c,       // centroid
                               const typename K::Iso_cuboid_3*,  // used for indirection
                               const K& k,                   // kernel
			       const CGAL::Dimension_tag<0>& tag,
			       const DiagonalizeTraits& diagonalize_traits)
{
  typedef typename K::Point_3 Point;
  typedef typename K::Iso_cuboid_3 Iso_cuboid;

  // precondition: at least one element in the container.
  CGAL_precondition(first != beyond);
  
  std::list<Point> points;
  for(InputIterator it = first;
      it != beyond;
      it++)
  {
    const Iso_cuboid& t = *it;
    points.push_back(t[0]);
    points.push_back(t[1]);
    points.push_back(t[2]);    
    points.push_back(t[3]);
    points.push_back(t[4]);
    points.push_back(t[5]);
    points.push_back(t[6]);    
    points.push_back(t[7]);
  }

  // compute fitting plane
  return linear_least_squares_fitting_3(points.begin(),points.end(),plane,c,(Point*)NULL,k,tag,
					diagonalize_traits);

} // end linear_least_squares_fitting_cuboids_3

// fits a line to a 3D cuboid set
template < typename InputIterator, 
           typename K,
	   typename DiagonalizeTraits >
typename K::FT
linear_least_squares_fitting_3(InputIterator first,
                               InputIterator beyond, 
                               typename K::Line_3& line,     // best fit line
                               typename K::Point_3& c,       // centroid
                               const typename K::Iso_cuboid_3*,  // used for indirection
                               const K& k,                   // kernel
			       const CGAL::Dimension_tag<3>& tag,
			       const DiagonalizeTraits& diagonalize_traits)
{
  typedef typename K::Iso_cuboid_3 Iso_cuboid;

  // precondition: at least one element in the container.
  CGAL_precondition(first != beyond);
  
  // compute centroid
  c = centroid(first,beyond,k,tag);

  // assemble covariance matrix
  typename DiagonalizeTraits::Covariance_matrix covariance = {{ 0., 0., 0., 0., 0., 0. }};
  assemble_covariance_matrix_3(first,beyond,covariance,c,k,(Iso_cuboid*) NULL,tag,diagonalize_traits);
  
  // compute fitting line
  return fitting_line_3(covariance,c,line,k,diagonalize_traits);
  
} // end linear_least_squares_fitting_cuboids_3

// fits a line to a 3D cuboid set
template < typename InputIterator, 
           typename K,
	   typename DiagonalizeTraits >
typename K::FT
linear_least_squares_fitting_3(InputIterator first,
                               InputIterator beyond, 
                               typename K::Line_3& line,   // best fit line
                               typename K::Point_3& c,       // centroid
                               const typename K::Iso_cuboid_3*,  // used for indirection
                               const K& k,                   // kernel
			       const CGAL::Dimension_tag<2>& tag,
			       const DiagonalizeTraits& diagonalize_traits)
{
  typedef typename K::Iso_cuboid_3 Iso_cuboid;

  // precondition: at least one element in the container.
  CGAL_precondition(first != beyond);
  
  // compute centroid
  c = centroid(first,beyond,k,tag);

  // assemble covariance matrix
  typename DiagonalizeTraits::Covariance_matrix covariance = {{ 0., 0., 0., 0., 0., 0. }};
  assemble_covariance_matrix_3(first,beyond,covariance,c,k,(Iso_cuboid*) NULL,tag, diagonalize_traits);
  
  // compute fitting line
  return fitting_line_3(covariance,c,line,k,diagonalize_traits);


} // end linear_least_squares_fitting_cuboids_3

// fits a line to a 3D cuboid set
template < typename InputIterator, 
           typename K,
	   typename DiagonalizeTraits >
typename K::FT
linear_least_squares_fitting_3(InputIterator first,
                               InputIterator beyond, 
                               typename K::Line_3& line,   // best fit line
                               typename K::Point_3& c,       // centroid
                               const typename K::Iso_cuboid_3*,  // used for indirection
                               const K& k,                   // kernel
			       const CGAL::Dimension_tag<1>& tag,
			       const DiagonalizeTraits& diagonalize_traits)
{
  typedef typename K::Segment_3 Segment;
  typedef typename K::Iso_cuboid_3 Iso_cuboid;

  // precondition: at least one element in the container.
  CGAL_precondition(first != beyond);
  
  std::list<Segment> segments;
  for(InputIterator it = first;
      it != beyond;
      it++)
  {
    const Iso_cuboid& t = *it;
    segments.push_back(Segment(t[0],t[1]));
    segments.push_back(Segment(t[1],t[2]));
    segments.push_back(Segment(t[2],t[3]));    
    segments.push_back(Segment(t[3],t[0]));
    segments.push_back(Segment(t[4],t[5]));
    segments.push_back(Segment(t[5],t[6]));
    segments.push_back(Segment(t[6],t[7]));
    segments.push_back(Segment(t[7],t[4]));
    segments.push_back(Segment(t[5],t[0]));
    segments.push_back(Segment(t[1],t[6]));
    segments.push_back(Segment(t[2],t[7]));
    segments.push_back(Segment(t[3],t[4]));
  }

  // compute fitting line
  return linear_least_squares_fitting_3(segments.begin(),segments.end(),line,c,(Segment*)NULL,k,tag,
					diagonalize_traits);

} // end linear_least_squares_fitting_cuboids_3

// fits a line to a 3D cuboid set
template < typename InputIterator, 
           typename K,
	   typename DiagonalizeTraits >
typename K::FT
linear_least_squares_fitting_3(InputIterator first,
                               InputIterator beyond, 
                               typename K::Line_3& line,   // best fit line
                               typename K::Point_3& c,       // centroid
                               const typename K::Iso_cuboid_3*,  // used for indirection
                               const K& k,                   // kernel
			       const CGAL::Dimension_tag<0>& tag,
			       const DiagonalizeTraits& diagonalize_traits)
{
  typedef typename K::Point_3 Point;
  typedef typename K::Iso_cuboid_3 Iso_cuboid;

  // precondition: at least one element in the container.
  CGAL_precondition(first != beyond);
  
  std::list<Point> points;
  for(InputIterator it = first;
      it != beyond;
      it++)
  {
    const Iso_cuboid& t = *it;
    points.push_back(t[0]);
    points.push_back(t[1]);
    points.push_back(t[2]);    
    points.push_back(t[3]);
    points.push_back(t[4]);
    points.push_back(t[5]);
    points.push_back(t[6]);    
    points.push_back(t[7]);
  }

  // compute fitting line
  return linear_least_squares_fitting_3(points.begin(),points.end(),line,c,(Point*)NULL,k,tag,
					diagonalize_traits);

} // end linear_least_squares_fitting_cuboids_3

} // end namespace internal

} //namespace CGAL

#endif // CGAL_LINEAR_LEAST_SQUARES_FITTING_CUBOIDS_3_H
