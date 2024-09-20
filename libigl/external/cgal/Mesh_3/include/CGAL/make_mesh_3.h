// Copyright (c) 2009 INRIA Sophia-Antipolis (France).
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
// Author(s)     : Stéphane Tayeb
//
//******************************************************************************
// File Description : make_mesh_3 function definition.
//******************************************************************************

#ifndef CGAL_MAKE_MESH_3_H
#define CGAL_MAKE_MESH_3_H

#include <CGAL/license/Mesh_3.h>

#include <CGAL/disable_warnings.h>

#include <CGAL/Mesh_3/config.h>
#include <CGAL/Mesh_3/global_parameters.h>
#include <CGAL/refine_mesh_3.h>
#include <CGAL/tags.h>
#include <CGAL/Mesh_3/Protect_edges_sizing_field.h>
#include <CGAL/Mesh_3/Has_features.h>
#include <CGAL/Mesh_3/C3T3_helpers.h>

#include <boost/mpl/has_xxx.hpp>

namespace CGAL {
  
namespace parameters {
  namespace internal {
    // Features
    struct Features_options
    {
      Features_options(bool b) : b_(b) {}
      bool features() const { return b_; }
    private:
      bool b_;
    };
    
    // -----------------------------------
    // Features generator
    // -----------------------------------
    // struct Features_option_generator
    template <typename HasFeatures>
    struct Features_options_generator {};
    
    template<>
    struct Features_options_generator<CGAL::Tag_true>
    {
      Features_options operator()() { return Features_options(true); }
    };
    
    template<>
    struct Features_options_generator<CGAL::Tag_false>
    {
      Features_options operator()() { return Features_options(false); }
    };
    
    // struct Domain_features_generator is designed to handle cases where
    // MeshDomain::Has_features is not a valid type
    template< typename MeshDomain, bool MeshDomainHasHasFeatures >
    struct Domain_features_generator {};
    
    template< typename MeshDomain >
    struct Domain_features_generator< MeshDomain, false >
    {
      Features_options operator()()
      { 
        return Features_options_generator<CGAL::Tag_false>()();
      }
    };
    
    template< typename MeshDomain >
    struct Domain_features_generator< MeshDomain, true >
    {
      Features_options operator()()
      { 
        return Features_options_generator<typename MeshDomain::Has_features>()();
      }
    };

  } // end namespace internal

  // -----------------------------------
  // Features_options
  // -----------------------------------
  inline internal::Features_options
  features() { return internal::Features_options(true); }

  inline internal::Features_options
  no_features() { return internal::Features_options(false); }
  
  template < typename MeshDomain >
  inline internal::Features_options
  features(const MeshDomain& /*domain*/)
  {
    typedef typename internal::Domain_features_generator<
      MeshDomain,
      CGAL::internal::Mesh_3::has_Has_features<MeshDomain>::value > Generator;
    
    return Generator()();
  }
  
  // -----------------------------------
  // Parameters
  // -----------------------------------

// see <CGAL/config.h>
CGAL_PRAGMA_DIAG_PUSH
// see <CGAL/Mesh_3/config.h>
CGAL_MESH_3_IGNORE_BOOST_PARAMETER_NAME_WARNINGS

  BOOST_PARAMETER_NAME( features_param )

CGAL_PRAGMA_DIAG_POP

  
} // end namespace parameters::internal


// -----------------------------------
// Initialize c3t3 stuff
// -----------------------------------
namespace internal {
namespace Mesh_3 {

template < typename C3T3, typename MeshDomain, typename MeshCriteria >
void
init_c3t3(C3T3& c3t3, const MeshDomain& domain, const MeshCriteria&,
          const int nb_initial_points)
{
  typedef typename MeshDomain::Point_3 Point_3;
  typedef typename MeshDomain::Index Index;
  typedef std::vector<std::pair<Point_3, Index> > Initial_points_vector;
  typedef typename Initial_points_vector::iterator Ipv_iterator;
  typedef typename C3T3::Vertex_handle Vertex_handle;
  
  // Mesh initialization : get some points and add them to the mesh
  Initial_points_vector initial_points;
  if (nb_initial_points > 0)
    domain.construct_initial_points_object()(std::back_inserter(initial_points),
                                             nb_initial_points);
  else //use default number of points
    domain.construct_initial_points_object()(std::back_inserter(initial_points));

  typename C3T3::Triangulation::Geom_traits::Construct_weighted_point_3 p2wp =
      c3t3.triangulation().geom_traits().construct_weighted_point_3_object();

  // Insert points and set their index and dimension
  for ( Ipv_iterator it = initial_points.begin() ;
       it != initial_points.end() ;
       ++it )
  {
    Vertex_handle v = c3t3.triangulation().insert(p2wp(it->first));
    
    // v could be null if point is hidden
    if ( v != Vertex_handle() )
    {
      c3t3.set_dimension(v,2); // by construction, points are on surface
      c3t3.set_index(v,it->second);      
    }
  }
}

template < typename EdgeCriteria >
struct Edge_criteria_sizing_field_wrapper
{
  typedef typename EdgeCriteria::Index    Index;
  typedef typename EdgeCriteria::FT       FT;
  typedef typename EdgeCriteria::Point_3  Point_3;
  
  Edge_criteria_sizing_field_wrapper(const EdgeCriteria& ec) : ec_(ec) {}
  FT operator()(const Point_3& p, const int dim, const Index& index) const
  { return ec_.sizing_field(p,dim,index); }

private:
  // No need to copy EdgeCriteria here
  const EdgeCriteria& ec_;
};

template < typename C3T3, typename MeshDomain, typename MeshCriteria >
void
init_c3t3_with_features(C3T3& c3t3,
                        const MeshDomain& domain,
                        const MeshCriteria& criteria,
                        bool nonlinear = false)
{
  typedef typename MeshCriteria::Edge_criteria Edge_criteria;
  typedef Edge_criteria_sizing_field_wrapper<Edge_criteria> Sizing_field;

  CGAL::Mesh_3::Protect_edges_sizing_field<C3T3,MeshDomain,Sizing_field>     
    protect_edges(c3t3, domain, Sizing_field(criteria.edge_criteria_object()));
  protect_edges.set_nonlinear_growth_of_balls(nonlinear);
  
  protect_edges(true);
}
  

// C3t3_initializer: initialize c3t3
template < typename C3T3,
           typename MeshDomain,
           typename MeshCriteria,
           bool MeshDomainHasHasFeatures,
           typename HasFeatures = int>
struct C3t3_initializer {};
  
// Partial specialization of C3t3_initializer
// Handle cases where MeshDomain::Has_features is not a valid type
template < typename C3T3, typename MD, typename MC, typename HasFeatures >
struct C3t3_initializer < C3T3, MD, MC, false, HasFeatures >
{
  void operator()(C3T3& c3t3,
                  const MD& domain,
                  const MC& criteria,
                  bool with_features,
                  bool /* nonlinear */= false,
                  const int nb_initial_points = -1)
  {
    if ( with_features )
    {
      std::cerr << "Warning: you requested a mesh with features from a domain"
                << " without features !" << std::endl;
    }
    
    init_c3t3(c3t3,domain,criteria,nb_initial_points);
  }
};

// Partial specialization of C3t3_initializer
// Handles cases where MeshDomain::Has_features is a valid type
template < typename C3T3, typename MD, typename MC, typename HasFeatures >
struct C3t3_initializer < C3T3, MD, MC, true, HasFeatures >
{
  void operator()(C3T3& c3t3,
                  const MD& domain,
                  const MC& criteria,
                  bool with_features,
                  bool nonlinear = false,
                  const int nb_initial_points = -1)
  {
    C3t3_initializer < C3T3, MD, MC, true, typename MD::Has_features >()
      (c3t3,domain,criteria,with_features,nonlinear,nb_initial_points);
  }  
};

// Partial specialization of C3t3_initializer
// Handles cases where MeshDomain::Has_features is a valid type and is defined
// to CGAL::Tag_true
template < typename C3T3, typename MD, typename MC >
struct C3t3_initializer < C3T3, MD, MC, true, CGAL::Tag_true >
{
  void operator()(C3T3& c3t3,
                  const MD& domain,
                  const MC& criteria,
                  bool with_features,
                  bool nonlinear = false,
                  const int nb_initial_points = -1)
  {
    if ( with_features ) {
      init_c3t3_with_features(c3t3,domain,criteria,nonlinear);

      // If c3t3 initialization is not sufficient (may happen if there is only
      // a planar curve as feature for example), add some surface points

      bool need_more_init = c3t3.triangulation().dimension() != 3;
      if(!need_more_init) {
        CGAL::Mesh_3::C3T3_helpers<C3T3, MD> helper(c3t3, domain);
        helper.update_restricted_facets();

        if (c3t3.number_of_facets() == 0) {
          need_more_init = true;
        }
      }
      if(need_more_init) {
        init_c3t3(c3t3, domain, criteria, nb_initial_points);
      }
    }
    else { init_c3t3(c3t3,domain,criteria,nb_initial_points); }
  }
};
  
// Partial specialization of C3t3_initializer
// Handles cases where MeshDomain::Has_features is a valid type and is defined
// to CGAL::Tag_false
template < typename C3T3, typename MD, typename MC >
struct C3t3_initializer < C3T3, MD, MC, true, CGAL::Tag_false >
{
  void operator()(C3T3& c3t3,
                  const MD& domain,
                  const MC& criteria,
                  bool with_features,
                  bool /* nonlinear */ = false,
                  const int nb_initial_points = -1)
  {
    if ( with_features )
    {
      std::cerr << "Warning: you requested a mesh with features from a domain"
                << " without features !" << std::endl;
    }
    
    init_c3t3(c3t3,domain,criteria,nb_initial_points);
  }
};

}} // end namespace internal::Mesh_3
  

// -----------------------------------
// make_mesh_3 stuff
// -----------------------------------

// Manual redirections
// boost::parameter can't handle make_mesh_3 return_type alone...
#ifndef CGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES

template <typename C3T3, typename MD, typename MC, typename ... T>
C3T3 make_mesh_3(const MD& md, const MC& mc, const T& ...t)
{
  C3T3 c3t3;
  make_mesh_3_bp(c3t3,md,mc,t...);
  return c3t3;  
}

#else

template <typename C3T3, typename MD, typename MC>
C3T3 make_mesh_3(const MD& md, const MC& mc)
{ 
  C3T3 c3t3;
  make_mesh_3_bp(c3t3,md,mc);
  return c3t3;
}
  
template <typename C3T3, typename MD, typename MC,
  typename Arg1>
C3T3 make_mesh_3(const MD& md, const MC& mc, const Arg1& a1)
{ 
  C3T3 c3t3;
  make_mesh_3_bp(c3t3,md,mc,a1);
  return c3t3;
}

template <typename C3T3, typename MD, typename MC,
  typename Arg1, typename Arg2>
C3T3 make_mesh_3(const MD& md, const MC& mc, const Arg1& a1, const Arg2& a2)
{
  C3T3 c3t3; 
  make_mesh_3_bp(c3t3,md,mc,a1,a2);
  return c3t3;
}

template <typename C3T3, typename MD, typename MC,
  typename Arg1, typename Arg2, typename Arg3>
C3T3 make_mesh_3(const MD& md, const MC& mc, const Arg1& a1, const Arg2& a2,
                 const Arg3& a3)
{ 
  C3T3 c3t3;
  make_mesh_3_bp(c3t3,md,mc,a1,a2,a3);
  return c3t3;
}

template <typename C3T3, typename MD, typename MC,
  typename Arg1, typename Arg2, typename Arg3, typename Arg4>
C3T3 make_mesh_3(const MD& md, const MC& mc, const Arg1& a1, const Arg2& a2,
                 const Arg3& a3, const Arg4& a4)
{ 
  C3T3 c3t3;
  make_mesh_3_bp(c3t3,md,mc,a1,a2,a3,a4);
  return c3t3;
}

template <typename C3T3, typename MD, typename MC,
  typename Arg1, typename Arg2, typename Arg3, typename Arg4, typename Arg5>
C3T3 make_mesh_3(const MD& md, const MC& mc, const Arg1& a1, const Arg2& a2,
                 const Arg3& a3, const Arg4& a4, const Arg5& a5)
{ 
  C3T3 c3t3;
  make_mesh_3_bp(c3t3,md,mc,a1,a2,a3,a4,a5);
  return c3t3;
}

#endif  
 
#if defined(BOOST_MSVC)
#  pragma warning(push)
#  pragma warning(disable:4003) // not enough actual parameters for macro
#endif
  
// see <CGAL/config.h>
CGAL_PRAGMA_DIAG_PUSH
// see <CGAL/Mesh_3/config.h>
CGAL_MESH_3_IGNORE_BOOST_PARAMETER_NAME_WARNINGS

BOOST_PARAMETER_FUNCTION(
  (void),
  make_mesh_3_bp,
  parameters::tag,
  (required (in_out(c3t3),*) (domain,*) (criteria,*) ) // nondeduced
  (deduced 
    (optional
      (features_param, (parameters::internal::Features_options), parameters::features(domain))
      (exude_param, (parameters::internal::Exude_options), parameters::exude())
      (perturb_param, (parameters::internal::Perturb_options), parameters::perturb())
      (odt_param, (parameters::internal::Odt_options), parameters::no_odt())
      (lloyd_param, (parameters::internal::Lloyd_options), parameters::no_lloyd())
      (mesh_options_param, (parameters::internal::Mesh_3_options), 
                           parameters::internal::Mesh_3_options())
      (manifold_options_param, (parameters::internal::Manifold_options),
                               parameters::internal::Manifold_options())
    )
  )
)
{
  make_mesh_3_impl(c3t3, domain, criteria,
                   exude_param, perturb_param, odt_param, lloyd_param,
                   features_param.features(), mesh_options_param,
                   manifold_options_param);
}
CGAL_PRAGMA_DIAG_POP

#if defined(BOOST_MSVC)
#  pragma warning(pop)
#endif


/**
 * @brief This function meshes the domain defined by mesh_traits
 * (respecting criteria), and outputs the mesh to c3t3
 *
 * @param domain the domain to be discretized
 * @param criteria the criteria
 * @param exude if it is set to \c true, an exudation step will be done at
 *   the end of the Delaunay refinement process
 *
 * @return The mesh as a C3T3 object
 */
template<class C3T3, class MeshDomain, class MeshCriteria>
void make_mesh_3_impl(C3T3& c3t3,
                      const MeshDomain&   domain,
                      const MeshCriteria& criteria,
                      const parameters::internal::Exude_options& exude,
                      const parameters::internal::Perturb_options& perturb,
                      const parameters::internal::Odt_options& odt,
                      const parameters::internal::Lloyd_options& lloyd,
                      const bool with_features,
                      const parameters::internal::Mesh_3_options& 
                        mesh_options = parameters::internal::Mesh_3_options(),
                      const parameters::internal::Manifold_options&
                        manifold_options = parameters::internal::Manifold_options())
{
#ifdef CGAL_MESH_3_INITIAL_POINTS_NO_RANDOM_SHOOTING
  CGAL::get_default_random() = CGAL::Random(0);
#endif

  // Initialize c3t3
  internal::Mesh_3::C3t3_initializer< 
    C3T3,
    MeshDomain,
    MeshCriteria,
    internal::Mesh_3::has_Has_features<MeshDomain>::value > () (c3t3,
            domain,
            criteria,
            with_features,
            mesh_options.nonlinear_growth_of_balls,
            mesh_options.number_of_initial_points);

  CGAL_assertion( c3t3.triangulation().dimension() == 3 );

  // Build mesher and launch refinement process
  // Don't reset c3t3 as we just created it
  refine_mesh_3(c3t3, domain, criteria,
                exude, perturb, odt, lloyd, parameters::no_reset_c3t3(), mesh_options,
                manifold_options);
}


}  // end namespace CGAL

#include <CGAL/enable_warnings.h>

#endif // CGAL_MAKE_MESH_3_H
