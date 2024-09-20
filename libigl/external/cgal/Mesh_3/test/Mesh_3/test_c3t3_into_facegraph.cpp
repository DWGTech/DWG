#include <cassert>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include <CGAL/Mesh_3/tet_soup_to_c3t3.h>


#include <CGAL/Mesh_3/Robust_intersection_traits_3.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include <CGAL/Mesh_domain_with_polyline_features_3.h>
#include <CGAL/IO/facets_in_complex_3_to_triangle_mesh.h>

#include <CGAL/tags.h>

#include <sstream>

int main (int argc, char** argv){
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Polyhedral_mesh_domain_with_features_3<K> Polyhedral_mesh_domain;

  //Robust_cc_geom_traits
  typedef CGAL::Kernel_traits<Polyhedral_mesh_domain>::Kernel
      Robust_intersections_traits;
  typedef CGAL::details::Mesh_geom_traits_generator<Robust_intersections_traits>::type
      Robust_K;

  // Triangulation
  typedef CGAL::Compact_mesh_cell_base_3<Robust_K, Polyhedral_mesh_domain>    Cell_base;
  typedef CGAL::Triangulation_cell_base_with_info_3<int, Robust_K, Cell_base> Cell_base_with_info;
  typedef CGAL::Mesh_triangulation_3<Polyhedral_mesh_domain,
      Robust_intersections_traits,
      CGAL::Sequential_tag,
      CGAL::Default,
      Cell_base_with_info>::type Tr;

  typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3;


  // Open file
  std::ifstream in (argc > 1 ? argv[1] : "data/elephant.mesh",
                    std::ios_base::in);
  if(!in) {
    std::cerr << "Error! Cannot open file "
              << "/home/gimeno/Bureau/test.mesh" << std::endl;
    return 1;
  }
  C3t3 c3t3;
  if(CGAL::build_triangulation_from_file<C3t3::Triangulation, true>(in, c3t3.triangulation()))
  {
    for( C3t3::Triangulation::Finite_cells_iterator
         cit = c3t3.triangulation().finite_cells_begin();
         cit != c3t3.triangulation().finite_cells_end();
         ++cit)
    {
      CGAL_assertion(cit->info() >= 0);
      c3t3.add_to_complex(cit, cit->info());
      for(int i=0; i < 4; ++i)
      {
        if(cit->surface_patch_index(i)>0)
        {
          c3t3.add_to_complex(cit, i, cit->surface_patch_index(i));
        }
      }
    }
    //if there is no facet in the complex, we add the border facets.
    if(c3t3.number_of_facets_in_complex() == 0)
    {
      for( C3t3::Triangulation::All_cells_iterator
           cit = c3t3.triangulation().all_cells_begin();
           cit != c3t3.triangulation().all_cells_end();
           ++cit)
      {
        if(c3t3.triangulation().is_infinite(cit))
        {
          for(int i=0; i<4; ++i)
          {
            if(!c3t3.triangulation().is_infinite(cit, i))
              c3t3.add_to_complex(cit, i, 1);
          }
        }
      }
    }
  }

  CGAL::Polyhedron_3<K> poly;
  CGAL::facets_in_complex_3_to_triangle_mesh(c3t3, poly);

  CGAL_assertion(is_valid(poly));
  return EXIT_SUCCESS;
}
