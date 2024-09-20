#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/Surface_mesh/IO.h>

#include <boost/unordered_map.hpp>
#include <boost/property_map/property_map.hpp>

#include <iostream>
#include <fstream>
#include <set>
#include <CGAL/boost/graph/selection.h>
#include <CGAL/boost/graph/helpers.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> SM;
typedef boost::graph_traits<SM>::face_descriptor face_descriptor;

int main()
{
  SM sm;
  std::ifstream input("data/head.off");
  input >> sm;

// define my selection of faces to remove
  boost::unordered_map<face_descriptor, bool> is_selected_map;

  const int selection_indices[] = {501, 652, 646, 322, 328, 212, 347, 345, 352, 353, 696, 697, 698, 706, 714, 2892};
  std::set<int> index_set(&selection_indices[0], &selection_indices[0]+16);

  std::vector<face_descriptor> faces_to_remove;
  int index = 0;
  BOOST_FOREACH(face_descriptor fh, faces(sm))
  {
    if(index_set.count(index)==0)
      is_selected_map[fh]=false;
    else
    {
      faces_to_remove.push_back(fh);
      is_selected_map[fh]=true;
    }
    ++index;
  }

  expand_face_selection_for_removal(faces_to_remove,
                                    sm,
                                    boost::make_assoc_property_map(is_selected_map));

  index=0;
  BOOST_FOREACH(face_descriptor fh, faces(sm))
  {
    if (is_selected_map[fh])
    {
      CGAL::Euler::remove_face(halfedge(fh, sm), sm);
      ++index;
    }
  }

  assert(index == 25);
  assert(is_valid(sm));
  return 0;
}

