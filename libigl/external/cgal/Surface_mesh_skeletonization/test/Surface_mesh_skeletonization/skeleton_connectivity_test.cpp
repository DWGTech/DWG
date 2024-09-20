#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/internal/corefinement/connected_components.h>
#include <CGAL/IO/Polyhedron_iostream.h>

#include <boost/foreach.hpp>

#include <fstream>
#include <set>
#include <queue>

typedef CGAL::Simple_cartesian<double>                               Kernel;
typedef Kernel::Point_3                                              Point;
typedef Kernel::Vector_3                                             Vector;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Polyhedron;
typedef boost::graph_traits<Polyhedron>::vertex_descriptor           vertex_descriptor;

typedef CGAL::Mean_curvature_flow_skeletonization<Polyhedron>        Mean_curvature_skeleton;
typedef Mean_curvature_skeleton::Skeleton                            Skeleton;
typedef boost::graph_traits<Skeleton>::vertex_descriptor             vertex_desc;
typedef boost::graph_traits<Skeleton>::edge_descriptor             edge_desc;

// The input of the skeletonization algorithm must be a pure triangular closed
// mesh and has only one component.
bool is_mesh_valid(Polyhedron& pMesh)
{
  if (!pMesh.is_closed())
  {
    std::cerr << "The mesh is not closed.";
    return false;
  }
  if (!pMesh.is_pure_triangle())
  {
    std::cerr << "The mesh is not a pure triangle mesh.";
    return false;
  }

  // the algorithm is only applicable on a mesh
  // that has only one connected component
  std::size_t num_component;
  CGAL::Counting_output_iterator output_it(&num_component);
  CGAL::internal::corefinement::extract_connected_components(pMesh, output_it);
  ++output_it;
  if (num_component != 1)
  {
    std::cerr << "The mesh is not a single closed mesh. It has "
              << num_component << " components.";
    return false;
  }
  return true;
}

int main()
{
  Polyhedron mesh;
  std::ifstream input("data/elephant.off");

  if ( !input || !(input >> mesh) || mesh.empty() ) {
    std::cerr << "Cannot open data/elephant.off" << std::endl;
    return EXIT_FAILURE;
  }
  if (!is_mesh_valid(mesh)) {
    return EXIT_FAILURE;
  }

  Skeleton skeleton;

  CGAL::extract_mean_curvature_flow_skeleton(mesh, skeleton);

  if (num_vertices(skeleton) == 0)
  {
    std::cerr << "The number of skeletal points is zero!\n";
    return EXIT_FAILURE;
  }

// check all vertices are seen exactly once
{
  std::set<vertex_descriptor> visited;
  BOOST_FOREACH(vertex_desc v, vertices(skeleton))
  {
    BOOST_FOREACH(vertex_descriptor vd, skeleton[v].vertices)
      if (!visited.insert(vd).second)
      {
        std::cerr << "A vertex was seen twice!\n";
        return EXIT_FAILURE;
      }
  }

  BOOST_FOREACH(vertex_descriptor vd, vertices(mesh))
  {
    if (!visited.count(vd))
    {
      std::cerr << "A vertex was not seen!\n";
      return EXIT_FAILURE;
    }
  }
}

// check the skeleton is connected
{
  std::queue<vertex_desc> qu;
  std::set<vertex_desc> visited;

  qu.push(*vertices(skeleton).first);
  visited.insert(qu.back());

  while (!qu.empty())
  {
    vertex_desc cur = qu.front();
    qu.pop();

    BOOST_FOREACH(edge_desc ed, in_edges(cur, skeleton))
    {
      vertex_desc next = source(ed, skeleton);
      if (visited.insert(next).second)
        qu.push(next);
    }
  }

  BOOST_FOREACH(vertex_desc vd, vertices(skeleton))
  {
    if (!visited.count(vd))
    {
      std::cerr << "Skeleton curve is not fully connected!\n";
      return EXIT_FAILURE;
    }
  }
}
  std::cout << "Pass connectivity test.\n";
  return EXIT_SUCCESS;
}

