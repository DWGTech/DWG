#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/repair.h>

#include <fstream>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;
typedef CGAL::Polyhedron_3<K> Polyhedron_3;

template <class TriangleMesh>
void test_middle_edge()
{
  typedef boost::graph_traits<TriangleMesh> GT;
  
  std::ifstream in("data_repair/edge_middle.off");
  TriangleMesh tm;
  in >> tm;
  typename GT::halfedge_descriptor h=GT::null_halfedge();
  Point src(0.75, 1, 0);
  Point tgt(1.5, 1, 0);
  BOOST_FOREACH(h, halfedges(tm))
  {
    if( get(CGAL::vertex_point, tm, source(h,tm)) == src &&
        get(CGAL::vertex_point, tm, target(h,tm)) == tgt )
      break;
  }
  CGAL_assertion( h!=GT::null_halfedge() );
  CGAL::Polygon_mesh_processing::remove_a_border_edge(edge(h,tm), tm);
  
  CGAL_assertion(is_valid(tm));
  CGAL_assertion(is_triangle_mesh(tm));
  std::ofstream out("edge_middle_out.off");
  out << tm;
}

template <class TriangleMesh>
void test_edge_border_case1()
{
  typedef boost::graph_traits<TriangleMesh> GT;
  
  std::ifstream in("data_repair/edge_border_case1.off");
  TriangleMesh tm;
  in >> tm;
  typename GT::halfedge_descriptor h=GT::null_halfedge();
  Point src(0, 0, 0);
  Point tgt(1.5, 0, 0);
  BOOST_FOREACH(h, halfedges(tm))
  {
    if( get(CGAL::vertex_point, tm, source(h,tm)) == src &&
        get(CGAL::vertex_point, tm, target(h,tm)) == tgt )
      break;
  }
  CGAL_assertion( h!=GT::null_halfedge() );
  CGAL::Polygon_mesh_processing::remove_a_border_edge(edge(h,tm), tm);
  
  CGAL_assertion(is_valid(tm));
  CGAL_assertion(is_triangle_mesh(tm));
  std::ofstream out("edge_border_case1_out.off");
  out << tm;
}

template <class TriangleMesh>
void test_edge_border_case2()
{
  typedef boost::graph_traits<TriangleMesh> GT;
  
  std::ifstream in("data_repair/edge_border_case2.off");
  TriangleMesh tm;
  in >> tm;
  typename GT::halfedge_descriptor h=GT::null_halfedge();
  Point src(3, 1, 0);
  Point tgt(3, 0, 0);
  BOOST_FOREACH(h, halfedges(tm))
  {
    if( get(CGAL::vertex_point, tm, source(h,tm)) == src &&
        get(CGAL::vertex_point, tm, target(h,tm)) == tgt )
      break;
  }
  CGAL_assertion( h!=GT::null_halfedge() );
  CGAL::Polygon_mesh_processing::remove_a_border_edge(edge(h,tm), tm);
  
  CGAL_assertion(is_valid(tm));
  CGAL_assertion(is_triangle_mesh(tm));
  std::ofstream out("edge_border_case2_out.off");
  out << tm;
}

int main()
{
  test_middle_edge<Surface_mesh>();  
  test_edge_border_case1<Surface_mesh>();
  test_edge_border_case2<Surface_mesh>();
  test_middle_edge<Polyhedron_3>();  
  test_edge_border_case1<Polyhedron_3>();
  test_edge_border_case2<Polyhedron_3>();
}
