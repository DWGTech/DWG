#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_id_2.h>
#include <CGAL/boost/graph/graph_traits_Delaunay_triangulation_2.h>

#include <CGAL/boost/graph/dijkstra_shortest_paths.h>
#include <boost/graph/filtered_graph.hpp>
#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;

typedef CGAL::Triangulation_vertex_base_with_id_2<K> Tvb;
typedef CGAL::Triangulation_face_base_2<K> Tfb;
typedef CGAL::Triangulation_data_structure_2<Tvb,Tfb> Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds> Triangulation;

// consider finite vertices and edges.

template <typename T>
struct Is_finite {

  const T* t_;

  Is_finite()
    : t_(NULL)
  {}

  Is_finite(const T& t)
    : t_(&t)
  { }

  template <typename VertexOrEdge>
  bool operator()(const VertexOrEdge& voe) const {
    return ! t_->is_infinite(voe);
  }
};

typedef Is_finite<Triangulation> Filter;
typedef boost::filtered_graph<Triangulation,Filter,Filter> Finite_triangulation;
typedef boost::graph_traits<Finite_triangulation>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Finite_triangulation>::vertex_iterator vertex_iterator;


int
main(int argc,char* argv[])
{
  const char* filename = (argc > 1) ? argv[1] : "data/points.xy";
  std::ifstream input(filename);
  Triangulation t;
  Filter is_finite(t);
  Finite_triangulation ft(t, is_finite, is_finite);

  Point p ;
  while(input >> p){
    t.insert(p);
  }

  vertex_iterator vit, ve;
  // associate indices to the vertices
  int index = 0;
  for(boost::tie(vit,ve)=boost::vertices(ft); vit!=ve; ++vit ){
    vertex_descriptor  vd = *vit;
    vd->id()= index++;
  }

  typedef boost::property_map<Triangulation, boost::vertex_index_t>::type VertexIdPropertyMap;
  VertexIdPropertyMap vertex_index_pmap = get(boost::vertex_index, ft);

  // Dijkstra's shortest path needs property maps for the predecessor and distance
  std::vector<vertex_descriptor> predecessor(boost::num_vertices(ft));
  boost::iterator_property_map<std::vector<vertex_descriptor>::iterator, VertexIdPropertyMap>
    predecessor_pmap(predecessor.begin(), vertex_index_pmap);

  std::vector<double> distance(boost::num_vertices(ft));
  boost::iterator_property_map<std::vector<double>::iterator, VertexIdPropertyMap>
    distance_pmap(distance.begin(), vertex_index_pmap);

  vertex_descriptor source =  *boost::vertices(ft).first;
  std::cout << "\nStart dijkstra_shortest_paths at " << source->point() << std::endl;

  boost::dijkstra_shortest_paths(ft, source ,
				 distance_map(distance_pmap)
				 .predecessor_map(predecessor_pmap));

  for(boost::tie(vit,ve)=boost::vertices(ft); vit!=ve; ++vit ){
    vertex_descriptor vd = *vit;
    std::cout << vd->point() << " [" << vd->id() << "] ";
    std::cout << " has distance = "  << get(distance_pmap,vd) << " and predecessor ";
    vd = get(predecessor_pmap,vd);
    std::cout << vd->point() << " [" << vd->id() << "]\n";
  }
  return 0;
}
