#if defined (_MSC_VER) && !defined (_WIN64)
#pragma warning(disable:4244) // boost::number_distance::distance()
                              // converts 64 to 32 bits integers
#endif

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Classification.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Random.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

typedef CGAL::Point_set_3<Point> Point_set;
typedef Point_set::Point_map Point_map;

typedef Kernel::Iso_cuboid_3 Iso_cuboid_3;

namespace Classification = CGAL::Classification;

typedef Classification::Label_handle                                            Label_handle;
typedef Classification::Feature_handle                                          Feature_handle;
typedef Classification::Label_set                                               Label_set;
typedef Classification::Feature_set                                             Feature_set;

typedef Classification::Sum_of_weighted_features_classifier                     Classifier;

typedef Classification::Point_set_feature_generator<Kernel, Point_set, Point_map> Feature_generator;

typedef Point_set::Property_map<std::size_t> Size_t_map;
typedef Point_set::Property_map<Classification::RGB_Color> Color_map;



int main (int, char**)
{
  Point_set pts;

  pts.add_normal_map();
  
  bool map_added = false;
  Size_t_map echo_map;
  Color_map color_map;

  boost::tie (echo_map, map_added) = pts.add_property_map<std::size_t> ("echo");
  CGAL_assertion (map_added);
  boost::tie (color_map, map_added) = pts.add_property_map<Classification::RGB_Color> ("color");
  CGAL_assertion (map_added);

  for (std::size_t i = 0; i < 1000; ++ i)
  {
    Point_set::iterator it
      = pts.insert (Point (CGAL::get_default_random().get_double(),
                           CGAL::get_default_random().get_double(),
                           CGAL::get_default_random().get_double()),
                    Vector (CGAL::get_default_random().get_double(),
                            CGAL::get_default_random().get_double(),
                            CGAL::get_default_random().get_double()));
    echo_map[*it] = std::size_t(CGAL::get_default_random().get_int(0, 4));
    color_map[*it] = CGAL::make_array ((unsigned char)(CGAL::get_default_random().get_int(0, 255)),
                                       (unsigned char)(CGAL::get_default_random().get_int(0, 255)),
                                       (unsigned char)(CGAL::get_default_random().get_int(0, 255)));
  }

  Feature_set features;
  Feature_generator generator (features, pts, pts.point_map(),
                               5,  // using 5 scales
                               pts.normal_map(),
                               color_map, echo_map);

  CGAL_assertion (generator.number_of_scales() == 5);
  CGAL_assertion (features.size() == 80);

  Label_set labels;

  std::vector<int> training_set (pts.size(), -1);
  for (std::size_t i = 0; i < 20; ++ i)
  {
    std::ostringstream oss;
    oss << "label_" << i;
    Label_handle lh = labels.add(oss.str().c_str());

    for (std::size_t j = 0; j < 10; ++ j)
      training_set[std::size_t(CGAL::get_default_random().get_int(0, int(training_set.size())))] = int(i);
  }
  CGAL_assertion (labels.size() == 20);
  
  Classifier classifier (labels, features);
  
  classifier.train<CGAL::Sequential_tag> (training_set, 800);
#ifdef CGAL_LINKED_WITH_TBB
  classifier.train<CGAL::Parallel_tag> (training_set, 800);
#endif

  std::vector<int> label_indices(pts.size(), -1);
  
  Classification::classify<CGAL::Sequential_tag>
    (pts, labels, classifier, label_indices);

  Classification::classify_with_local_smoothing<CGAL::Sequential_tag>
    (pts, pts.point_map(), labels, classifier,
     generator.neighborhood().sphere_neighbor_query(0.01f),
     label_indices);

  Classification::classify_with_graphcut<CGAL::Sequential_tag>
    (pts, pts.point_map(), labels, classifier,
     generator.neighborhood().k_neighbor_query(12),
     0.2f, 10, label_indices);

#ifdef CGAL_LINKED_WITH_TBB
  Classification::classify<CGAL::Sequential_tag>
    (pts, labels, classifier, label_indices);

  Classification::classify_with_local_smoothing<CGAL::Sequential_tag>
    (pts, pts.point_map(), labels, classifier,
     generator.neighborhood().sphere_neighbor_query(0.01f),
     label_indices);

  Classification::classify_with_graphcut<CGAL::Sequential_tag>
    (pts, pts.point_map(), labels, classifier,
     generator.neighborhood().k_neighbor_query(12),
     0.2f, 10, label_indices);
#endif

  Classification::Evaluation evaluation (labels, training_set, label_indices);
  
  return EXIT_SUCCESS;
}
