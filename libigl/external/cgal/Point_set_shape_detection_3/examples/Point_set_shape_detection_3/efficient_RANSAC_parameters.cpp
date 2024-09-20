#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>

#include <CGAL/Shape_detection_3.h>

#include <iostream>
#include <fstream>


// Type declarations
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::FT                                           FT;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;

// In Shape_detection_traits the basic types, i.e., Point and Vector types
// as well as iterator type and property maps, are defined.
typedef CGAL::Shape_detection_3::Shape_detection_traits<Kernel,
  Pwn_vector, Point_map, Normal_map>            Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection_3::Cone<Traits>             Cone;
typedef CGAL::Shape_detection_3::Cylinder<Traits>         Cylinder;
typedef CGAL::Shape_detection_3::Plane<Traits>            Plane;
typedef CGAL::Shape_detection_3::Sphere<Traits>           Sphere;
typedef CGAL::Shape_detection_3::Torus<Traits>            Torus;


int main() 
{
  // Points with normals.
  Pwn_vector points;

  // Loads point set from a file. 
  // read_xyz_points_and_normals takes an OutputIterator for storing the points
  // and a property map to store the normal vector with each point.
  std::ifstream stream("data/cube.pwn");

  if (!stream ||
    !CGAL::read_xyz_points(stream,
      std::back_inserter(points),
      CGAL::parameters::point_map(Point_map()).
      normal_map(Normal_map())))
  {
    std::cerr << "Error: cannot read file cube.pwn" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << points.size() << " points" << std::endl;

  // Instantiates shape detection engine.
  Efficient_ransac ransac;

  // Provides the input data.
  ransac.set_input(points);
    
  // Register shapes for detection
  ransac.add_shape_factory<Plane>();
  ransac.add_shape_factory<Sphere>();
  ransac.add_shape_factory<Cylinder>();
  ransac.add_shape_factory<Cone>();
  ransac.add_shape_factory<Torus>();

  // Sets parameters for shape detection.
  Efficient_ransac::Parameters parameters;

  // Sets probability to miss the largest primitive at each iteration.
  parameters.probability = 0.05;
 
  // Detect shapes with at least 500 points.
  parameters.min_points = 200;

  // Sets maximum Euclidean distance between a point and a shape.
  parameters.epsilon = 0.002;
 
  // Sets maximum Euclidean distance between points to be clustered.
  parameters.cluster_epsilon = 0.01;
 
  // Sets maximum normal deviation.
  // 0.9 < dot(surface_normal, point_normal); 
  parameters.normal_threshold = 0.9;   
  
  // Detects shapes
  ransac.detect(parameters);

  // Prints number of detected shapes and unassigned points.
   std::cout << ransac.shapes().end() - ransac.shapes().begin() << " detected shapes, "
     << ransac.number_of_unassigned_points()
     << " unassigned points." << std::endl;
  
  // Efficient_ransac::shapes() provides
  // an iterator range to the detected shapes.
  Efficient_ransac::Shape_range shapes = ransac.shapes();
  Efficient_ransac::Shape_range::iterator it = shapes.begin();

  while (it != shapes.end()) {
    
    // Get specific parameters depending on detected shape.
    if (Plane* plane = dynamic_cast<Plane*>(it->get()))
      {
        Kernel::Vector_3 normal = plane->plane_normal();
        std::cout << "Plane with normal " << normal
                << std::endl;
        
        // Plane shape can also be converted to Kernel::Plane_3
        std::cout << "Kernel::Plane_3: " << static_cast<Kernel::Plane_3>(*plane) << std::endl;
      }
    else if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get()))
      {
        Kernel::Line_3 axis = cyl->axis();
        FT radius = cyl->radius();
        std::cout << "Cylinder with axis " << axis
                  << " and radius " << radius
                  << std::endl;
      }
    else
      {
        // Prints the parameters of the detected shape.
        // This function is available for any type of shape.
        std::cout << (*it)->info() << std::endl;
      }
    
    // Proceeds with next detected shape.
    it++;
  }

  return EXIT_SUCCESS;
}
