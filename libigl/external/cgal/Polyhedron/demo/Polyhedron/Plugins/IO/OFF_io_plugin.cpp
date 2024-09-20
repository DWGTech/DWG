#include "Scene_polyhedron_item.h"
#include "Scene_polygon_soup_item.h"
#include "Scene_points_with_normal_item.h"
#include "Polyhedron_type.h"

#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

#include <CGAL/Three/Polyhedron_demo_io_plugin_interface.h>
#include <fstream>

#include <CGAL/IO/File_scanner_OFF.h>
#include <CGAL/IO/OBJ_reader.h>
#include <QMessageBox>
#include <QApplication>

using namespace CGAL::Three;
class Polyhedron_demo_off_plugin :
  public QObject,
  public Polyhedron_demo_io_plugin_interface
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_io_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.IOPluginInterface/1.0")

public:
  bool isDefaultLoader(const Scene_item *item) const 
  { 
    if(qobject_cast<const Scene_polyhedron_item*>(item)
       || qobject_cast<const Scene_polygon_soup_item*>(item)) 
      return true; 
    return false;
  }
  QString name() const { return "off_plugin"; }
  QString nameFilters() const { return "OFF files (*.off);;Wavefront OBJ (*.obj)"; }
  bool canLoad() const;
  CGAL::Three::Scene_item* load(QFileInfo fileinfo);
   CGAL::Three::Scene_item* load_off(QFileInfo fileinfo);
   CGAL::Three::Scene_item* load_obj(QFileInfo fileinfo);
  
  bool canSave(const CGAL::Three::Scene_item*);
  bool save(const CGAL::Three::Scene_item*, QFileInfo fileinfo);
};

bool Polyhedron_demo_off_plugin::canLoad() const {
  return true;
}


CGAL::Three::Scene_item*
Polyhedron_demo_off_plugin::load(QFileInfo fileinfo) {
  if(fileinfo.suffix().toLower() == "off"){
    return load_off(fileinfo);
  } else if(fileinfo.suffix().toLower() == "obj"){
    return load_obj(fileinfo);
  }
  return 0;
}


CGAL::Three::Scene_item*
Polyhedron_demo_off_plugin::load_off(QFileInfo fileinfo) {
  // Open file
  std::ifstream in(fileinfo.filePath().toUtf8());
  if(!in) {
    std::cerr << "Error! Cannot open file " << (const char*)fileinfo.filePath().toUtf8() << std::endl;
    return NULL;
  }


  CGAL::File_scanner_OFF scanner( in, false);

  // Try to read .off in a point set
  if (scanner.size_of_facets() == 0)
    {
      in.seekg(0);
      Scene_points_with_normal_item* item = new Scene_points_with_normal_item();
      item->setName(fileinfo.completeBaseName());
      if (scanner.size_of_vertices()==0) return item;
      if(!item->read_off_point_set(in))
        {
          delete item;
          return 0;
        }

      return item;
    }
  
  // to detect isolated vertices
  std::size_t total_nb_of_vertices = scanner.size_of_vertices();
  in.seekg(0);

  // Try to read .off in a polyhedron
  Scene_polyhedron_item* item = new Scene_polyhedron_item();
  item->setName(fileinfo.completeBaseName());
  if(!item->load(in))
  {
    delete item;

    // Try to read .off in a polygon soup
    Scene_polygon_soup_item* soup_item = new Scene_polygon_soup_item;
    soup_item->setName(fileinfo.completeBaseName());
    in.close();
    std::ifstream in2(fileinfo.filePath().toUtf8());
    if(!soup_item->load(in2)) {
      delete soup_item;
      return 0;
    }
    return soup_item;
  }
  else
    if( total_nb_of_vertices!= item->polyhedron()->size_of_vertices())
    {
      item->setNbIsolatedvertices(total_nb_of_vertices - item->polyhedron()->size_of_vertices());
      //needs two restore, it's not a typo
      QApplication::restoreOverrideCursor();
      QMessageBox::warning((QWidget*)NULL,
                     tr("Isolated vertices found"),
                     tr("%1 isolated vertices ignored")
                     .arg(item->getNbIsolatedvertices()));
    }

  //if file > 100 MB, assume it is a very big file and disable flat shading to gain memory.
  if(fileinfo.size() > 100000000)//100 MB
  {
    item->set_flat_disabled(true);
    QApplication::restoreOverrideCursor();
    QMessageBox::warning((QWidget*)NULL,
                   tr("The file seems to be very big."),
                   tr("Flat shading has been disabled to gain memory. You can force it in the context menu of the item."));
  }
  return item;
}

CGAL::Three::Scene_item*
Polyhedron_demo_off_plugin::load_obj(QFileInfo fileinfo) {
  // Open file
  std::ifstream in(fileinfo.filePath().toUtf8());
  if(!in) {
    std::cerr << "Error! Cannot open file " << (const char*)fileinfo.filePath().toUtf8() << std::endl;
    return NULL;
  }
  typedef Polyhedron::Vertex::Point Point;
  std::vector<Point> points;
  std::vector<std::vector<std::size_t> > faces;
  if(!CGAL::read_OBJ(in,points,faces)) return 0;

  Scene_item* item = 0;

  namespace pmp = CGAL::Polygon_mesh_processing;
  if(pmp::is_polygon_soup_a_polygon_mesh(faces)) {
    Scene_polyhedron_item* poly_item = new Scene_polyhedron_item();
    pmp::polygon_soup_to_polygon_mesh(std::move(points),
                                      std::move(faces),
                                      *(poly_item->polyhedron()));
    item = poly_item;
    item->invalidateOpenGLBuffers();
  } else {
    Scene_polygon_soup_item* polygon_soup_item = new Scene_polygon_soup_item();
    polygon_soup_item->load(std::move(points), std::move(faces));
    item = polygon_soup_item;
  }
  // Try to read .obj in a polyhedron
  item->setName(fileinfo.completeBaseName());
  return item;
}

bool Polyhedron_demo_off_plugin::canSave(const CGAL::Three::Scene_item* item)
{
  // This plugin supports polyhedrons and polygon soups
  return qobject_cast<const Scene_polyhedron_item*>(item) ||
    qobject_cast<const Scene_polygon_soup_item*>(item) ||
    qobject_cast<const Scene_points_with_normal_item*>(item);
}

bool Polyhedron_demo_off_plugin::save(const CGAL::Three::Scene_item* item, QFileInfo fileinfo)
{
  // This plugin supports point sets, polyhedrons and polygon soups
  const Scene_points_with_normal_item* points_item =
    qobject_cast<const Scene_points_with_normal_item*>(item);
  const Scene_polyhedron_item* poly_item = 
    qobject_cast<const Scene_polyhedron_item*>(item);
  const Scene_polygon_soup_item* soup_item = 
    qobject_cast<const Scene_polygon_soup_item*>(item);

  if(!poly_item && !soup_item && !points_item)
    return false;

  std::ofstream out(fileinfo.filePath().toUtf8());
  out.precision (std::numeric_limits<double>::digits10 + 2);

  if(fileinfo.suffix().toLower() == "off"){
    return (poly_item && poly_item->save(out)) || 
      (soup_item && soup_item->save(out)) ||
      (points_item && points_item->write_off_point_set(out));
  }
  if(fileinfo.suffix().toLower() == "obj"){
    return (poly_item && poly_item->save_obj(out));
  }
  return false;
}

#include "OFF_io_plugin.moc"
