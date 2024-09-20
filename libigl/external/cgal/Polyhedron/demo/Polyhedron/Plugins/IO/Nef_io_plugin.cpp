#include "Scene_nef_polyhedron_item.h"

#include <CGAL/Three/Polyhedron_demo_io_plugin_interface.h>
#include <fstream>
#include <limits>

using namespace CGAL::Three;
class Polyhedron_demo_io_nef_plugin :
  public QObject,
  public Polyhedron_demo_io_plugin_interface
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_io_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")

public:
  QString nameFilters() const;
  QString name() const { return "io_nef_plugin"; }
  bool canLoad() const;
  CGAL::Three::Scene_item* load(QFileInfo fileinfo);

  bool canSave(const CGAL::Three::Scene_item*);
  bool save(const CGAL::Three::Scene_item*, QFileInfo fileinfo);
};

QString Polyhedron_demo_io_nef_plugin::nameFilters() const {
  return "nef files (*.nef3)";
}

bool Polyhedron_demo_io_nef_plugin::canLoad() const {
  return true;
}


CGAL::Three::Scene_item*
Polyhedron_demo_io_nef_plugin::load(QFileInfo fileinfo) {
  //do not try file with extension different from nef3
  if (fileinfo.suffix() != "nef3") return 0;
  
  // Open file
  std::ifstream in(fileinfo.filePath().toUtf8());
  if(!in) {
    std::cerr << "Error! Cannot open file " << (const char*)fileinfo.filePath().toUtf8() << std::endl;
    return NULL;
  }
    
  // Try to read .nef3 in a polyhedron
  Scene_nef_polyhedron_item* item = new Scene_nef_polyhedron_item();
  item->setName(fileinfo.baseName());
  if(!item->load(in))
  {
    delete item;
    return 0;
  }

  return item;
}

bool Polyhedron_demo_io_nef_plugin::canSave(const CGAL::Three::Scene_item* item)
{
  // This plugin supports polyhedrons and polygon soups
  return qobject_cast<const Scene_nef_polyhedron_item*>(item);
}

bool Polyhedron_demo_io_nef_plugin::save(const CGAL::Three::Scene_item* item, QFileInfo fileinfo)
{
  // This plugin supports polyhedrons and polygon soups
  const Scene_nef_polyhedron_item* nef_item = 
    qobject_cast<const Scene_nef_polyhedron_item*>(item);

  if(!nef_item)
    return false;

  std::ofstream out(fileinfo.filePath().toUtf8());
  out.precision (std::numeric_limits<double>::digits10 + 2);
  return (nef_item && nef_item->save(out));
}

#include "Nef_io_plugin.moc"
