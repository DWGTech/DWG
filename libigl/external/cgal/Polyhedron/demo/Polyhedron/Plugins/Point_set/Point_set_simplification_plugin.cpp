#include "config.h"
#include "Scene_points_with_normal_item.h"
#include <CGAL/Three/Polyhedron_demo_plugin_helper.h>
#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>

#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/random_simplify_point_set.h>
#include <CGAL/hierarchy_simplify_point_set.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/Timer.h>
#include <CGAL/Memory_sizer.h>

#include <QObject>
#include <QAction>
#include <QMainWindow>
#include <QApplication>
#include <QtPlugin>
#include <QMessageBox>

#include "ui_Point_set_simplification_plugin.h"

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif


using namespace CGAL::Three;
class Polyhedron_demo_point_set_simplification_plugin :
  public QObject,
  public Polyhedron_demo_plugin_helper
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")

  QAction* actionSimplify;

public:
  void init(QMainWindow* mainWindow, CGAL::Three::Scene_interface* scene_interface,Messages_interface*) {
    scene = scene_interface;
    actionSimplify = new QAction(tr("Simplification Selection"), mainWindow);
    actionSimplify->setProperty("subMenuName","Point Set Processing");

    actionSimplify->setObjectName("actionSimplify");
    autoConnectActions();
  }

  bool applicable(QAction*) const {
    return qobject_cast<Scene_points_with_normal_item*>(scene->item(scene->mainSelectionIndex()));
  }

  QList<QAction*> actions() const {
    return QList<QAction*>() << actionSimplify;
  }

public Q_SLOTS:
  void on_actionSimplify_triggered();


}; // end Polyhedron_demo_point_set_simplification_plugin

class Point_set_demo_point_set_simplification_dialog : public QDialog, private Ui::PointSetSimplificationDialog
{
  Q_OBJECT
  public:
    Point_set_demo_point_set_simplification_dialog(QWidget * /*parent*/ = 0)
    {
      setupUi(this);
    }

  unsigned int simplificationMethod() const
  {
    if (Random->isChecked())
      return 0;
    else if (Grid->isChecked())
      return 1;
    else
      return 2;
  }
  double randomSimplificationPercentage() const { return m_randomSimplificationPercentage->value(); }
  double gridCellSize() const { return m_gridCellSize->value(); }
  unsigned int maximumClusterSize() const { return m_maximumClusterSize->value(); }
  double maximumSurfaceVariation() const { return m_maximumSurfaceVariation->value(); }

public Q_SLOTS:
  
  void on_Random_toggled (bool toggled)
  {
    m_randomSimplificationPercentage->setEnabled (toggled);
    m_gridCellSize->setEnabled (!toggled);
    m_maximumClusterSize->setEnabled (!toggled);
    m_maximumSurfaceVariation->setEnabled (!toggled);
  }
  void on_Grid_toggled (bool toggled)
  {
    m_randomSimplificationPercentage->setEnabled (!toggled);
    m_gridCellSize->setEnabled (toggled);
    m_maximumClusterSize->setEnabled (!toggled);
    m_maximumSurfaceVariation->setEnabled (!toggled);
  }
  void on_Hierarchy_toggled (bool toggled)
  {
    m_randomSimplificationPercentage->setEnabled (!toggled);
    m_gridCellSize->setEnabled (!toggled);
    m_maximumClusterSize->setEnabled (toggled);
    m_maximumSurfaceVariation->setEnabled (toggled);
  }

};

void Polyhedron_demo_point_set_simplification_plugin::on_actionSimplify_triggered()
{
  const CGAL::Three::Scene_interface::Item_id index = scene->mainSelectionIndex();

  Scene_points_with_normal_item* item =
    qobject_cast<Scene_points_with_normal_item*>(scene->item(index));

  if(item)
  {
    // Gets point set
    Point_set* points = item->point_set();
    if(points == NULL)
        return;

    // Gets options
    Point_set_demo_point_set_simplification_dialog dialog;
    if(!dialog.exec())
      return;

    QApplication::setOverrideCursor(Qt::WaitCursor);

    CGAL::Timer task_timer; task_timer.start();

    // First point to delete
    Point_set::iterator first_point_to_remove = points->end();

    unsigned int method = dialog.simplificationMethod ();
    if (method == 0)
    {
      std::cerr << "Point set random simplification (" << dialog.randomSimplificationPercentage() <<"%)...\n";

      // Computes points to remove by random simplification
      first_point_to_remove =
        CGAL::random_simplify_point_set(*points,
                                        dialog.randomSimplificationPercentage());
    }
    else if (method == 1)
    {
      std::cerr << "Point set grid simplification (cell size = " << dialog.gridCellSize() <<" * average spacing)...\n";

      // Computes average spacing
      double average_spacing = CGAL::compute_average_spacing<Concurrency_tag>(*points, 6 /* knn = 1 ring */);

      // Computes points to remove by Grid Clustering
      first_point_to_remove =
        CGAL::grid_simplify_point_set(*points,
                                      dialog.gridCellSize()*average_spacing);
    }
    else
    {
      std::cerr << "Point set hierarchy simplification (cluster size = " << dialog.maximumClusterSize()
		<< ", maximum variation = " << dialog.maximumSurfaceVariation() << ")...\n";

      // Computes points to remove by Grid Clustering
      first_point_to_remove =
        CGAL::hierarchy_simplify_point_set(*points,
                                           points->parameters().
                                           size(dialog.maximumClusterSize()).
                                           maximum_variation(dialog.maximumSurfaceVariation()));
    }

    std::size_t nb_points_to_remove = std::distance(first_point_to_remove, points->end());
    std::size_t memory = CGAL::Memory_sizer().virtual_size();
    std::cerr << "Simplification: " << nb_points_to_remove << " point(s) are selected for removal ("
                                    << task_timer.time() << " seconds, "
                                    << (memory>>20) << " Mb allocated)"
                                    << std::endl;

    // Selects points to delete
    points->set_first_selected(first_point_to_remove);

    // Updates scene
    item->invalidateOpenGLBuffers();
    scene->itemChanged(index);

    QApplication::restoreOverrideCursor();

    // Warns user
    if (nb_points_to_remove > 0)
    {
      QMessageBox::information(NULL,
                               tr("Points selected for removal"),
                               tr("%1 point(s) are selected for removal.\nYou may delete or reset the selection using the item context menu.")
                               .arg(nb_points_to_remove));
    }
  }
}

#include "Point_set_simplification_plugin.moc"
