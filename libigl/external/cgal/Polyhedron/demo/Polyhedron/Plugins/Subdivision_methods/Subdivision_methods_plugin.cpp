#include <QTime>
#include <QApplication>
#include <QMainWindow>
#include <QAction>
#include <QInputDialog>

#include <CGAL/Three/Polyhedron_demo_plugin_helper.h>
#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>

#include "Messages_interface.h"
#include "Scene_polyhedron_item.h"
#include "Scene_surface_mesh_item.h"
#include "Polyhedron_type.h"
#include "SMesh_type.h"
#include <CGAL/subdivision_method_3.h>
using namespace CGAL::Three;
class Polyhedron_demo_subdivision_methods_plugin :
  public QObject,
  public Polyhedron_demo_plugin_helper
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")
public:
  // used by Polyhedron_demo_plugin_helper
  QList<QAction*> actions() const {
    return _actions;
  }

  void init(QMainWindow* mw,
            Scene_interface* scene_interface,
            Messages_interface* m)
  {
      this->mw = mw;
      messages = m;
      scene = scene_interface;
      QAction *actionLoop = new QAction("Loop", mw);
      QAction *actionCatmullClark = new QAction("Catmull Clark", mw);
      QAction *actionSqrt3 = new QAction("Sqrt3", mw);
      QAction *actionDooSabin = new QAction("Doo Sabin", mw);
      actionLoop->setObjectName("actionLoop");
      actionCatmullClark->setObjectName("actionCatmullClark");
      actionSqrt3->setObjectName("actionSqrt3");
      actionDooSabin->setObjectName("actionDooSabin");
      _actions << actionLoop
               << actionCatmullClark
               << actionSqrt3
               << actionDooSabin;
      Q_FOREACH(QAction* action, _actions)
        action->setProperty("subMenuName", "3D Surface Subdivision Methods");
      autoConnectActions();

  }

  bool applicable(QAction*) const {
    return qobject_cast<Scene_polyhedron_item*>(scene->item(scene->mainSelectionIndex()))
        || qobject_cast<Scene_surface_mesh_item*>(scene->item(scene->mainSelectionIndex()));
  }

public Q_SLOTS:
  void on_actionLoop_triggered();
  void on_actionCatmullClark_triggered();
  void on_actionSqrt3_triggered();
  void on_actionDooSabin_triggered();
private :
  Messages_interface* messages;
  QList<QAction*> _actions;
  template<class FaceGraphItem>
  void apply_loop(FaceGraphItem* item, int nb_steps);
  template<class FaceGraphItem>
  void apply_catmullclark(FaceGraphItem* item, int nb_steps);
  template<class FaceGraphItem>
  void apply_sqrt3(FaceGraphItem* item, int nb_steps);
  template<class FaceGraphItem>
  void apply_doosabin(FaceGraphItem* item, int nb_steps);
}; // end Polyhedron_demo_subdivision_methods_plugin


template<class FaceGraphItem>
void Polyhedron_demo_subdivision_methods_plugin::apply_loop(FaceGraphItem* item, int nb_steps)
{
  typename FaceGraphItem::Face_graph* graph = item->face_graph();
  QTime time;
  time.start();
  messages->information("Loop subdivision...");
  QApplication::setOverrideCursor(Qt::WaitCursor);
  CGAL::Subdivision_method_3::Loop_subdivision(*graph, nb_steps);
  messages->information(QString("ok (%1 ms)").arg(time.elapsed()));
  QApplication::restoreOverrideCursor();
  item->invalidateOpenGLBuffers();
  scene->itemChanged(item);
}

void Polyhedron_demo_subdivision_methods_plugin::on_actionLoop_triggered()
{
  CGAL::Three::Scene_interface::Item_id index = scene->mainSelectionIndex();

  Scene_polyhedron_item* item =
      qobject_cast<Scene_polyhedron_item*>(scene->item(index));

  if(!item)
  {
    Scene_surface_mesh_item* sm_item =
        qobject_cast<Scene_surface_mesh_item*>(scene->item(index));
    if(!sm_item)
      return;
    int nb_steps = QInputDialog::getInt(mw,
                                        QString("Number of Iterations"),
                                        QString("Choose number of iterations"),
                                        1,
                                        1);
    apply_loop(sm_item, nb_steps);
  }
  else
  {
    int nb_steps = QInputDialog::getInt(mw,
                                        QString("Number of Iterations"),
                                        QString("Choose number of iterations"),
                                        1,
                                        1);
    apply_loop(item, nb_steps);
  }
}

template<class FaceGraphItem>
void Polyhedron_demo_subdivision_methods_plugin::apply_catmullclark(FaceGraphItem* item, int nb_steps)
{
  typename FaceGraphItem::Face_graph* graph = item->face_graph();
  if(!graph) return;
  QTime time;
  time.start();
  messages->information("Catmull-Clark subdivision...");
  QApplication::setOverrideCursor(Qt::WaitCursor);
  CGAL::Subdivision_method_3::CatmullClark_subdivision(*graph, nb_steps);
  messages->information(QString("ok (%1 ms)").arg(time.elapsed()));
  QApplication::restoreOverrideCursor();
  item->invalidateOpenGLBuffers();
  scene->itemChanged(item);
}
void Polyhedron_demo_subdivision_methods_plugin::on_actionCatmullClark_triggered()
{
  CGAL::Three::Scene_interface::Item_id index = scene->mainSelectionIndex();

  Scene_polyhedron_item* item =
      qobject_cast<Scene_polyhedron_item*>(scene->item(index));

  if(!item)
  {
    Scene_surface_mesh_item* sm_item =
        qobject_cast<Scene_surface_mesh_item*>(scene->item(index));
    if(!sm_item)
      return;
    int nb_steps = QInputDialog::getInt(mw,
                                        QString("Number of Iterations"),
                                        QString("Choose number of iterations"),
                                        1,
                                        1);
    apply_catmullclark(sm_item, nb_steps);
  }
  else{
    int nb_steps = QInputDialog::getInt(mw,
                                        QString("Number of Iterations"),
                                        QString("Choose number of iterations"),
                                        1,
                                        1);
    apply_catmullclark(item, nb_steps);
  }

}

template<class FaceGraphItem>
void Polyhedron_demo_subdivision_methods_plugin::apply_sqrt3(FaceGraphItem* item, int nb_steps)
{
  typename FaceGraphItem::Face_graph* graph = item->face_graph();
  if(!graph) return;
  QTime time;
  time.start();
  messages->information("Sqrt-3 subdivision...");
  QApplication::setOverrideCursor(Qt::WaitCursor);
  CGAL::Subdivision_method_3::Sqrt3_subdivision(*graph, nb_steps);
  messages->information(QString("ok (%1 ms)").arg(time.elapsed()));
  QApplication::restoreOverrideCursor();
  item->invalidateOpenGLBuffers();
  scene->itemChanged(item);
}

void Polyhedron_demo_subdivision_methods_plugin::on_actionSqrt3_triggered()
{
  CGAL::Three::Scene_interface::Item_id index = scene->mainSelectionIndex();

  Scene_polyhedron_item* item =
      qobject_cast<Scene_polyhedron_item*>(scene->item(index));

  if(!item)
  {
    Scene_surface_mesh_item* sm_item =
        qobject_cast<Scene_surface_mesh_item*>(scene->item(index));
    if(!sm_item)
      return;
    int nb_steps = QInputDialog::getInt(mw,
                                        QString("Number of Iterations"),
                                        QString("Choose number of iterations"),
                                        1,
                                        1);
    apply_sqrt3(sm_item, nb_steps);
  }
  else{
    int nb_steps = QInputDialog::getInt(mw,
                                        QString("Number of Iterations"),
                                        QString("Choose number of iterations"),
                                        1,
                                        1);
    apply_sqrt3(item, nb_steps);
  }

}

template<class FaceGraphItem>
void Polyhedron_demo_subdivision_methods_plugin::apply_doosabin(FaceGraphItem* item, int nb_steps)
{
  typename FaceGraphItem::Face_graph* graph = item->face_graph();
  if(!graph) return;
  QTime time;
  time.start();
  messages->information("Doo-Sabin subdivision...");
  QApplication::setOverrideCursor(Qt::WaitCursor);
  CGAL::Subdivision_method_3::DooSabin_subdivision(*graph, nb_steps);
  messages->information(QString("ok (%1 ms)").arg(time.elapsed()));
  QApplication::restoreOverrideCursor();
  item->invalidateOpenGLBuffers();
  scene->itemChanged(item);
}

void Polyhedron_demo_subdivision_methods_plugin::on_actionDooSabin_triggered()
{
  CGAL::Three::Scene_interface::Item_id index = scene->mainSelectionIndex();

  Scene_polyhedron_item* item =
      qobject_cast<Scene_polyhedron_item*>(scene->item(index));

  if(!item)
  {
    Scene_surface_mesh_item* sm_item =
        qobject_cast<Scene_surface_mesh_item*>(scene->item(index));
    if(!sm_item)
      return;
    int nb_steps = QInputDialog::getInt(mw,
                                        QString("Number of Iterations"),
                                        QString("Choose number of iterations"),
                                        1,
                                        1);
    apply_doosabin(sm_item, nb_steps);
  }
  else{
    int nb_steps = QInputDialog::getInt(mw,
                                        QString("Number of Iterations"),
                                        QString("Choose number of iterations"),
                                        1,
                                        1);
    apply_doosabin(item, nb_steps);
  }
}

#include "Subdivision_methods_plugin.moc"
