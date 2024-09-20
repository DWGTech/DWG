#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "config.h"
#include "MainWindow_config.h"

#include <QtOpenGL/qgl.h>
#include <CGAL/Qt/DemosMainWindow.h>

#include <QScriptEngine>
#include <QScriptable>


#include <QVector>
#include <QList>
#include <QFileInfo>
#include <QStringList>
#include <QSet>
#include <QModelIndex>
class Scene;
class Viewer;
class QTreeView;
class QMenu;
namespace CGAL {
namespace Three{
class Polyhedron_demo_io_plugin_interface;
class Polyhedron_demo_plugin_interface;
class Scene_item;
}
}

class QSortFilterProxyModel;

namespace Ui {
  class MainWindow;
  class Statistics_on_item_dialog;
}

#include "Polyhedron_type_fwd.h"

#include "Messages_interface.h"

/*!
 * \brief The main window of the applicatipon.
 *
 * It contains all the widgets, the menus and the elements of interface
 * of the application.*/

class MAINWINDOW_EXPORT MainWindow : 
  public CGAL::Qt::DemosMainWindow,
  public Messages_interface,
  protected QScriptable
{
  Q_OBJECT
  Q_INTERFACES(Messages_interface)
public:
  //! \brief Global state of the application.
  //!
  //! A plugin that outputs a `Facegraph_item` will match the input type for the output.
  //! But when the input is not a `Facegraph_item`, the plugin will use this state to know what type to use.
  enum Face_graph_mode{
    SURFACE_MESH=0,
    POLYHEDRON
  };
  /*! \brief The constructor
   *
   * It links the class with its UI file and sets it up.
   * It also saves pointers to viewer and the Geometric Objects view.
   * Then it creates and initializes the scene and do the
   * connexions with the UI. Finally it loads the plugins.*/

  MainWindow(bool verbose = false,QWidget* parent = 0);
  ~MainWindow();

  /*! Finds an IO plugin.
   * throws std::invalid_argument if no loader with that argument can be found
   @returns the IO plugin associated with `loader_name`*/
  CGAL::Three::Polyhedron_demo_io_plugin_interface* findLoader(const QString& loader_name) const;

  /*! \brief Loads an item with a given loader.
   *
   * throws `std::logic_error` if loading does not succeed or
   * `std::invalid_argument` if `fileinfo` specifies an invalid file*/
  CGAL::Three::Scene_item* loadItem(QFileInfo fileinfo, CGAL::Three::Polyhedron_demo_io_plugin_interface*);

Q_SIGNALS:
  //! Is emitted when the Application is closed.
  void on_closure();
  //! Is emitted when a group is expanded.
  void expanded(QModelIndex);
  //! Is emitted when a group is collapsed.
  void collapsed(QModelIndex);


public Q_SLOTS:
  //!Creates a new group and adds it to the scene.
  void makeNewGroup();
  //! Re-computes the viewer's Bbox
  //! If `b` is true, recenters the scene.
  void updateViewerBBox(bool b);
  //! Opens a script or a file with the default loader if there is.
  void open(QString);
  //! Is called when the up button is pressed.
  void on_upButton_pressed();
  //! Is called when the down button is pressed.
  void on_downButton_pressed();
  //! COllapses the groups that were before the Geometric Objects view was re-drawn.
  void restoreCollapseState();
  //! Expands a group
  void setExpanded(QModelIndex);
  //! Collapses a group.
  void setCollapsed(QModelIndex);
  //! checks if a file matches `filters`
  bool file_matches_filter(const QString& filters, const QString& filename);
  void reset_default_loaders();
  //!Prints a dialog containing statistics on the selected polyhedrons.
  void statisticsOnItem();
  /*! Open a file with a given loader, and return true if it was successful.
   This slot is for use by scripts.*/
  bool open(QString filename, QString loader_name);

  /*! Reloads an item. Expects to be called by a QAction with the
   index of the item to be reloaded as data attached to the action.
   The index must identify a valid `Scene_item`.*/
  void reloadItem();
  
  //! Loads a script. Returns true if it worked.
  bool loadScript(QString filename);

  //! Loads a script. Returns true if it worked.
  bool loadScript(QFileInfo);

  /*!
   * Gives the keyboard input focus to the widget searchEdit.
   */
  void setFocusToQuickSearch();

  /*!
   * Clears the current selection and selects the scene_item with
   * the index i in the Geometric Objects view.
   */
  void selectSceneItem(int i);
  /*!
   * Prints coordinates of a point and its distance to the last
   * position printed by this function.
   */
  void showSelectedPoint(double, double, double);
  /*!
   * Removes an item from the current selection.
   */
  void unSelectSceneItem(int i);
  /*!
   * Clears the current selection and select all the scene_items
   * in the Geometric Objects view.
   */
  void selectAll();
  /*!
   * Assigns a different color to each selected item.
   */
  void colorItems();

  /*!
   * Adds the scene_item with the index i in the Geometric Objects view to the
   * current selection. Calls itemChanged(i) from the scene.
   */
  void addSceneItemInSelection(int i);

  /*!
   * Removes the scene_item with the index i in the Geometric Objects view to the
   * current selection.
   */
  void removeSceneItemFromSelection(int i);

  /*!
   * Sets the modifiers of the viewer.
   */
  void setAddKeyFrameKeyboardModifiers(Qt::KeyboardModifiers);
  /*!
   * Set the visibility of all actions from the targeted menu
   * and its submenus to false and disables them.
   */
  void clearMenu(QMenu*);
  /*!
   * Enables and sets the targeted action's visibility to true.
   * Does the same in all the menus of the widgets associated with
   * this action.
   */
  void addAction(QAction*);
  /*!
   * Creates an action with text actionText named actionName and
   * adds it to the menu menuName. If menuName does not exist,
   * it is created.
   */
  void addAction(QString actionName,
                 QString actionText,
                 QString menuName);
  /*!
   * Sets the scene center to the target position and makes the
   * scene slide to this new center. Also sets the pivotPoint of
   * the camera to this position.
   */
  void viewerShow(float, float, float);
  /*!
   * Sets the scene center to be the center of the target BBox.
   * Also sets the pivotPoint of the camera to this position.
   */
  void viewerShow(float, float, float, float, float, float);
  /*!
   * Centers the scene on the target object.
   */
  void viewerShowObject();
  /*!
   * Displays a text preceded by the mention "INFO :".
   */
  void information(QString);
  /*!
   * Displays a blue text preceded by the mention "WARNING :".
   */
  void warning(QString);
  /*!
   * Displays a red text preceded by the mention "ERROR :".
   */
  void error(QString);

    //!Displays a text in the chosen html color with the chosen html font.

  void message(QString, QString, QString = QString("normal"));

    //!Returns true if the target plugin is present. If not, returns false.
  bool hasPlugin(const QString&) const;
  /*!
   * If able, finds a script debugger and interrupts current action. Default
   * value for parameter is true.
   */
  void enableScriptDebugger(bool = true);

  /// This slot is used to test exception handling in Qt Scripts.
  void throw_exception();

  /*!
   * set_face_graph_default_type sets the global state of the application to `Polyhedron mode` or `Surface_mesh mode`.
   */
  void set_face_graph_default_type(MainWindow::Face_graph_mode m);

  /*!
   * Writes the statistics dialog content in a text file.
   */
  void exportStatistics();
protected Q_SLOTS:

   //!Gets the new selected item(s) from the Geometric Objects view and updates the scene
   //!and viewer accordingly.
  /*!
   * Set the scene selected item or selected item list. Sets the manipulated
   * frame of the viewer to the new selected item's and calls update().
   */
  void selectionChanged();
  //! Scrolls to the new selected item from its name in the Geometric Objects list.
  void recenterSceneView(const QModelIndex &id);

  /*!
   * Invoques a context menu for the currently selected item at the requested
   * position.
   */
  void contextMenuRequested(const QPoint& global_pos);
  /*!
   * Invoques a context menu for the requested item at the requested
   * position.
   */
  void showSceneContextMenu(int selectedItemIndex,
                            const QPoint& global_pos);

  //!This is an overloaded function. Invoques a context menu at the requested
  //!position.
  /*!
   * If the widget which received the request is not the Geometric Objects view, the index
   * chosen by default for the menu is the one of the currently selected item.
   * If it is the Geometric Objects view, then the index of the clicked item is collected.
   * If this index is valid, then it is used for the menu. If not, the function
   * returns.
   */
  void showSceneContextMenu(const QPoint& local_pos_of_treeview);

  //!Prints information about the currently selected item if able.
  void updateInfo();
  //!Prints graphical information about the currently selected item if able.
  void updateDisplayInfo();
  //!Sets the current manipulated frame to 0.
  void removeManipulatedFrame(CGAL::Three::Scene_item*);

  // settings
  //!Closes the main window.
  void quit();
  //!Reads the plugin_blacklist contents and apply them.
  void readSettings();
  //!Sets up the plugin_blacklist.
  void writeSettings();

  // load, erase, duplicate
  //!Loops on on_actionErase_triggered();
  void on_actionEraseAll_triggered();
  //!Opens a dialog to open one or several files.
  void on_actionLoad_triggered();
  //!Erases the selected items. Returns true if items remain in the Geometric Objects view.
  bool on_actionErase_triggered();
  //!Duplicates the selected item and selects the new item.
  void on_actionDuplicate_triggered();
  //!If QT_SCRIPT_LIB is defined, opens a dialog to choose a script.
  void on_actionLoadScript_triggered();
  //!Loads a plugin from a specified directory
  void on_actionLoadPlugin_triggered();
  // Show/Hide
  //!Swap the visibility of the selected item(s).
  void on_actionShowHide_triggered();
  //!Pops a dialog to change the max TextItems
  void on_actionMaxTextItemsDisplayed_triggered();
  // Select A/B
  //!Sets the selected item as item_A.
  void on_actionSetPolyhedronA_triggered();
  //!Sets the selected item as Item_B.
  void on_actionSetPolyhedronB_triggered();

  //Preferences edition
  //!Opens the Preferences dialog.
  void on_actionPreferences_triggered();
  // save as...
  //!Opens a dialog to save selected item if able.
  void on_actionSaveAs_triggered(); 
  //!Calls the function save of the current plugin if able.
  void save(QString filename, CGAL::Three::Scene_item* item);
  //!Calls the function saveSnapShot of the viewer.
  void on_actionSaveSnapshot_triggered();
  //!Opens a Dialog to choose a color and make it the background color.
  void on_actionSetBackgroundColor_triggered();
  /*! Opens a Dialog to enter coordinates of the new center point and sets it
   * with viewerShow.
   *@see viewerShow(float, float, float, float, float, float)
   */
  void on_actionLookAt_triggered();
  //!Returns the position and orientation of the current camera frame.
  QString cameraString() const;
  /*! Prints the position and orientation of the current camera frame.
   * @see cameraString()
   */
  void on_actionDumpCamera_triggered();
  //!Sets the coordinates of the camera in the clipboard text.
  void on_actionCopyCamera_triggered();
  //!Gets coordinates from the clipboard and sets them as the current ones for
  //!the camera.
  void on_actionPasteCamera_triggered();
  //!Hides not available operations and show available operations in all the
  //!menus.
  void filterOperations();
  //!Updates the bounding box and moves the camera to fits the scene.
  void on_actionRecenterScene_triggered();
  //!Resizes the header of the scene view
  void resetHeader();
  //!apply an action named `name` to all selected items
  void propagate_action();
protected:
  QList<QAction*> createSubMenus(QList<QAction*>);
  /*! For each objects in the Geometric Objects view, loads the associated plugins.
   * Gets the property "submenuName" of all the actions and creates submenus.
   * Sorts the Operations menu by name.
   * @see initPlugin(QObject*);
   * @see initIOPlugin(QObject*);
   */
  void loadPlugins();
  /*!
   * \brief Initializes the plugins.
   * Makes pairs between plugins and object names and fills the Operations menu.
   * Called only once.
   */
  bool initPlugin(QObject*);
  //!Initializes the IO plugin for the target object.
  bool initIOPlugin(QObject*);
  /*!
   * Calls writeSettings() and set the flag accepted for the event.
   * @see writeSettings()
   */
  void closeEvent(QCloseEvent *event);
  /*! Returns the currently selected item in the Geometric Objects view. Returns -1
   * if none is selected.
   */
  int getSelectedSceneItemIndex() const;
  //! Returns a list of the selected items in the Geometric Objects view.
  QList<int> getSelectedSceneItemIndices() const;
private:
  void updateMenus();
  bool load_plugin(QString names, bool blacklisted);
  void recurseExpand(QModelIndex index);
  QMap<QString, QMenu*> menu_map;
  QString get_item_stats();
  QString strippedName(const QString &fullFileName);
  void setMenus(QString, QString, QAction *a);
  /// plugin black-list
  QSet<QString> plugin_blacklist;
  QMap<QString, std::vector<QString> > PathNames_map; //For each non-empty plugin directory, contains a vector of plugin names
  QMap<QString, QString > pluginsStatus_map; //For each non-empty plugin directory, contains a vector of plugin names
  Scene* scene;
  Viewer* viewer;
  QSortFilterProxyModel* proxyModel;
  QTreeView* sceneView;
  Ui::MainWindow* ui;
  QVector<CGAL::Three::Polyhedron_demo_io_plugin_interface*> io_plugins;
  QString last_saved_dir;
  QMap<QString,QString> default_plugin_selection;
  // typedef to make Q_FOREACH work
  typedef QPair<CGAL::Three::Polyhedron_demo_plugin_interface*, QString> PluginNamePair;
  QVector<PluginNamePair > plugins;
  //!Called when "Add new group" in the file menu is triggered.
  QAction* actionAddToGroup;
  QAction* actionResetDefaultLoaders;
  void print_message(QString message) { messages->information(message); }
  Messages_interface* messages;

  QDialog *statistics_dlg;
  Ui::Statistics_on_item_dialog* statistics_ui;
  bool verbose;
  void insertActionBeforeLoadPlugin(QMenu*, QAction *actionToInsert);

#ifdef QT_SCRIPT_LIB
  QScriptEngine* script_engine;
public:
  /*! Evaluates a script and search for uncaught exceptions. If quiet is false, prints the
   *backtrace of the uncaught exceptions.
   */
  void evaluate_script(QString script, 
                       const QString & fileName = QString(),
                       const bool quiet = false);
  //! Calls evaluate_script(script, filename, true).
  void evaluate_script_quiet(QString script, 
                             const QString & fileName = QString());
#endif
private Q_SLOTS:
  void set_facegraph_mode_adapter(bool is_polyhedron);
};

#endif // ifndef MAINWINDOW_H
