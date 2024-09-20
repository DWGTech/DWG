#ifndef ID_PRINTING_H
#define ID_PRINTING_H
#include <CGAL/boost/graph/selection.h>
#include <CGAL/Three/Viewer_interface.h>
#include <CGAL/Three/TextRenderer.h>
#include <CGAL/Kernel_traits.h>
#include <boost/foreach.hpp>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <vector>

template<class Mesh>
struct VKRingPMAP{
  typedef typename boost::graph_traits<Mesh>::vertex_descriptor key_type;
  typedef bool                                                  value_type;
  typedef value_type                                            reference;
  typedef boost::read_write_property_map_tag                    category;
  typedef typename boost::property_map<Mesh, boost::vertex_index_t>::type IDmap;
  std::vector<bool>* vec;
  Mesh* poly;
  IDmap idmap;


  VKRingPMAP(std::vector<bool>* vec, Mesh* poly)
    :vec(vec), poly(poly)
  {
    idmap = get(boost::vertex_index, *poly);
  }

  friend value_type get(const VKRingPMAP<Mesh>& map, const key_type& f){
    return (*map.vec)[get(map.idmap, f)];
  }
  friend void put(VKRingPMAP<Mesh>& map, const key_type& f, const value_type i){
    (*map.vec)[get(map.idmap, f)] = i;
  }
};
template<class Mesh>
struct EdgeKRingPMAP{
  typedef typename boost::graph_traits<Mesh>::edge_descriptor  key_type;
  typedef bool                                                 value_type;
  typedef value_type                                           reference;
  typedef boost::read_write_property_map_tag                   category;
  typedef typename boost::property_map<Mesh, boost::halfedge_index_t>::type IDmap;
  std::vector<bool>* vec;
  Mesh* poly;
  IDmap idmap;

  EdgeKRingPMAP(std::vector<bool>* vec, Mesh* poly)
    :vec(vec), poly(poly)
  {
    idmap = get(boost::halfedge_index, *poly);
  }

  friend value_type get(const EdgeKRingPMAP<Mesh>& map, const key_type& f){
    return (*map.vec)[get(map.idmap, halfedge(f, *map.poly))/2];
  }
  friend void put(EdgeKRingPMAP<Mesh>& map, const key_type& f, const value_type i){
    (*map.vec)[get(map.idmap, halfedge(f, *map.poly))/2] = i;
  }
};
template<class Mesh>
struct FKRingPMAP{
  typedef typename boost::graph_traits<Mesh>::face_descriptor key_type;
  typedef bool                                                value_type;
  typedef value_type                                          reference;
  typedef boost::read_write_property_map_tag                  category;
  typedef typename boost::property_map<Mesh, boost::face_index_t>::type IDmap;
  std::vector<bool>* vec;
  Mesh* poly;
  IDmap idmap;

  FKRingPMAP(std::vector<bool>* vec, Mesh* poly)
    :vec(vec), poly(poly)
  {
    idmap = get(boost::face_index, *poly);
  }

  friend value_type get(const FKRingPMAP<Mesh>& map, const key_type& f){
    return (*map.vec)[get(map.idmap, f)];
  }
  friend void put(FKRingPMAP<Mesh>& map, const key_type& f, const value_type i){
    (*map.vec)[get(map.idmap, f)] = i;
  }
};
void deleteIds(CGAL::Three::Viewer_interface* viewer,
               TextListItem* vitems,
               TextListItem* eitems,
               TextListItem* fitems,
               std::vector<TextItem*>* targeted_ids)
{
  TextRenderer *renderer = viewer->textRenderer();
  BOOST_FOREACH(TextItem* it, vitems->textList())
      delete it;
  BOOST_FOREACH(TextItem* it, eitems->textList())
      delete it;
  BOOST_FOREACH(TextItem* it, fitems->textList())
      delete it;
  vitems->clear();
  renderer->removeTextList(vitems);
  eitems->clear();
  renderer->removeTextList(eitems);
  fitems->clear();
  renderer->removeTextList(fitems);
  targeted_ids->clear();
  viewer->update();
}




template<typename Handle, typename Point, typename Tree>
bool find_primitive_id(const QPoint& point,
                       Tree* aabb_tree,
                       CGAL::Three::Viewer_interface *viewer,
                       Handle& selected_fh,
                       Point& pt_under)
{
  typedef typename CGAL::Kernel_traits<Point>::Kernel Traits;
  bool found = false;
  qglviewer::Vec point_under = viewer->camera()->pointUnderPixel(point,found);
  const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();

  //find clicked facet
  qglviewer::Vec dir = point_under - viewer->camera()->position();
  const Point ray_origin(viewer->camera()->position().x - offset.x,
                         viewer->camera()->position().y - offset.y,
                         viewer->camera()->position().z - offset.z);

  const typename Traits::Vector_3 ray_dir(dir.x, dir.y, dir.z);
  const typename Traits::Ray_3 ray(ray_origin, ray_dir);

  typedef typename Tree::template Intersection_and_primitive_id<typename Traits::Ray_3>::Type Intersection_and_primitive_id;
  typedef std::list<Intersection_and_primitive_id> Intersections;
  Intersections intersections;
  aabb_tree->all_intersections(ray, std::back_inserter(intersections));

  if(intersections.empty())
    return false;
  typename Intersections::iterator closest = intersections.begin();
  const Point* closest_point =
      boost::get<Point>(&closest->first);
  for(typename Intersections::iterator
      it = boost::next(intersections.begin()),
      end = intersections.end();
      it != end; ++it)
  {
    if(! closest_point) {
      closest = it;
    }
    else {
      const Point* it_point =
          boost::get<Point>(&it->first);
      if(it_point &&
         (ray_dir * (*it_point - *closest_point)) < 0)
      {
        closest = it;
        closest_point = it_point;
      }
    }
  }
  if(!closest_point)
    return false;
  pt_under = Point(point_under.x, point_under.y, point_under.z);
  selected_fh = closest->second;
  return true;
}

template<typename Mesh, typename Point >
void compute_displayed_ids(Mesh& mesh,
                           CGAL::Three::Viewer_interface *viewer,
                           const typename boost::graph_traits<Mesh>::face_descriptor& selected_fh,
                           const Point& pt_under,
                           const qglviewer::Vec& offset,
                           TextListItem* vitems,
                           TextListItem* eitems,
                           TextListItem* fitems,
                           std::vector<TextItem*>* targeted_ids)
{
  typedef typename boost::graph_traits<Mesh>::face_descriptor face_descriptor;
  typedef typename boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
  typedef typename boost::graph_traits<Mesh>::edge_descriptor edge_descriptor;
  typedef typename boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;

  typedef typename boost::property_map<Mesh, boost::vertex_point_t>::type Ppmap;
  Ppmap ppmap = get(boost::vertex_point, mesh);

  typedef typename boost::property_map<Mesh, boost::vertex_index_t>::type VIDmap;
  VIDmap vidmap = get(boost::vertex_index, mesh);

  typedef typename boost::property_map<Mesh, boost::halfedge_index_t>::type HIDmap;
  HIDmap hidmap = get(boost::halfedge_index, mesh);

  typedef typename boost::property_map<Mesh, boost::face_index_t>::type FIDmap;
  FIDmap fidmap = get(boost::face_index, mesh);

  QFont font;
  font.setBold(true);
  std::vector<vertex_descriptor> displayed_vertices;
  std::vector<edge_descriptor> displayed_edges;
  std::vector<face_descriptor> displayed_faces;
  //Test spots around facet to find the closest to point

  double min_dist = (std::numeric_limits<double>::max)();

  // test the vertices of the closest face
  BOOST_FOREACH(vertex_descriptor vh, vertices_around_face(halfedge(selected_fh, mesh), mesh))
  {
    Point test=Point(get(ppmap, vh).x()+offset.x,
                     get(ppmap, vh).y()+offset.y,
                     get(ppmap, vh).z()+offset.z);
    double dist = CGAL::squared_distance(test, pt_under);
    if( dist < min_dist){
      min_dist = dist;
      displayed_vertices.clear();
      displayed_vertices.push_back(vh);
    }
  }
  QVector3D point(
      get(ppmap, displayed_vertices[0]).x() + offset.x,
      get(ppmap, displayed_vertices[0]).y() + offset.y,
      get(ppmap, displayed_vertices[0]).z() + offset.z);

  //test if we want to erase or not
  BOOST_FOREACH(TextItem* text_item, *targeted_ids)
  {
    if(text_item->position() == point)
    {
      //hide and stop
      deleteIds(viewer, vitems, eitems, fitems, targeted_ids);
      return;
    }
  }
  deleteIds(viewer, vitems, eitems, fitems, targeted_ids);
  // test the midpoint of edges of the closest face
  BOOST_FOREACH(halfedge_descriptor e, halfedges_around_face(halfedge(selected_fh, mesh), mesh))
  {
    Point test=CGAL::midpoint(get(ppmap, source(e, mesh)),get(ppmap, target(e, mesh)));
    test = Point(test.x()+offset.x,
                 test.y()+offset.y,
                 test.z()+offset.z);
    double dist = CGAL::squared_distance(test, pt_under);
    if(dist < min_dist){
      min_dist = dist;
      displayed_vertices.clear();
      displayed_edges.clear();
      displayed_edges.push_back(edge(e, mesh));
    }
  }
  // test the centroid of the closest face
  double x(0), y(0), z(0);
  int total(0);
  BOOST_FOREACH(vertex_descriptor vh, vertices_around_face(halfedge(selected_fh, mesh), mesh))
  {
    x+=get(ppmap, vh).x();
    y+=get(ppmap, vh).y();
    z+=get(ppmap, vh).z();
    ++total;
  }

  Point test(x/total+offset.x,
             y/total+offset.y,
             z/total+offset.z);
  double dist = CGAL::squared_distance(test, pt_under);
  if(dist < min_dist){
    min_dist = dist;
    displayed_vertices.clear();
    displayed_edges.clear();
    displayed_faces.clear();
    if(selected_fh != boost::graph_traits<Mesh>::null_face())
      displayed_faces.push_back(selected_fh);
  }

  if(!displayed_vertices.empty())
  {
    BOOST_FOREACH(face_descriptor f, CGAL::faces_around_target(halfedge(displayed_vertices[0],mesh), mesh))
    {
      if(f != boost::graph_traits<Mesh>::null_face())
        displayed_faces.push_back(f);
    }
    BOOST_FOREACH(halfedge_descriptor h, CGAL::halfedges_around_target(halfedge(displayed_vertices[0], mesh), mesh))
    {
      displayed_edges.push_back(edge(h, mesh));
    }
  }
  else if(!displayed_edges.empty())
  {
    displayed_vertices.push_back(target(halfedge(displayed_edges[0], mesh), mesh));
    displayed_vertices.push_back(target(opposite(halfedge(displayed_edges[0], mesh), mesh),mesh));
    face_descriptor f1(face(halfedge(displayed_edges[0], mesh),mesh)),
        f2(face(opposite(halfedge(displayed_edges[0], mesh), mesh),mesh));
    if(f1 != boost::graph_traits<Mesh>::null_face())
      displayed_faces.push_back(f1);
    if(f2 != boost::graph_traits<Mesh>::null_face())
      displayed_faces.push_back(f2);
  }

  else if(!displayed_faces.empty())
  {
    BOOST_FOREACH(halfedge_descriptor h, CGAL::halfedges_around_face(halfedge(displayed_faces[0], mesh), mesh))
    {
      displayed_edges.push_back(edge(h, mesh));
      displayed_vertices.push_back(target(h, mesh));
    }
  }
  //fill TextItems
  std::vector<bool> vertex_selection(false);
  vertex_selection.resize(num_vertices(mesh));
  VKRingPMAP<Mesh> vpmap(&vertex_selection, &mesh);
  BOOST_FOREACH(vertex_descriptor v_h, displayed_vertices)
      put(vpmap, v_h, true);
  CGAL::expand_vertex_selection(displayed_vertices,
                                mesh,
                                1,
                                vpmap,
                                std::back_inserter(displayed_vertices));

  std::vector<bool> edge_selection(false);
  edge_selection.resize(num_edges(mesh));
  EdgeKRingPMAP<Mesh> epmap(&edge_selection, &mesh);
  BOOST_FOREACH(edge_descriptor e_d, displayed_edges)
      put(epmap, e_d, true);
  CGAL::expand_edge_selection(displayed_edges,
                              mesh,
                              1,
                              epmap,
                              std::back_inserter(displayed_edges));

  std::vector<bool> face_selection(false);
  face_selection.resize(num_faces(mesh));
  FKRingPMAP<Mesh> fpmap(&face_selection, &mesh);
  BOOST_FOREACH(face_descriptor f_h, displayed_faces)
  {
      put(fpmap, f_h, true);
  }
  CGAL::expand_face_selection(displayed_faces,
                              mesh,
                              1,
                              fpmap,
                              std::back_inserter(displayed_faces));

  BOOST_FOREACH(vertex_descriptor vh, displayed_vertices)
  {
    Point pos=Point(get(ppmap, vh).x()+offset.x,
                    get(ppmap, vh).y()+offset.y,
                    get(ppmap, vh).z()+offset.z);
    TextItem* text_item = new TextItem(pos.x(), pos.y(), pos.z(), QString("%1").arg(get(vidmap, vh)), true, font, Qt::red);
    vitems->append(text_item);
    targeted_ids->push_back(text_item);
  }
  BOOST_FOREACH(edge_descriptor e, displayed_edges)
  {
    halfedge_descriptor  h(halfedge(e, mesh));
    Point pos=CGAL::midpoint(get(ppmap, source(h, mesh)),get(ppmap, target(h, mesh)));
    pos = Point(pos.x()+offset.x,
                pos.y()+offset.y,
                pos.z()+offset.z);

    TextItem* text_item = new TextItem(pos.x(), pos.y(), pos.z(), QString("%1").arg(get(hidmap, h)/2), true, font, Qt::green);
    eitems->append(text_item);
  }

  BOOST_FOREACH(face_descriptor  f, displayed_faces)
  {
    double x(0), y(0), z(0);
    int total(0);
    BOOST_FOREACH(vertex_descriptor  vh, vertices_around_face(halfedge(f, mesh), mesh))
    {
      x+=get(ppmap, vh).x();
      y+=get(ppmap, vh).y();
      z+=get(ppmap, vh).z();
      ++total;
    }

    Point pos(x/total+offset.x,
              y/total+offset.y,
              z/total+offset.z);
    TextItem* text_item = new TextItem(pos.x(), pos.y(), pos.z(), QString("%1").arg(get(fidmap,f)), true, font, Qt::blue);
    fitems->append(text_item);
  }
}

template<class Mesh>
bool printVertexIds(const Mesh& mesh,
                    TextListItem* vitems,
                    CGAL::Three::Viewer_interface *viewer)
{
  typedef typename boost::property_map<Mesh, boost::vertex_point_t>::const_type Ppmap;
  typedef typename boost::property_traits<Ppmap>::value_type Point;
  typedef typename boost::property_map<Mesh, boost::vertex_index_t>::type IDmap;

  Ppmap ppmap = get(boost::vertex_point, mesh);
  IDmap idmap = get(boost::vertex_index, mesh);
  TextRenderer *renderer = viewer->textRenderer();
  const qglviewer::Vec offset = viewer->offset();
  QFont font;
  font.setBold(true);

  //fills textItems
  BOOST_FOREACH(typename boost::graph_traits<Mesh>::vertex_descriptor vh, vertices(mesh))
  {
    const Point& p = get(ppmap, vh);
    vitems->append(new TextItem((float)p.x() + offset.x,
                                (float)p.y() + offset.y,
                                (float)p.z() + offset.z,
                                QString("%1").arg(get(idmap, vh)), true, font, Qt::red));

  }
  //add the QList to the render's pool
  renderer->addTextList(vitems);
  if(vitems->size() > static_cast<std::size_t>(renderer->getMax_textItems()))
  {
    return false;
  }
  return true;
}

template<class Mesh>
bool printEdgeIds(const Mesh& mesh,
                  TextListItem* eitems,
                  CGAL::Three::Viewer_interface *viewer)
{
  typedef typename boost::property_map<Mesh, boost::vertex_point_t>::const_type Ppmap;
  typedef typename boost::property_traits<Ppmap>::value_type Point;
  typedef typename boost::property_map<Mesh, boost::halfedge_index_t>::type IDmap;

  Ppmap ppmap = get(boost::vertex_point, mesh);
  IDmap idmap = get(boost::halfedge_index, mesh);
  TextRenderer *renderer = viewer->textRenderer();
  const qglviewer::Vec offset = viewer->offset();
  QFont font;
  font.setBold(true);

  BOOST_FOREACH(typename boost::graph_traits<Mesh>::edge_descriptor e, edges(mesh))
  {
    const Point& p1 = get(ppmap, source(e, mesh));
    const Point& p2 = get(ppmap, target(e, mesh));
    eitems->append(new TextItem((float)(p1.x() + p2.x()) / 2 + offset.x,
                                (float)(p1.y() + p2.y()) / 2 + offset.y,
                                (float)(p1.z() + p2.z()) / 2 + offset.z,
                                QString("%1").arg(get(idmap, halfedge(e, mesh)) / 2), true, font, Qt::green));
  }
  //add the QList to the render's pool
  renderer->addTextList(eitems);
  if(eitems->size() > static_cast<std::size_t>(renderer->getMax_textItems()))
  {
    return false;
  }
  return true;
}

template<class Mesh>
bool printFaceIds(const Mesh& mesh,
                  TextListItem* fitems,
                  CGAL::Three::Viewer_interface *viewer)
{
  typedef typename boost::property_map<Mesh, boost::vertex_point_t>::const_type Ppmap;
  typedef typename boost::property_map<Mesh, boost::face_index_t>::type IDmap;

  Ppmap ppmap = get(boost::vertex_point, mesh);
  IDmap idmap = get(boost::face_index, mesh);
  TextRenderer *renderer = viewer->textRenderer();
  const qglviewer::Vec offset = viewer->offset();
  QFont font;
  font.setBold(true);
  BOOST_FOREACH(typename boost::graph_traits<Mesh>::face_descriptor fh, faces(mesh))
  {
    double x(0), y(0), z(0);
    int total(0);
    BOOST_FOREACH(typename boost::graph_traits<Mesh>::vertex_descriptor vh, vertices_around_face(halfedge(fh, mesh), mesh))
    {
      x += get(ppmap, vh).x();
      y += get(ppmap, vh).y();
      z += get(ppmap, vh).z();
      ++total;
    }

    fitems->append(new TextItem((float)x / total + offset.x,
                                (float)y / total + offset.y,
                                (float)z / total + offset.z,
                                QString("%1").arg(get(idmap, fh)), true, font, Qt::blue));
  }
  //add the QList to the render's pool
  renderer->addTextList(fitems);
  if(fitems->size() > static_cast<std::size_t>(renderer->getMax_textItems()))
  {
    return false;
  }
  return true;
}

template<class Mesh, typename Point>
int zoomToId(const Mesh& mesh,
             const QString& text,
             CGAL::Three::Viewer_interface* viewer,
             typename boost::graph_traits<Mesh>::face_descriptor& selected_fh,
             Point& p)
{
  typedef typename boost::graph_traits<Mesh>::face_descriptor face_descriptor;
  typedef typename boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
  typedef typename boost::property_map<Mesh, boost::vertex_point_t>::const_type Ppmap;
  typedef typename boost::property_map<Mesh, boost::vertex_index_t>::type VIDmap;
  typedef typename boost::property_map<Mesh, boost::halfedge_index_t>::type EIDmap;
  typedef typename boost::property_map<Mesh, boost::face_index_t>::type FIDmap;
  typedef typename CGAL::Kernel_traits<Point>::Kernel Traits;


  Ppmap ppmap = get(boost::vertex_point, mesh);
  VIDmap vidmap = get(boost::vertex_index, mesh);
  EIDmap eidmap = get(boost::halfedge_index, mesh);
  FIDmap fidmap = get(boost::face_index, mesh);

  bool is_int;
  typename boost::property_traits<VIDmap>::value_type id = text.right(text.length()-1).toUInt(&is_int);
  QString first = text.left(1);
  if((first != QString("v") &&
      first != QString("e") &&
      first != QString("f")) ||
     !is_int)
  {
    return 1; //("Input must be of the form [v/e/f][int]"
  }
  const qglviewer::Vec offset = viewer->offset();
  typename Traits::Vector_3 normal;
  if(first == QString("v"))
  {
    bool found = false;
    BOOST_FOREACH(vertex_descriptor vh, vertices(mesh))
    {
      if(get(vidmap, vh) == id)
      {
        p = Point(get(ppmap, vh).x() + offset.x,
                  get(ppmap, vh).y() + offset.y,
                  get(ppmap, vh).z() + offset.z);
        selected_fh = face(halfedge(vh, mesh), mesh);
        normal = CGAL::Polygon_mesh_processing::compute_vertex_normal(vh, mesh);
        found = true;
        break;
      }
    }
    if(!found)
    {
      return 2;//"No vertex with id %1").arg(id)
    }
  }
  else if(first == QString("e"))
  {
    bool found = false;
    BOOST_FOREACH(typename boost::graph_traits<Mesh>::edge_descriptor e, edges(mesh))
    {
      if(get(eidmap, halfedge(e, mesh))/2 == id)
      {
        const Point& p1 = get(ppmap, source(e, mesh));
        const Point& p2 = get(ppmap, target(e, mesh));
        p = Point((float)(p1.x() + p2.x()) / 2 + offset.x,
                  (float)(p1.y() + p2.y()) / 2 + offset.y,
                  (float)(p1.z() + p2.z()) / 2 + offset.z );
        typename Traits::Vector_3 normal1 = CGAL::Polygon_mesh_processing::compute_face_normal(face(halfedge(e, mesh),mesh),
                                                                                               mesh);
        typename Traits::Vector_3 normal2 = CGAL::Polygon_mesh_processing::compute_face_normal(face(opposite(halfedge(e, mesh), mesh), mesh),
                                                                                               mesh);
        normal = 0.5*normal1+0.5*normal2;
        selected_fh = face(halfedge(e, mesh), mesh);
        found = true;
        break;
      }
    }
    if(!found)
    {
      return 3;//"No edge with id %1").arg(id)
    }
  }
  else if(first == QString("f"))
  {
    bool found = false;
    double x(0), y(0), z(0);
    int total(0);
    BOOST_FOREACH(face_descriptor fh, faces(mesh))
    {
      if(get(fidmap, fh) != id)
        continue;
      BOOST_FOREACH(vertex_descriptor vh, vertices_around_face(halfedge(fh, mesh), mesh))
      {
        x+=get(ppmap, vh).x();
        y+=get(ppmap, vh).y();
        z+=get(ppmap, vh).z();
        ++total;
      }
      p = Point(x/total + offset.x,
                          y/total + offset.y,
                          z/total + offset.z);
      normal = CGAL::Polygon_mesh_processing::compute_face_normal(
            fh,
            mesh);
      selected_fh = fh;
      found = true;
      break;
    }
    if(!found)
    {
      return 4; //"No face with id %1").arg(id)
    }
  }
  qglviewer::Quaternion new_orientation(qglviewer::Vec(0,0,-1),
                                        qglviewer::Vec(-normal.x(), -normal.y(), -normal.z()));
  Point new_pos = p +
      qglviewer::Vec(
        viewer->camera()->position().x - viewer->camera()->sceneCenter().x,
        viewer->camera()->position().y - viewer->camera()->sceneCenter().y,
        viewer->camera()->position().z - viewer->camera()->sceneCenter().z)
      .norm() * normal ;

#if QGLVIEWER_VERSION >= 0x020502
  viewer->camera()->setPivotPoint(qglviewer::Vec(p.x(),
                                                 p.y(),
                                                 p.z()));
#else
  viewer->camera()->setRevolveAroundPoint(qglviewer::Vec(p.x(),
                                                         p.y(),
                                                         p.z()));
#endif

  viewer->moveCameraToCoordinates(QString("%1 %2 %3 %4 %5 %6 %7").arg(new_pos.x())
                                  .arg(new_pos.y())
                                  .arg(new_pos.z())
                                  .arg(new_orientation[0])
      .arg(new_orientation[1])
      .arg(new_orientation[2])
      .arg(new_orientation[3]));
  viewer->update();
  return 0; //all clear;
}
#endif // ID_PRINTING_H

