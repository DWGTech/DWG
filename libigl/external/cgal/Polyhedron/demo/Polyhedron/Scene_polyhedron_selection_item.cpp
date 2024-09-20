#include <QApplication>
#include "Scene_polyhedron_selection_item.h"
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/boost/graph/dijkstra_shortest_paths.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/property_map.h>
#include <CGAL/Handle_hash_function.h>
#include <CGAL/Unique_hash_map.h>

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/range.hpp>

#include <exception>
#include <functional>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "triangulate_primitive.h"

#ifdef USE_SURFACE_MESH
typedef Scene_surface_mesh_item Scene_face_graph_item;
#else
typedef Scene_polyhedron_item Scene_face_graph_item;
#endif

typedef Scene_face_graph_item::Face_graph Face_graph;
typedef boost::property_map<Face_graph,CGAL::vertex_point_t>::type VPmap;
typedef boost::property_map<Face_graph,CGAL::vertex_point_t>::const_type constVPmap;

typedef Scene_face_graph_item::Vertex_selection_map Vertex_selection_map;

typedef boost::graph_traits<Face_graph>::vertex_descriptor fg_vertex_descriptor;
typedef boost::graph_traits<Face_graph>::face_descriptor fg_face_descriptor;
typedef boost::graph_traits<Face_graph>::edge_descriptor fg_edge_descriptor;
typedef boost::graph_traits<Face_graph>::halfedge_descriptor fg_halfedge_descriptor;

struct Scene_polyhedron_selection_item_priv{

  typedef Scene_facegraph_item_k_ring_selection::Active_handle Active_handle;
  typedef boost::unordered_set<fg_vertex_descriptor, CGAL::Handle_hash_function>    Selection_set_vertex;
  typedef boost::unordered_set<fg_face_descriptor, CGAL::Handle_hash_function>      Selection_set_facet;
  typedef boost::unordered_set<fg_edge_descriptor, CGAL::Handle_hash_function>    Selection_set_edge;
  struct vertex_on_path
  {
    fg_vertex_descriptor vertex;
    bool is_constrained;
  };

  Scene_polyhedron_selection_item_priv(Scene_polyhedron_selection_item* parent):
    item(parent)
  {
  }

  void initializeBuffers(CGAL::Three::Viewer_interface *viewer) const;
  void initialize_temp_buffers(CGAL::Three::Viewer_interface *viewer) const;
  void initialize_HL_buffers(CGAL::Three::Viewer_interface *viewer) const;
  void computeElements() const;
  void compute_any_elements(std::vector<float> &p_facets, std::vector<float> &p_lines, std::vector<float> &p_points, std::vector<float> &p_normals,
                            const Selection_set_vertex& p_sel_vertex, const Selection_set_facet &p_sel_facet, const Selection_set_edge &p_sel_edges) const;
  void compute_temp_elements() const;
  void compute_HL_elements() const;
  void triangulate_facet(fg_face_descriptor, Kernel::Vector_3 normal,
                         std::vector<float> &p_facets,std::vector<float> &p_normals) const;
  void tempInstructions(QString s1, QString s2);

  void computeAndDisplayPath();
  void addVertexToPath(fg_vertex_descriptor, vertex_on_path &);

  enum VAOs{
    Facets = 0,
    TempFacets,
    Edges,
    TempEdges,
    Points,
    TempPoints,
    FixedPoints,
    HLPoints,
    HLEdges,
    HLFacets,
    NumberOfVaos
  };
  enum VBOs{
    VertexFacets = 0,
    NormalFacets,
    VertexEdges,
    VertexPoints,
    VertexTempFacets,
    NormalTempFacets,
    VertexTempEdges,
    VertexTempPoints,
    VertexFixedPoints,
    ColorFixedPoints,
    VertexHLPoints,
    VertexHLEdges,
    VertexHLFacets,
    NormalHLFacets,
    NumberOfVbos
  };

  QList<vertex_on_path> path;
  QList<fg_vertex_descriptor> constrained_vertices;
  bool is_path_selecting;
  bool poly_need_update;
  mutable bool are_temp_buffers_filled;
  //Specifies Selection/edition mode
  bool first_selected;
  int operation_mode;
  QString m_temp_instructs;
  bool is_treated;
  fg_vertex_descriptor to_split_vh;
  fg_face_descriptor to_split_fh;
  fg_edge_descriptor to_join_ed;
  Active_handle::Type original_sel_mode;
  //Only needed for the triangulation
  Face_graph* poly;
  CGAL::Unique_hash_map<fg_face_descriptor, Kernel::Vector_3>  face_normals_map;
  CGAL::Unique_hash_map<fg_vertex_descriptor, Kernel::Vector_3>  vertex_normals_map;
  boost::associative_property_map< CGAL::Unique_hash_map<fg_face_descriptor, Kernel::Vector_3> >
    nf_pmap;
  boost::associative_property_map< CGAL::Unique_hash_map<fg_vertex_descriptor, Kernel::Vector_3> >
    nv_pmap;
  Scene_face_graph_item::ManipulatedFrame *manipulated_frame;
  bool ready_to_move;

  Vertex_selection_map vertex_selection_map()
  {
    return item->poly_item->vertex_selection_map();
  }

  Face_graph* polyhedron() { return poly; }
  const Face_graph* polyhedron()const { return poly; }

  bool canAddFace(fg_halfedge_descriptor hc, Scene_polyhedron_selection_item::fg_halfedge_descriptor t);
  bool canAddFaceAndVertex(Scene_polyhedron_selection_item::fg_halfedge_descriptor hc, Scene_polyhedron_selection_item::fg_halfedge_descriptor t);

  mutable std::vector<float> positions_facets;
  mutable std::vector<float> normals;
  mutable std::vector<float> positions_lines;
  mutable std::vector<float> positions_points;
  mutable std::size_t nb_facets;
  mutable std::size_t nb_points;
  mutable std::size_t nb_lines;

  mutable std::vector<float> positions_temp_facets;
  mutable std::vector<float> positions_fixed_points;
  mutable std::vector<float> color_fixed_points;
  mutable std::vector<float> temp_normals;
  mutable std::vector<float> positions_temp_lines;
  mutable std::vector<float> positions_temp_points;
  mutable std::vector<float> positions_HL_facets;
  mutable std::vector<float> HL_normals;
  mutable std::vector<float> positions_HL_lines;
  mutable std::vector<float> positions_HL_points;

  mutable std::size_t nb_temp_facets;
  mutable std::size_t nb_temp_points;
  mutable std::size_t nb_temp_lines;
  mutable std::size_t nb_fixed_points;

  mutable QOpenGLShaderProgram *program;
  mutable bool are_HL_buffers_filled;
  Scene_polyhedron_selection_item* item;
};


void Scene_polyhedron_selection_item_priv::initializeBuffers(CGAL::Three::Viewer_interface *viewer)const
{
  //vao containing the data for the facets
  program = item->getShaderProgram(Scene_polyhedron_selection_item::PROGRAM_WITH_LIGHT, viewer);
  program->bind();
    item->vaos[Facets]->bind();
      item->buffers[VertexFacets].bind();
        item->buffers[VertexFacets].allocate(positions_facets.data(),
                            static_cast<int>(positions_facets.size()*sizeof(float)));
        program->enableAttributeArray("vertex");
        program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
      item->buffers[VertexFacets].release();
      item->buffers[NormalFacets].bind();
        item->buffers[NormalFacets].allocate(normals.data(),
                            static_cast<int>(normals.size()*sizeof(float)));
        program->enableAttributeArray("normals");
        program->setAttributeBuffer("normals",GL_FLOAT,0,3);
      item->buffers[NormalFacets].release();
      program->disableAttributeArray("colors");
    item->vaos[Facets]->release();
  program->release();


  program = item->getShaderProgram(Scene_polyhedron_selection_item::PROGRAM_NO_SELECTION, viewer);
  program->bind();
  //vao containing the data for the points
    item->vaos[Points]->bind();
      item->buffers[VertexPoints].bind();
        item->buffers[VertexPoints].allocate(positions_points.data(),
                            static_cast<int>(positions_points.size()*sizeof(float)));
        program->enableAttributeArray("vertex");
        program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
      item->buffers[VertexPoints].release();
      program->disableAttributeArray("colors");
    item->vaos[Points]->release();

  //vao containing the data for the  lines

    item->vaos[Edges]->bind();
      item->buffers[VertexEdges].bind();
        item->buffers[VertexEdges].allocate(positions_lines.data(),
                            static_cast<int>(positions_lines.size()*sizeof(float)));
        program->enableAttributeArray("vertex");
        program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
      item->buffers[VertexEdges].release();
      program->disableAttributeArray("colors");
    item->vaos[Edges]->release();
  program->release();



  nb_facets = positions_facets.size();
  positions_facets.resize(0);
  positions_facets.shrink_to_fit();

  normals.resize(0);
  normals.shrink_to_fit();

  nb_lines = positions_lines.size();
  positions_lines.resize(0);
  positions_lines.shrink_to_fit();

  nb_points = positions_points.size();
  positions_points.resize(0);
  positions_points.shrink_to_fit();

  item->are_buffers_filled = true;
}

void Scene_polyhedron_selection_item_priv::initialize_temp_buffers(CGAL::Three::Viewer_interface *viewer)const
{
  //vao containing the data for the temp facets
  {
    program = item->getShaderProgram(Scene_polyhedron_selection_item::PROGRAM_WITH_LIGHT, viewer);
    program->bind();

    item->vaos[TempFacets]->bind();
    item->buffers[VertexTempFacets].bind();
    item->buffers[VertexTempFacets].allocate(positions_temp_facets.data(),
                        static_cast<int>(positions_temp_facets.size()*sizeof(float)));
    program->enableAttributeArray("vertex");
    program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
    item->buffers[VertexTempFacets].release();



    item->buffers[NormalTempFacets].bind();
    item->buffers[NormalTempFacets].allocate(temp_normals.data(),
                        static_cast<int>(temp_normals.size()*sizeof(float)));
    program->enableAttributeArray("normals");
    program->setAttributeBuffer("normals",GL_FLOAT,0,3);
    item->buffers[NormalTempFacets].release();
    program->disableAttributeArray("colors");
    item->vaos[TempFacets]->release();
    program->release();
  }
  //vao containing the data for the temp lines
  {
    program = item->getShaderProgram(Scene_polyhedron_selection_item::PROGRAM_NO_SELECTION, viewer);
    program->bind();
    item->vaos[TempEdges]->bind();

    item->buffers[VertexTempEdges].bind();
    item->buffers[VertexTempEdges].allocate(positions_temp_lines.data(),
                        static_cast<int>(positions_temp_lines.size()*sizeof(float)));
    program->enableAttributeArray("vertex");
    program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
    item->buffers[VertexTempEdges].release();
    program->disableAttributeArray("colors");
    item->vaos[TempEdges]->release();
    program->release();


  }
  //vaos containing the data for the temp points
  {
    program = item->getShaderProgram(Scene_polyhedron_selection_item::PROGRAM_NO_SELECTION, viewer);
    program->bind();
    item->vaos[TempPoints]->bind();

    item->buffers[VertexTempPoints].bind();
    item->buffers[VertexTempPoints].allocate(positions_temp_points.data(),
                        static_cast<int>(positions_temp_points.size()*sizeof(float)));
    program->enableAttributeArray("vertex");
    program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
    item->buffers[VertexTempPoints].release();
    program->disableAttributeArray("colors");
    item->vaos[TempPoints]->release();

    item->vaos[FixedPoints]->bind();

    item->buffers[VertexFixedPoints].bind();
    item->buffers[VertexFixedPoints].allocate(positions_fixed_points.data(),
                        static_cast<int>(positions_fixed_points.size()*sizeof(float)));
    program->enableAttributeArray("vertex");
    program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
    item->buffers[VertexFixedPoints].release();
    item->buffers[ColorFixedPoints].bind();
    item->buffers[ColorFixedPoints].allocate(color_fixed_points.data(),
                        static_cast<int>(color_fixed_points.size()*sizeof(float)));
    program->enableAttributeArray("colors");
    program->setAttributeBuffer("colors",GL_FLOAT,0,3);
    item->buffers[ColorFixedPoints].release();
    program->disableAttributeArray("colors");
    item->vaos[FixedPoints]->release();

    program->release();
  }

  nb_temp_facets = positions_temp_facets.size();
  positions_temp_facets.resize(0);
  std::vector<float>(positions_temp_facets).swap(positions_temp_facets);

  temp_normals.resize(0);
  std::vector<float>(temp_normals).swap(temp_normals);

  nb_temp_lines = positions_temp_lines.size();
  positions_temp_lines.resize(0);
  std::vector<float>(positions_temp_lines).swap(positions_temp_lines);

  nb_temp_points = positions_temp_points.size();
  positions_temp_points.resize(0);
  std::vector<float>(positions_temp_points).swap(positions_temp_points);

  nb_fixed_points = positions_fixed_points.size();
  positions_fixed_points.resize(0);
  std::vector<float>(positions_fixed_points).swap(positions_fixed_points);
  are_temp_buffers_filled = true;
}

void Scene_polyhedron_selection_item_priv::initialize_HL_buffers(CGAL::Three::Viewer_interface *viewer)const
{
  //vao containing the data for the temp facets
  {
    program = item->getShaderProgram(Scene_polyhedron_selection_item::PROGRAM_WITH_LIGHT, viewer);
    program->bind();

    item->vaos[HLFacets]->bind();
    item->buffers[VertexHLFacets].bind();
    item->buffers[VertexHLFacets].allocate(positions_HL_facets.data(),
                        static_cast<int>(positions_HL_facets.size()*sizeof(float)));
    program->enableAttributeArray("vertex");
    program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
    item->buffers[VertexHLFacets].release();


    item->buffers[NormalHLFacets].bind();
    item->buffers[NormalHLFacets].allocate(HL_normals.data(),
                        static_cast<int>(HL_normals.size()*sizeof(float)));
    program->enableAttributeArray("normals");
    program->setAttributeBuffer("normals",GL_FLOAT,0,3);
    item->buffers[NormalHLFacets].release();
    program->disableAttributeArray("colors");
    item->vaos[HLFacets]->release();
    program->release();

  }
  //vao containing the data for the temp lines
  {
    program = item->getShaderProgram(Scene_polyhedron_selection_item::PROGRAM_NO_SELECTION, viewer);
    program->bind();
    item->vaos[HLEdges]->bind();

    item->buffers[VertexHLEdges].bind();
    item->buffers[VertexHLEdges].allocate(positions_HL_lines.data(),
                        static_cast<int>(positions_HL_lines.size()*sizeof(float)));
    program->enableAttributeArray("vertex");
    program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
    item->buffers[VertexHLEdges].release();
    program->disableAttributeArray("colors");
    item->vaos[HLEdges]->release();
    program->release();


  }
  //vao containing the data for the temp points
  {
    program = item->getShaderProgram(Scene_polyhedron_selection_item::PROGRAM_NO_SELECTION, viewer);
    program->bind();
    item->vaos[HLPoints]->bind();

    item->buffers[VertexHLPoints].bind();
    item->buffers[VertexHLPoints].allocate(positions_HL_points.data(),
                        static_cast<int>(positions_HL_points.size()*sizeof(float)));
    program->enableAttributeArray("vertex");
    program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
    item->buffers[VertexHLPoints].release();
    program->disableAttributeArray("colors");
    item->vaos[HLPoints]->release();
    program->release();

  }
  are_HL_buffers_filled = true;
}
template<typename TypeWithXYZ, typename ContainerWithPushBack>
void push_back_xyz(const TypeWithXYZ& t,
                   ContainerWithPushBack& vector)
{
  vector.push_back(t.x());
  vector.push_back(t.y());
  vector.push_back(t.z());
}

typedef Kernel Traits;

//Make sure all the facets are triangles
typedef Traits::Point_3	            Point_3;
typedef Traits::Point_3	            Point;
typedef Traits::Vector_3	    Vector;

void
Scene_polyhedron_selection_item_priv::triangulate_facet(fg_face_descriptor fit,const Vector normal,
                                                   std::vector<float> &p_facets,std::vector<float> &p_normals ) const
{
  typedef FacetTriangulator<Face_graph, Kernel, fg_vertex_descriptor> FT;
  double diagonal;
  if(item->poly_item->diagonalBbox() != std::numeric_limits<double>::infinity())
    diagonal = item->poly_item->diagonalBbox();
  else
    diagonal = 0.0;
  FT triangulation(fit,normal,poly,diagonal);
    //iterates on the internal faces to add the vertices to the positions
    //and the normals to the appropriate vectors
    for(FT::CDT::Finite_faces_iterator
        ffit = triangulation.cdt->finite_faces_begin(),
        end = triangulation.cdt->finite_faces_end();
        ffit != end; ++ffit)
    {
        if(ffit->info().is_external)
            continue;

        push_back_xyz(ffit->vertex(0)->point(), p_facets);
        push_back_xyz(ffit->vertex(1)->point(), p_facets);
        push_back_xyz(ffit->vertex(2)->point(), p_facets);

        push_back_xyz(normal, p_normals);
        push_back_xyz(normal, p_normals);
        push_back_xyz(normal, p_normals);
    }
}


void Scene_polyhedron_selection_item_priv::compute_any_elements(std::vector<float>& p_facets, std::vector<float>& p_lines, std::vector<float>& p_points, std::vector<float>& p_normals,
                                                           const Selection_set_vertex& p_sel_vertices, const Selection_set_facet& p_sel_facets, const Selection_set_edge& p_sel_edges)const
{
    const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
    p_facets.clear();
    p_lines.clear();
    p_points.clear();
    p_normals.clear();
    //The facet

    if(!poly)
      return;

    VPmap vpm = get(CGAL::vertex_point,*poly);
    for(Selection_set_facet::iterator
        it = p_sel_facets.begin(),
        end = p_sel_facets.end();
        it != end; it++)
    {
      fg_face_descriptor f = (*it);
      if (f == boost::graph_traits<Face_graph>::null_face())
        continue;
      Vector nf = get(nf_pmap, f);
      if(is_triangle(halfedge(f,*poly),*poly))
      {
        p_normals.push_back(nf.x());
        p_normals.push_back(nf.y());
        p_normals.push_back(nf.z());

        p_normals.push_back(nf.x());
        p_normals.push_back(nf.y());
        p_normals.push_back(nf.z());

        p_normals.push_back(nf.x());
        p_normals.push_back(nf.y());
        p_normals.push_back(nf.z());


        BOOST_FOREACH(fg_halfedge_descriptor he, halfedges_around_face(halfedge(f,*polyhedron()), *polyhedron()))
        {
          const Point& p = get(vpm,target(he,*poly));
          p_facets.push_back(p.x()+offset.x);
          p_facets.push_back(p.y()+offset.y);
          p_facets.push_back(p.z()+offset.z);
        }
      }
      else if (is_quad(halfedge(f,*poly), *poly))
      {
        Kernel::Vector_3 v_offset(offset.x, offset.y, offset.z);
        Vector nf = get(nf_pmap, f);
        {
          //1st half-quad
          const Point& p0 = get(vpm,target(halfedge(f,*poly),*poly));
          const Point& p1 = get(vpm,target(next(halfedge(f,*poly),*poly),*poly));
          const Point& p2 = get(vpm,target(next(next(halfedge(f,*poly),*poly),*poly),*poly));

          push_back_xyz(p0+v_offset, p_facets);
          push_back_xyz(p1+v_offset, p_facets);
          push_back_xyz(p2+v_offset, p_facets);

          push_back_xyz(nf, p_normals);
          push_back_xyz(nf, p_normals);
          push_back_xyz(nf, p_normals);
        }
        {
          //2nd half-quad
          const Point& p0 = get(vpm, target(next(next(halfedge(f,*poly),*poly),*poly),*poly));
          const Point& p1 = get(vpm, target(prev(halfedge(f,*poly),*poly),*poly));
          const Point& p2 = get(vpm, target(halfedge(f,*poly),*poly));

          push_back_xyz(p0+v_offset, p_facets);
          push_back_xyz(p1+v_offset, p_facets);
          push_back_xyz(p2+v_offset, p_facets);

          push_back_xyz(nf, p_normals);
          push_back_xyz(nf, p_normals);
          push_back_xyz(nf, p_normals);
        }
      }
      else
      {
        triangulate_facet(f, nf, p_facets, p_normals);
      }
    }

    //The Lines
    {

        for(Selection_set_edge::iterator it = p_sel_edges.begin(); it != p_sel_edges.end(); ++it) {
          const Point& a = get(vpm, target(halfedge(*it,*poly),*poly));
          const Point& b = get(vpm, target(opposite((halfedge(*it,*poly)),*poly),*poly));
            p_lines.push_back(a.x()+offset.x);
            p_lines.push_back(a.y()+offset.y);
            p_lines.push_back(a.z()+offset.z);

            p_lines.push_back(b.x()+offset.x);
            p_lines.push_back(b.y()+offset.y);
            p_lines.push_back(b.z()+offset.z);
        }

    }
    //The points
    {
        for(Selection_set_vertex::iterator
            it = p_sel_vertices.begin(),
            end = p_sel_vertices.end();
            it != end; ++it)
        {
          const Point& p = get(vpm, *it);
            p_points.push_back(p.x()+offset.x);
            p_points.push_back(p.y()+offset.y);
            p_points.push_back(p.z()+offset.z);
        }
    }
}
void Scene_polyhedron_selection_item_priv::computeElements()const
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  compute_any_elements(positions_facets, positions_lines, positions_points, normals,
                       item->selected_vertices, item->selected_facets, item->selected_edges);
  QApplication::restoreOverrideCursor();
}
void Scene_polyhedron_selection_item_priv::compute_temp_elements()const
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  compute_any_elements(positions_temp_facets, positions_temp_lines, positions_temp_points, temp_normals,
                       item->temp_selected_vertices, item->temp_selected_facets, item->temp_selected_edges);
  //The fixed points
  {
    const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
    color_fixed_points.clear();
    positions_fixed_points.clear();
    int i=0;

    constVPmap vpm = get(CGAL::vertex_point,*polyhedron());

    for(Scene_polyhedron_selection_item::Selection_set_vertex::iterator
        it = item->fixed_vertices.begin(),
        end = item->fixed_vertices.end();
        it != end; ++it)
    {
      const Point& p = get(vpm,*it);
      positions_fixed_points.push_back(p.x()+offset.x);
      positions_fixed_points.push_back(p.y()+offset.y);
      positions_fixed_points.push_back(p.z()+offset.z);

      if(*it == constrained_vertices.first()|| *it == constrained_vertices.last())
      {
        color_fixed_points.push_back(0.0);
        color_fixed_points.push_back(0.0);
        color_fixed_points.push_back(1.0);
      }
      else
      {
        color_fixed_points.push_back(1.0);
        color_fixed_points.push_back(0.0);
        color_fixed_points.push_back(0.0);
      }
      i++;
    }
  }
  QApplication::restoreOverrideCursor();
}

void Scene_polyhedron_selection_item_priv::compute_HL_elements()const
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  compute_any_elements(positions_HL_facets, positions_HL_lines, positions_HL_points, HL_normals,
                       item->HL_selected_vertices, item->HL_selected_facets, item->HL_selected_edges);
  QApplication::restoreOverrideCursor();
}

void Scene_polyhedron_selection_item::draw(CGAL::Three::Viewer_interface* viewer) const
{
  GLfloat offset_factor;
  GLfloat offset_units;

  if(!d->are_HL_buffers_filled)
  {
    d->compute_HL_elements();
    d->initialize_HL_buffers(viewer);
  }

  viewer->glGetFloatv(GL_POLYGON_OFFSET_FACTOR, &offset_factor);
  viewer->glGetFloatv(GL_POLYGON_OFFSET_UNITS, &offset_units);
  viewer->glPolygonOffset(0.5f, 0.9f);

  vaos[Scene_polyhedron_selection_item_priv::HLFacets]->bind();
  d->program = getShaderProgram(PROGRAM_WITH_LIGHT);
  attribBuffers(viewer,PROGRAM_WITH_LIGHT);
  d->program->bind();
  d->program->setAttributeValue("colors",QColor(255,153,51));
  viewer->glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(d->positions_HL_facets.size())/3);
  d->program->release();
  vaos[Scene_polyhedron_selection_item_priv::HLFacets]->release();

  if(!d->are_temp_buffers_filled)
  {
    d->compute_temp_elements();
    d->initialize_temp_buffers(viewer);
  }
  vaos[Scene_polyhedron_selection_item_priv::TempFacets]->bind();

  attribBuffers(viewer,PROGRAM_WITH_LIGHT);
  d->program->bind();
  d->program->setAttributeValue("colors",QColor(0,255,0));
  viewer->glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(d->nb_temp_facets/3));
  d->program->release();
  vaos[Scene_polyhedron_selection_item_priv::TempFacets]->release();
  d->program = getShaderProgram(PROGRAM_WITH_LIGHT);
  if(!are_buffers_filled)
  {
    d->computeElements();
    d->initializeBuffers(viewer);
  }
  viewer->makeCurrent();
  vaos[Scene_polyhedron_selection_item_priv::Facets]->bind();

  attribBuffers(viewer,PROGRAM_WITH_LIGHT);
  d->program->bind();
  d->program->setAttributeValue("colors",this->color());
  viewer->glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(d->nb_facets/3));
  d->program->release();
  vaos[Scene_polyhedron_selection_item_priv::Facets]->release();

  viewer->glEnable(GL_POLYGON_OFFSET_LINE);
  viewer->glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
  viewer->glPolygonOffset(0.0f, 1.5f);
  drawEdges(viewer);
  viewer->glDisable(GL_POLYGON_OFFSET_LINE);
  viewer->glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
  viewer->glPolygonOffset(offset_factor, offset_units);
  drawPoints(viewer);
  viewer->glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
}

void Scene_polyhedron_selection_item::drawEdges(CGAL::Three::Viewer_interface* viewer) const
{
  viewer->glLineWidth(3.f);

  if(!d->are_HL_buffers_filled)
  {
    d->compute_HL_elements();
    d->initialize_HL_buffers(viewer);
  }

  vaos[Scene_polyhedron_selection_item_priv::HLEdges]->bind();
  d->program = getShaderProgram(PROGRAM_NO_SELECTION);
  attribBuffers(viewer,PROGRAM_NO_SELECTION);
  d->program->bind();

  d->program->setAttributeValue("colors",QColor(255,153,51));
  viewer->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(d->positions_HL_lines.size()/3));
  d->program->release();
  vaos[Scene_polyhedron_selection_item_priv::HLEdges]->release();

  if(!d->are_temp_buffers_filled)
  {
    d->compute_temp_elements();
    d->initialize_temp_buffers(viewer);
  }

  vaos[Scene_polyhedron_selection_item_priv::TempEdges]->bind();
  d->program = getShaderProgram(PROGRAM_NO_SELECTION);
  attribBuffers(viewer,PROGRAM_NO_SELECTION);
  d->program->bind();

  d->program->setAttributeValue("colors",QColor(0,200,0));
  viewer->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(d->nb_temp_lines/3));
  d->program->release();
  vaos[Scene_polyhedron_selection_item_priv::TempEdges]->release();
  viewer->glLineWidth(3.0f);
  if(!are_buffers_filled)
  {
    d->computeElements();
    d->initializeBuffers(viewer);
  }
  viewer->makeCurrent();
  vaos[Scene_polyhedron_selection_item_priv::Edges]->bind();
  d->program = getShaderProgram(PROGRAM_NO_SELECTION);
  attribBuffers(viewer,PROGRAM_NO_SELECTION);
  d->program->bind();

  d->program->setAttributeValue("colors",QColor(255,
                                                color().blue()/2,
                                                color().green()/2));
  viewer->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(d->nb_lines/3));
  d->program->release();
  vaos[Scene_polyhedron_selection_item_priv::Edges]->release();


  viewer->glLineWidth(1.f);
}

void Scene_polyhedron_selection_item::drawPoints(CGAL::Three::Viewer_interface* viewer) const
{

  viewer->glPointSize(5.5f);

  if(!d->are_HL_buffers_filled)
  {
    d->compute_HL_elements();
    d->initialize_HL_buffers(viewer);
  }
  d->program = getShaderProgram(PROGRAM_NO_SELECTION);
  attribBuffers(viewer,PROGRAM_NO_SELECTION);
  vaos[Scene_polyhedron_selection_item_priv::HLPoints]->bind();
  d->program->bind();
  d->program->setAttributeValue("colors",QColor(255,153,51));
  viewer->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(d->positions_HL_points.size()/3));
  d->program->release();
  vaos[Scene_polyhedron_selection_item_priv::HLPoints]->release();

  if(!d->are_temp_buffers_filled)
  {
    d->compute_temp_elements();
    d->initialize_temp_buffers(viewer);
  }

  vaos[Scene_polyhedron_selection_item_priv::TempPoints]->bind();
  d->program = getShaderProgram(PROGRAM_NO_SELECTION);
  attribBuffers(viewer,PROGRAM_NO_SELECTION);
  d->program->bind();
  d->program->setAttributeValue("colors",QColor(0,50,0));
  viewer->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(d->nb_temp_points/3));
  vaos[Scene_polyhedron_selection_item_priv::TempPoints]->release();
  vaos[Scene_polyhedron_selection_item_priv::FixedPoints]->bind();
  viewer->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(d->nb_fixed_points/3));
  d->program->release();
  vaos[Scene_polyhedron_selection_item_priv::FixedPoints]->release();

  if(!are_buffers_filled)
  {
    d->computeElements();
    d->initializeBuffers(viewer);
  }
  viewer->makeCurrent();
  vaos[Scene_polyhedron_selection_item_priv::Points]->bind();
  d->program = getShaderProgram(PROGRAM_NO_SELECTION);
  attribBuffers(viewer,PROGRAM_NO_SELECTION);
  d->program->bind();
  d->program->setAttributeValue("colors",QColor(255,
                                                (std::min)(color().blue()+color().red(), 255),
                                                (std::min)(color().green()+color().red(), 255)));
  viewer->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(d->nb_points/3));
  d->program->release();
  vaos[Points]->release();

  viewer->glPointSize(1.f);
}


void Scene_polyhedron_selection_item::inverse_selection()
{
  switch(k_ring_selector.active_handle_type)
  {
  case Active_handle::VERTEX:
  {
    Selection_set_vertex temp_select = selected_vertices;
    select_all();
    Q_FOREACH(fg_vertex_descriptor vh, temp_select)
    {
      selected_vertices.erase(vh);
    }
    break;
  }
  case Active_handle::EDGE:
  {
    Selection_set_edge temp_select = selected_edges;
    select_all();
    Q_FOREACH(fg_edge_descriptor ed , temp_select)
      selected_edges.erase(ed);
    break;
  }
  default:
  {
    Selection_set_facet temp_select = selected_facets;
    select_all();
    Q_FOREACH(fg_face_descriptor fh, temp_select)
      selected_facets.erase(fh);
    break;
  }
  }
  invalidateOpenGLBuffers();
  QGLViewer* v = *QGLViewer::QGLViewerPool().begin();
  v->update();
}

void Scene_polyhedron_selection_item::set_operation_mode(int mode)
{
  k_ring_selector.setEditMode(true);
  Q_EMIT updateInstructions(QString("SHIFT + left click to apply operation."));
  switch(mode)
  {
  case -2:
    set_active_handle_type(d->original_sel_mode);
    Q_EMIT updateInstructions("Select two vertices to create the path between them. (1/2)");
    break;
  case -1:
    //restore original selection_type
    set_active_handle_type(d->original_sel_mode);
    clearHL();
    k_ring_selector.setEditMode(false);
    break;
    //Join vertex
  case 0:
    Q_EMIT updateInstructions("Select the edge with extremities you want to join.");
    //set the selection type to Edge
    set_active_handle_type(static_cast<Active_handle::Type>(2));
    break;
    //Split vertex
  case 1:
    Q_EMIT updateInstructions("Select the vertex you want to split. (1/3)");
    //set the selection type to Vertex
    set_active_handle_type(static_cast<Active_handle::Type>(0));
    break;
    //Split edge
  case 2:
    Q_EMIT updateInstructions("Select the edge you want to split.");
    //set the selection type to Edge
    set_active_handle_type(static_cast<Active_handle::Type>(2));
    break;
    //Join face
  case 3:
    Q_EMIT updateInstructions("Select the edge separating the faces you want to join.");
    //set the selection type to Edge
    set_active_handle_type(static_cast<Active_handle::Type>(2));
    break;
    //Split face
  case 4:
    Q_EMIT updateInstructions("Select the facet you want to split (degree >= 4). (1/3)");
    //set the selection type to Facet
    set_active_handle_type(static_cast<Active_handle::Type>(1));
    break;
    //Collapse edge
  case 5:
    Q_EMIT updateInstructions("Select the edge you want to collapse.");
    //set the selection type to Edge
    set_active_handle_type(static_cast<Active_handle::Type>(2));
    break;
    //Flip edge
  case 6:
    Q_EMIT updateInstructions("Select the edge you want to flip.");
    //set the selection type to Edge
    set_active_handle_type(static_cast<Active_handle::Type>(2));
    break;
    //Add center vertex
  case 7:
    Q_EMIT updateInstructions("Select a facet.");
    //set the selection type to Facet
    set_active_handle_type(static_cast<Active_handle::Type>(1));
    break;
    //Remove center vertex
  case 8:
    Q_EMIT updateInstructions("Select the vertex you want to remove.");
    //set the selection type to vertex
    set_active_handle_type(static_cast<Active_handle::Type>(0));
    break;
    //Add vertex and face to border
  case 9:
    Q_EMIT updateInstructions("Select a border edge. (1/2)");
    //set the selection type to Edge
    set_active_handle_type(static_cast<Active_handle::Type>(2));
    break;
    //Add face to border
  case 10:
    Q_EMIT updateInstructions("Select a border edge. (1/2)");
    //set the selection type to Edge
    set_active_handle_type(static_cast<Active_handle::Type>(2));
    break;
  case 11:
    Q_EMIT updateInstructions("Select a vertex. (1/2)");
    //set the selection type to Edge
    set_active_handle_type(static_cast<Active_handle::Type>(0));
    break;
  default:
    break;
  }
  d->operation_mode = mode;
}
template<typename HandleRange>
bool Scene_polyhedron_selection_item::treat_classic_selection(const HandleRange& selection)
{
  typedef typename HandleRange::value_type HandleType;
  Selection_traits<HandleType, Scene_polyhedron_selection_item> tr(this);
  bool any_change = false;
  if(is_insert) {
    BOOST_FOREACH(HandleType h, selection)
        any_change |= tr.container().insert(h).second;
  }
  else{
    BOOST_FOREACH(HandleType h, selection)
        any_change |= (tr.container().erase(h)!=0);
  }
  if(any_change) { invalidateOpenGLBuffers(); Q_EMIT itemChanged(); }
  return any_change;
}

bool Scene_polyhedron_selection_item::treat_selection(const std::set<fg_vertex_descriptor>& selection)
{
  VPmap vpm = get(CGAL::vertex_point, *polyhedron());
  if(!d->is_treated)
  {
    fg_vertex_descriptor vh = *selection.begin();
    Selection_traits<fg_vertex_descriptor, Scene_polyhedron_selection_item> tr(this);
    switch(d->operation_mode)
    {
    //classic selection
    case -2:
    case -1:
    {
      if(!d->is_path_selecting)
      {
        return treat_classic_selection(selection);
      }
      else
      {
        if(is_insert)
        {
          selectPath(*selection.begin());
          invalidateOpenGLBuffers();
          Q_EMIT itemChanged();
        }
      }
      return false;
      break;
    }
      //Split vertex
    case 1:
    {
      //save VH
      d->to_split_vh = vh;
      temp_selected_vertices.insert(d->to_split_vh);
      //set to select facet
      set_active_handle_type(static_cast<Active_handle::Type>(1));
      invalidateOpenGLBuffers();
      Q_EMIT updateInstructions("Select first facet. (2/3)");
      break;
    }
      //Split face
    case 4:
    {
      static fg_vertex_descriptor s;
      static fg_halfedge_descriptor h1,h2;
      static bool found_h1(false), found_h2(false);
      if(!d->first_selected)
      {
          //Is the vertex on the face ?
        BOOST_FOREACH(fg_halfedge_descriptor hafc, halfedges_around_face(halfedge(d->to_split_fh,*polyhedron()), *polyhedron()))
          {
            if(target(hafc,*polyhedron())==vh)
            {
              h1 = hafc;
              s = vh;
              found_h1 = true;
                break;
            }
          }
          if(!found_h1)
          {
            d->tempInstructions("Vertex not selected : The vertex is not on the face.",
                             "Select the first vertex. (2/3)");
          }
          else
          {
            d->first_selected = true;
            temp_selected_vertices.insert(s);
            invalidateOpenGLBuffers();
            Q_EMIT updateInstructions("Select the second vertex (3/3)");
          }
      }
      else
      {
        bool is_same(false), are_next(false);
        for(int i=0; i<1; i++) //seems useless but allow the use of break.
        {
          //Is the vertex on the face ?
          BOOST_FOREACH(fg_halfedge_descriptor hafc, halfedges_around_face(halfedge(d->to_split_fh,*polyhedron()), *polyhedron()))
            if(target(hafc,*polyhedron())==vh)
          {
            h2 = hafc;
            found_h2 = true;
            break;
          }
          if(!found_h2)
          {
            break;
          }
          //Are they different ?
          if(h1 == h2)
          {
            is_same = true;
            break;
          }
          is_same = false;
          //Are they directly following each other?
          if(next(h1, *polyhedron()) == h2 ||
             next(h2, *polyhedron()) == h1)
          {
            are_next = true;
            break;
          }
          are_next = false;
        }
        if(!found_h2)
          d->tempInstructions("Vertex not selected : The vertex is not on the face.",
                           "Select the second vertex (3/3).");
        else if(is_same)
          d->tempInstructions("Vertex not selected : The vertices must be different.",
                           "Select the second vertex (3/3).");
        else if(are_next)
          d->tempInstructions("Vertex not selected : The vertices must not directly follow each other.",
                           "Select the second vertex (3/3).");
        else
        {
          CGAL::Euler::split_face(h1,h2, *polyhedron());
          d->first_selected = false;
          temp_selected_vertices.clear();
          temp_selected_facets.clear();
          compute_normal_maps();
          invalidateOpenGLBuffers();
          //reset selection type to Facet
          set_active_handle_type(static_cast<Active_handle::Type>(1));
          d->tempInstructions("Face split.",
                           "Select a facet (1/3).");
          polyhedron_item()->invalidateOpenGLBuffers();
        }
      }
      break;
    }
      //Remove center vertex
    case 8:
    {
      bool has_hole = false;
      BOOST_FOREACH(fg_halfedge_descriptor hc, halfedges_around_target(vh,*polyhedron()))
      {
        if(is_border(hc,*polyhedron()))
        {
          has_hole = true;
          break;
        }
      }
      if(!has_hole)
      {
        CGAL::Euler::remove_center_vertex(halfedge(vh,*polyhedron()),*polyhedron());
        compute_normal_maps();
        polyhedron_item()->invalidateOpenGLBuffers();
      }
      else
      {
        d->tempInstructions("Vertex not selected : There must be no hole incident to the selection.",
                         "Select the vertex you want to remove.");
      }
      break;
    }
    case 11:
      QGLViewer* viewer = *QGLViewer::QGLViewerPool().begin();
      const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(viewer)->offset();
      if(viewer->manipulatedFrame() != d->manipulated_frame)
      {
        temp_selected_vertices.insert(vh);
        k_ring_selector.setEditMode(false);
        const Point_3& p = get(vpm,vh);
        d->manipulated_frame->setPosition(p.x()+offset.x, p.y()+offset.y, p.z()+offset.z);
        viewer->setManipulatedFrame(d->manipulated_frame);
        connect(d->manipulated_frame, SIGNAL(modified()), this, SLOT(updateTick()));
        invalidateOpenGLBuffers();
        Q_EMIT updateInstructions("Ctrl+Right-click to move the point. \nHit Ctrl+Z to leave the selection. (2/2)");
      }
      else
      {
        temp_selected_vertices.clear();
        temp_selected_vertices.insert(vh);
        const Point_3& p = get(vpm,vh);
        d->manipulated_frame->setPosition(p.x()+offset.x, p.y()+offset.y, p.z()+offset.z);
        invalidateOpenGLBuffers();
      }
      break;
    }
  }
  d->is_treated = true;
  //Keeps the item from trying to draw primitive that has just been deleted.
  clearHL();
  return false;
}

//returns true if halfedge's facet's degree >= degree
/*
std::size_t facet_degree(fg_halfedge_descriptor h, const Face_graph& polyhedron)
{
  return degree(h,polyhedron);
}
*/
bool Scene_polyhedron_selection_item:: treat_selection(const std::set<fg_edge_descriptor>& selection)
{
  VPmap vpm = get(CGAL::vertex_point, *polyhedron());
  fg_edge_descriptor ed =  *selection.begin();
  if(!d->is_treated)
  {
    Selection_traits<fg_edge_descriptor, Scene_polyhedron_selection_item> tr(this);
    switch(d->operation_mode)
    {
    //classic selection
    case -1:
    {
      return treat_classic_selection(selection);
      break;
    }
      //Join vertex
    case 0:
      if(boost::distance(CGAL::halfedges_around_face(halfedge(ed, *polyhedron()), *polyhedron())) < 4
           ||
         boost::distance(CGAL::halfedges_around_face(opposite(halfedge(ed, *polyhedron()),*polyhedron()),*polyhedron()))< 4)
        {
          d->tempInstructions("Edge not selected: the incident facets must have a degree of at least 4.",
                           "Select the edge with extremities you want to join.");
        }
        else
        {
          fg_halfedge_descriptor targt = halfedge(ed, *polyhedron());
          Point S,T;
          S = get(vpm, source(targt, *polyhedron()));
          T = get(vpm, target(targt, *polyhedron()));
          put(vpm, target(CGAL::Euler::join_vertex(targt,*polyhedron()),*polyhedron()), Point(0.5*(S.x()+T.x()), 0.5*(S.y()+T.y()), 0.5*(S.z()+T.z())));
          d->tempInstructions("Vertices joined.",
                           "Select the edge with extremities you want to join.");
          compute_normal_maps();
          invalidateOpenGLBuffers();
          polyhedron_item()->invalidateOpenGLBuffers();
        }
      break;
      //Split edge
    case 2:
    {

      Point_3 a(get(vpm,target(halfedge(ed, *polyhedron()),*polyhedron()))),
        b(get(vpm,target(opposite(halfedge(ed, *polyhedron()),*polyhedron()),*polyhedron())));
      fg_halfedge_descriptor hhandle = CGAL::Euler::split_edge(halfedge(ed, *polyhedron()),*polyhedron());
        Point_3 p((b.x()+a.x())/2.0, (b.y()+a.y())/2.0,(b.z()+a.z())/2.0);

        put(vpm, target(hhandle,*polyhedron()), p);
        invalidateOpenGLBuffers();
        poly_item->invalidateOpenGLBuffers();
        compute_normal_maps();
        d->tempInstructions("Edge splitted.",
                            "Select the edge you want to split.");
        break;
    }
      //Join face
    case 3:
        if(out_degree(source(halfedge(ed,*polyhedron()),*polyhedron()),*polyhedron())<3 ||
           out_degree(target(halfedge(ed,*polyhedron()),*polyhedron()),*polyhedron())<3)
          d->tempInstructions("Faces not joined : the two ends of the edge must have a degree of at least 3.",
                           "Select the edge separating the faces you want to join.");
        else
        {
          CGAL::Euler::join_face(halfedge(ed, *polyhedron()), *polyhedron());
          compute_normal_maps();
          poly_item->invalidateOpenGLBuffers();
        }
      break;
      //Collapse edge
    case 5:
        if(!is_triangle_mesh(*polyhedron()))
        {
          d->tempInstructions("Edge not collapsed : the graph must be triangulated.",
                           "Select the edge you want to collapse.");
        }
        else if(!CGAL::Euler::does_satisfy_link_condition(ed, *polyhedron()))
        {
          d->tempInstructions("Edge not collapsed : link condition not satidfied.",
                           "Select the edge you want to collapse.");
        }
        else
        {
          fg_halfedge_descriptor targt = halfedge(ed, *polyhedron());
          Point S,T;
          S = get(vpm, source(targt, *polyhedron()));
          T = get(vpm, target(targt, *polyhedron()));

          put(vpm, CGAL::Euler::collapse_edge(ed, *polyhedron()), Point(0.5*(S.x()+T.x()), 0.5*(S.y()+T.y()), 0.5*(S.z()+T.z())));
          compute_normal_maps();
          polyhedron_item()->invalidateOpenGLBuffers();

          d->tempInstructions("Edge collapsed.",
                           "Select the edge you want to collapse.");
        }
      break;
      //Flip edge
    case 6:

        //check preconditions
      if(boost::distance(CGAL::halfedges_around_face(halfedge(ed, *polyhedron()),*polyhedron())) == 3 
         && 
         boost::distance(CGAL::halfedges_around_face(opposite(halfedge(ed, *polyhedron()),*polyhedron()),*polyhedron())) == 3)
        {
          CGAL::Euler::flip_edge(halfedge(ed, *polyhedron()), *polyhedron());
          polyhedron_item()->invalidateOpenGLBuffers();
          compute_normal_maps();
        }
        else
        {
          d->tempInstructions("Edge not selected : incident facets must be triangles.",
                           "Select the edge you want to flip.");
        }

      break;
      //Add vertex and face to border
    case 9:
    {
      static fg_halfedge_descriptor t;
      if(!d->first_selected)
      {
          bool found = false;
          fg_halfedge_descriptor hc = halfedge(ed, *polyhedron());
          if(is_border(hc,*polyhedron()))
          {
            t = hc;
            found = true;
          }
          else if(is_border(opposite(hc,*polyhedron()),*polyhedron()))
          {
            t = opposite(hc,*polyhedron());
            found = true;
          }
          if(found)
          {
            d->first_selected = true;
            temp_selected_edges.insert(edge(t, *polyhedron()));
            temp_selected_vertices.insert(target(t,*polyhedron()));
            invalidateOpenGLBuffers();
            Q_EMIT updateInstructions("Select second edge. (2/2)");
          }
          else
          {
            d->tempInstructions("Edge not selected : no border found.",
                             "Select a border edge. (1/2)");
          }
      }
      else
      {
        fg_halfedge_descriptor hc = halfedge(ed, *polyhedron());
        if(d->canAddFaceAndVertex(hc, t))
        {
          d->first_selected = false;


          temp_selected_edges.clear();
          temp_selected_vertices.clear();
          compute_normal_maps();
          invalidateOpenGLBuffers();
          polyhedron_item()->invalidateOpenGLBuffers();
          d->tempInstructions("Face and vertex added.",
                           "Select a border edge. (1/2)");
        }
      }
      break;
    }
      //Add face to border
    case 10:
    {
      static fg_halfedge_descriptor t;
      if(!d->first_selected)
      {
          bool found = false;
          fg_halfedge_descriptor hc = halfedge(ed, *polyhedron());
          if(is_border(hc,*polyhedron()))
          {
            t = hc;
            found = true;
          }
          else if(is_border(opposite(hc,*polyhedron()),*polyhedron()))
          {
            t = opposite(hc,*polyhedron());
            found = true;
          }
          if(found)
          {
            d->first_selected = true;
            temp_selected_edges.insert(edge(t, *polyhedron()));
            temp_selected_vertices.insert(target(t,*polyhedron()));
            invalidateOpenGLBuffers();
            Q_EMIT updateInstructions("Select second edge. (2/2)");
            set_active_handle_type(static_cast<Active_handle::Type>(2));
          }
          else
          {
            d->tempInstructions("Edge not selected : no border found.",
                             "Select a border edge. (1/2)");
          }
      }
      else
      {
        fg_halfedge_descriptor hc = halfedge(ed, *polyhedron());
        if(d->canAddFace(hc, t))
        {
          d->first_selected = false;
          temp_selected_vertices.clear();
          temp_selected_edges.clear();
          compute_normal_maps();
          invalidateOpenGLBuffers();
          polyhedron_item()->invalidateOpenGLBuffers();
          d->tempInstructions("Face added.",
                           "Select a border edge. (1/2)");
        }
      }
      break;
    }
    }
  }
  d->is_treated = true;
  //Keeps the item from trying to draw primitive that has just been deleted.
  clearHL();
  return false;
}

bool Scene_polyhedron_selection_item::treat_selection(const std::vector<fg_face_descriptor>& selection)
{
  return treat_classic_selection(selection);
}

bool Scene_polyhedron_selection_item::treat_selection(const std::set<fg_face_descriptor>& selection)
{
  VPmap vpm = get(CGAL::vertex_point,*polyhedron());
  if(!d->is_treated)
  {
    fg_face_descriptor fh = *selection.begin();
    Selection_traits<fg_face_descriptor, Scene_polyhedron_selection_item> tr(this);
    switch(d->operation_mode)
    {
    //classic selection
    case -1:
    {
      return treat_classic_selection(selection);
      break;
    }
    //Split vertex
    case 1:
    {
      static fg_halfedge_descriptor h1;
      //stores first fh and emit change label
      if(!d->first_selected)
      {
          bool found = false;
          //test preco
          BOOST_FOREACH(fg_halfedge_descriptor hafc, halfedges_around_face(halfedge(fh,*polyhedron()),*polyhedron()))
          {
            if(target(hafc,*polyhedron())==d->to_split_vh)
            {
              h1 = hafc;
              found = true;
              break;
            }
          }
          if(found)
          {
            d->first_selected = true;
            temp_selected_facets.insert(fh);
            invalidateOpenGLBuffers();
            Q_EMIT updateInstructions("Select the second facet. (3/3)");
          }
          else
            d->tempInstructions("Facet not selected : no valid halfedge",
                             "Select first facet. (2/3)");
      }
      //call the function with point and facets.
      else
      {
          //get the right halfedges
          fg_halfedge_descriptor h2;
          bool found = false;
          BOOST_FOREACH(fg_halfedge_descriptor hafc, halfedges_around_face(halfedge(fh,*polyhedron()),*polyhedron()))
          {
            if(target(hafc,*polyhedron())==d->to_split_vh)
            {
              h2 = hafc;
              found = true;
              break;
            }
          }

          if(found &&(h1 != h2))
          {
            fg_halfedge_descriptor hhandle = CGAL::Euler::split_vertex(h1,h2,*polyhedron());

            temp_selected_facets.clear();
            Point_3 p1t = get(vpm, target(h1,*polyhedron()));
            Point_3 p1s = get(vpm, target(opposite(h1,*polyhedron()),*polyhedron()));
            double x =  p1t.x() + 0.01 * (p1s.x() - p1t.x());
            double y =  p1t.y() + 0.01 * (p1s.y() - p1t.y());
            double z =  p1t.z() + 0.01 * (p1s.z() - p1t.z());
            put(vpm, target(opposite(hhandle,*polyhedron()),*polyhedron()), Point_3(x,y,z));;
            d->first_selected = false;
            temp_selected_vertices.clear();
            compute_normal_maps();
            invalidateOpenGLBuffers();
            //reset selection mode
            set_active_handle_type(static_cast<Active_handle::Type>(0));
            poly_item->invalidateOpenGLBuffers();
            d->tempInstructions("Vertex splitted.", "Select the vertex you want splitted. (1/3)");
          }
          else if(h1 == h2)
          {
             d->tempInstructions("Facet not selected : same as the first.", "Select the second facet. (3/3)");
          }
          else
          {
            d->tempInstructions("Facet not selected : no valid halfedge.", "Select the second facet. (3/3)");
          }
      }
      break;
    }
      //Split face
    case 4:
      if(is_triangle(halfedge(fh,*d->poly), *d->poly))
      {
        d->tempInstructions("Facet not selected : Facet must not be a triangle.",
                         "Select the facet you want to split (degree >= 4). (1/3)");
      }
      else
      {
        d->to_split_fh = fh;
        temp_selected_facets.insert(d->to_split_fh);
        compute_normal_maps();
        invalidateOpenGLBuffers();
        //set to select vertex
        set_active_handle_type(static_cast<Active_handle::Type>(0));
        Q_EMIT updateInstructions("Select first vertex. (2/3)");
      }
      break;
      //Add center vertex
    case 7:
      if(is_border(halfedge(fh,*polyhedron()),*polyhedron()))
        {
          d->tempInstructions("Facet not selected : Facet must not be null.",
                           "Select a Facet. (1/3)");
        }
        else
        {
          double x(0), y(0), z(0);
          int total(0);

          BOOST_FOREACH(fg_halfedge_descriptor hafc, halfedges_around_face(halfedge(fh,*polyhedron()),*polyhedron()))
          {
            fg_vertex_descriptor vd = target(hafc,*polyhedron());
            Point_3& p = get(vpm,vd);
            x+= p.x(); y+=p.y(); z+=p.z();
            total++;
          }
          fg_halfedge_descriptor hhandle = CGAL::Euler::add_center_vertex(halfedge(fh,*polyhedron()), *polyhedron());
          if(total !=0)
            put(vpm, target(hhandle,*polyhedron()), Point_3(x/(double)total, y/(double)total, z/(double)total));
          compute_normal_maps();
          poly_item->invalidateOpenGLBuffers();

        }
      break;
    }
  }
  d->is_treated = true;
  //Keeps the item from trying to draw primitive that has just been deleted.
  clearHL();
  return false;
}

void Scene_polyhedron_selection_item_priv::tempInstructions(QString s1, QString s2)
{
  m_temp_instructs = s2;
  Q_EMIT item->updateInstructions(QString("<font color='red'>%1</font>").arg(s1));
  QTimer timer;
  timer.singleShot(5500, item, SLOT(emitTempInstruct()));
}
void Scene_polyhedron_selection_item::emitTempInstruct()
{
  Q_EMIT updateInstructions(QString("<font color='black'>%1</font>").arg(d->m_temp_instructs));
}

/// An exception used while catching a throw that stops Dijkstra's algorithm
/// once the shortest path to a target has been found.
class Dijkstra_end_exception : public std::exception
{
  const char* what() const throw ()
  {
    return "Dijkstra shortest path: reached the target vertex.";
  }
};

/// Visitor to stop Dijkstra's algorithm once the given target turns 'BLACK',
/// that is when the target has been examined through all its incident edges and
/// the shortest path is thus known.
class Stop_at_target_Dijkstra_visitor : boost::default_dijkstra_visitor
{
  fg_vertex_descriptor destination_vd;

public:
  Stop_at_target_Dijkstra_visitor(fg_vertex_descriptor destination_vd)
    : destination_vd(destination_vd)
  { }

  void initialize_vertex(const fg_vertex_descriptor& /*s*/, const Face_graph& /*mesh*/) const { }
  void examine_vertex(const fg_vertex_descriptor& /*s*/, const Face_graph& /*mesh*/) const { }
  void examine_edge(const fg_edge_descriptor& /*e*/, const Face_graph& /*mesh*/) const { }
  void edge_relaxed(const fg_edge_descriptor& /*e*/, const Face_graph& /*mesh*/) const { }
  void discover_vertex(const fg_vertex_descriptor& /*s*/, const Face_graph& /*mesh*/) const { }
  void edge_not_relaxed(const fg_edge_descriptor& /*e*/, const Face_graph& /*mesh*/) const { }
  void finish_vertex(const fg_vertex_descriptor &vd, const Face_graph& /* mesh*/) const
  {
    if(vd == destination_vd)
      throw Dijkstra_end_exception();
  }
};

void Scene_polyhedron_selection_item_priv::computeAndDisplayPath()
{
  item->temp_selected_edges.clear();
  path.clear();

  typedef boost::unordered_map<fg_vertex_descriptor, fg_vertex_descriptor>     Pred_umap;
  typedef boost::associative_property_map<Pred_umap>                     Pred_pmap;

  Pred_umap predecessor;
  Pred_pmap pred_pmap(predecessor);

  vertex_on_path vop;
  QList<fg_vertex_descriptor>::iterator it;
  for(it = constrained_vertices.begin(); it!=constrained_vertices.end()-1; ++it)
  {
    fg_vertex_descriptor t(*it), s(*(it+1));
    Stop_at_target_Dijkstra_visitor vis(t);

    try
    {
      boost::dijkstra_shortest_paths(*item->polyhedron(), s,
                                     boost::predecessor_map(pred_pmap).visitor(vis));
    }
    catch (const std::exception& e)
    {
      std::cout << e.what() << std::endl;
    }

    // Walk back from target to source and collect vertices along the way
    do
    {
      vop.vertex = t;
      if(constrained_vertices.contains(t))
      {
        vop.is_constrained = true;
      }
      else
        vop.is_constrained = false;
      path.append(vop);
      t = get(pred_pmap, t);
    }
    while(t != s);
  }

  // Add the last vertex
  vop.vertex = constrained_vertices.last();
  vop.is_constrained = true;
  path.append(vop);

  // Display path
  double path_length = 0;
  QList<vertex_on_path>::iterator path_it;
  for(path_it = path.begin(); path_it!=path.end()-1; ++path_it)
  {
    std::pair<fg_halfedge_descriptor, bool> h = halfedge((path_it+1)->vertex,path_it->vertex,*item->polyhedron());
    if(h.second)
    {
      VPmap vpm = get(CGAL::vertex_point,*polyhedron());
      Point p1(get(vpm, (path_it+1)->vertex)), p2(get(vpm, path_it->vertex));
          path_length += CGAL::sqrt(Vector(p1,p2).squared_length());
      item->temp_selected_edges.insert(edge(h.first, *item->polyhedron()));
    }
  }
  item->printMessage(QString("New path length: %1").arg(path_length));
}

void Scene_polyhedron_selection_item_priv::addVertexToPath(fg_vertex_descriptor vh, vertex_on_path &first)
{
  vertex_on_path source;
  source.vertex = vh;
  source.is_constrained = true;
  path.append(source);
  first = source;
}
void Scene_polyhedron_selection_item::selectPath(fg_vertex_descriptor vh)
{

  bool replace = !temp_selected_edges.empty();
  static Scene_polyhedron_selection_item_priv::vertex_on_path first;
  if(!d->first_selected)
  {
    //if the path doesnt exist, add the vertex as the source of the path.
    if(!replace)
    {
      d->addVertexToPath(vh, first);
    }
    //if the path exists, get the vertex_on_path corresponding to the selected vertex.
    else
    {
      //The first vertex of the path can not be moved, but you can close your path on it to make a loop.
      bool alone = true;
      QList<Scene_polyhedron_selection_item_priv::vertex_on_path>::iterator it;
      for(it = d->path.begin(); it!=d->path.end(); ++it)
      {
        if(it->vertex == vh&& it!=d->path.begin())
          alone = false;
      }
      if(d->path.begin()->vertex == vh )
        if(alone)
        {
          d->constrained_vertices.append(vh); //if the path loops, the indexOf may be invalid, hence the check.
          //Display the new path
          d->computeAndDisplayPath();
          d->first_selected = false;
          d->constrained_vertices.clear();
          fixed_vertices.clear();
          for(it = d->path.begin(); it!=d->path.end(); ++it)
          {
            if(it->is_constrained )
            {
              d->constrained_vertices.append(it->vertex);
              fixed_vertices.insert(it->vertex);
            }
          }

          return;
        }
      bool found = false;
      Q_FOREACH(Scene_polyhedron_selection_item_priv::vertex_on_path vop, d->path)
      {
        if(vop.vertex == vh)
        {
          first = vop;
          found = true;
          break;
        }
      }
      if(!found)//add new end_point;
      {
        d->constrained_vertices.append(vh);
        //Display the new path
        d->computeAndDisplayPath();
        d->first_selected = false;
        d->constrained_vertices.clear();
        fixed_vertices.clear();
        for(it = d->path.begin(); it!=d->path.end(); ++it)
        {
          if(it->is_constrained )
          {
            d->constrained_vertices.append(it->vertex);
            fixed_vertices.insert(it->vertex);
          }
        }

        return;
      }
    }
    temp_selected_vertices.insert(vh);
    d->first_selected = true;
  }
  else
  {
    if(!replace)
    {
      d->constrained_vertices.append(vh);
      temp_selected_vertices.erase(first.vertex);

      updateInstructions("You can select a vertex on the green path to move it. "
                         "If you do so, it will become a red fixed point. "
                         "The path will be recomputed to go through that point. "
                         "Click on 'Add to selection' to validate the selection.   (2/2)");
    }
    else
    {
      bool is_same(false), alone(true);
      if( (vh == d->constrained_vertices.first() && first.vertex == d->constrained_vertices.last())
          || (vh == d->constrained_vertices.last() && first.vertex == d->constrained_vertices.first()))

      {
        is_same = true;
      }
      if(first.vertex == d->path.begin()->vertex)
        alone =false;
      bool is_last = true;
      //find the previous constrained vertex on path
      Scene_polyhedron_selection_item_priv::vertex_on_path closest = d->path.last();
      QList<Scene_polyhedron_selection_item_priv::vertex_on_path>::iterator it;
      int index = 0;
      int closest_index = 0;
      //get first's index
      for(it = d->path.begin(); it!=d->path.end(); ++it)
      {
        bool end_of_path_is_prio = true;//makes the end of the path prioritary over the other points when there is a conflict
        if(first.vertex == (d->path.end()-1)->vertex)
          if(it != d->path.end()-1)
            end_of_path_is_prio = false;
        //makes the end of the path prioritary over the other points when there is a conflict
        if(it->vertex == first.vertex &&
           !(it == d->path.begin())&&// makes the begining of the path impossible to move
           end_of_path_is_prio)
        {
          if(it!=d->path.end()-1 &&! is_same )
          {
            d->constrained_vertices.removeAll(it->vertex);
            if(!alone)
              d->constrained_vertices.prepend(it->vertex);
          }
          d->path.erase(it);
          break;
        }
        if(it->is_constrained)
          closest_index++;
        index++;
      }
      //get first constrained vertex following first in path
      for(it = d->path.begin() + index; it!=d->path.end(); ++it)
      {
        if(it->is_constrained )
        {
          is_last = false;
          closest = *it;
          break;
        }
      }
      //mark the new vertex as constrained before closest.
      temp_selected_vertices.erase(first.vertex);
      //check if the vertex is contained several times in the path
      if(!is_last)
      {
        d->constrained_vertices.insert(closest_index, vh);//cannot really use indexOf in case a fixed_point is used several times
      }
      else
        d->constrained_vertices.replace(d->constrained_vertices.size()-1, vh);


    }
    //Display the new path
    d->computeAndDisplayPath();
    d->first_selected = false;
  }
  //update constrained_vertices
  d->constrained_vertices.clear();
  fixed_vertices.clear();
  QList<Scene_polyhedron_selection_item_priv::vertex_on_path>::iterator it;
  for(it = d->path.begin(); it!=d->path.end(); ++it)
  {
    if(it->is_constrained )
    {
      d->constrained_vertices.append(it->vertex);
      fixed_vertices.insert(it->vertex);
    }
  }
}


void Scene_polyhedron_selection_item::on_Ctrlz_pressed()
{
  d->path.clear();
  d->constrained_vertices.clear();
  fixed_vertices.clear();
  validateMoveVertex();
  d->first_selected = false;
  temp_selected_vertices.clear();
  temp_selected_edges.clear();
  temp_selected_facets.clear();
  d->are_temp_buffers_filled = false;
  set_operation_mode(d->operation_mode);
  Q_EMIT itemChanged();
}

Scene_polyhedron_selection_item::Scene_polyhedron_selection_item()
  : Scene_polyhedron_item_decorator(NULL, false)
{

  d = new Scene_polyhedron_selection_item_priv(this);
  d->original_sel_mode = static_cast<Active_handle::Type>(0);
  d->operation_mode = -1;
  QGLViewer::QGLViewerPool().first()->makeCurrent();
  for(int i=0; i<Scene_polyhedron_selection_item_priv::NumberOfVaos; i++)
  {
    addVaos(i);
    vaos[i]->create();
  }

  for(int i=0; i<Scene_polyhedron_selection_item_priv::NumberOfVbos; i++)
  {
    buffers[i].create();
  }
  d->nb_facets = 0;
  d->nb_points = 0;
  d->nb_lines = 0;
  this->setColor(QColor(87,87,87));
  d->first_selected = false;
  d->is_treated = false;
  d->poly_need_update = false;
  d->are_temp_buffers_filled = false;
  d->poly = NULL;
  d->ready_to_move = false;
}

Scene_polyhedron_selection_item::Scene_polyhedron_selection_item(Scene_face_graph_item* poly_item, QMainWindow* mw)
  : Scene_polyhedron_item_decorator(NULL, false)
{
  d = new Scene_polyhedron_selection_item_priv(this);
  d->original_sel_mode = static_cast<Active_handle::Type>(0);
  d->operation_mode = -1;
  d->nb_facets = 0;
  d->nb_points = 0;
  d->nb_lines = 0;

  for(int i=0; i<Scene_polyhedron_selection_item_priv::NumberOfVaos; i++)
  {
    addVaos(i);
    vaos[i]->create();
  }

  for(int i=0; i<Scene_polyhedron_selection_item_priv::NumberOfVbos; i++)
  {
    buffers[i].create();
  }
  d->poly = NULL;
  init(poly_item, mw);
  this->setColor(QColor(87,87,87));
  invalidateOpenGLBuffers();
  compute_normal_maps();
  d->first_selected = false;
  d->is_treated = false;
  d->poly_need_update = false;
  d->ready_to_move = false;

}

Scene_polyhedron_selection_item::~Scene_polyhedron_selection_item()
{
  delete d;
  QGLViewer* v = *QGLViewer::QGLViewerPool().begin();
  CGAL::Three::Viewer_interface* viewer = dynamic_cast<CGAL::Three::Viewer_interface*>(v);
  viewer->setBindingSelect();
}

void Scene_polyhedron_selection_item::setPathSelection(bool b) {
  k_ring_selector.setEditMode(b);
  d->is_path_selecting = b;
  if(d->is_path_selecting){
    int ind = 0;
    boost::property_map<Face_graph,CGAL::vertex_selection_t>::type vsm =
      get(CGAL::vertex_selection,*polyhedron());
    BOOST_FOREACH(fg_vertex_descriptor vd, vertices(*polyhedron())){
      put(vsm,vd, ind++);
    }
  }
}

void Scene_polyhedron_selection_item::update_poly()
{
  if(d->poly_need_update)
    poly_item->invalidateOpenGLBuffers();
}

void Scene_polyhedron_selection_item::resetIsTreated() { d->is_treated = false;}

void Scene_polyhedron_selection_item::invalidateOpenGLBuffers() {

  // do not use decorator function, which calls changed on poly_item which cause deletion of AABB
    //  poly_item->invalidateOpenGLBuffers();
      are_buffers_filled = false;
      d->are_temp_buffers_filled = false;
      d->poly = polyhedron();
      compute_bbox();
}

void Scene_polyhedron_selection_item::add_to_selection()
{
  Q_FOREACH(fg_edge_descriptor ed, temp_selected_edges)
  {
    selected_edges.insert(ed);
    temp_selected_edges.erase(ed);
  }
  on_Ctrlz_pressed();
  invalidateOpenGLBuffers();
  QGLViewer* v = *QGLViewer::QGLViewerPool().begin();
  v->update();
  d->tempInstructions("Path added to selection.",
                   "Select two vertices to create the path between them. (1/2)");
}

void Scene_polyhedron_selection_item::save_handleType()
{
  d->original_sel_mode = get_active_handle_type();
}
void Scene_polyhedron_selection_item::compute_normal_maps()
{

  d->face_normals_map.clear();
  d->vertex_normals_map.clear();
  d->nf_pmap = boost::associative_property_map< CGAL::Unique_hash_map<fg_face_descriptor, Kernel::Vector_3> >(d->face_normals_map);
  d->nv_pmap = boost::associative_property_map< CGAL::Unique_hash_map<fg_vertex_descriptor, Kernel::Vector_3> >(d->vertex_normals_map);
  PMP::compute_normals(*d->poly, d->nv_pmap, d->nf_pmap);
}

void Scene_polyhedron_selection_item::updateTick()
{
    d->ready_to_move = true;
    QTimer::singleShot(0,this,SLOT(moveVertex()));
}


void Scene_polyhedron_selection_item::moveVertex()
{
  if(d->ready_to_move)
  {
     const qglviewer::Vec offset = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first())->offset();
    fg_vertex_descriptor vh = *temp_selected_vertices.begin();

    VPmap vpm = get(CGAL::vertex_point,*polyhedron());
    put(vpm, vh, Point_3(d->manipulated_frame->position().x-offset.x,
                         d->manipulated_frame->position().y-offset.y,
                         d->manipulated_frame->position().z-offset.z));
    invalidateOpenGLBuffers();
    poly_item->invalidateOpenGLBuffers();
    d->ready_to_move = false;
  }
}

void Scene_polyhedron_selection_item::validateMoveVertex()
{
  temp_selected_vertices.clear();
  QGLViewer* viewer = *QGLViewer::QGLViewerPool().begin();
  k_ring_selector.setEditMode(true);
  viewer->setManipulatedFrame(NULL);
  invalidateOpenGLBuffers();
  Q_EMIT updateInstructions("Select a vertex. (1/2)");
}


bool Scene_polyhedron_selection_item_priv::canAddFace(fg_halfedge_descriptor hc, fg_halfedge_descriptor t)
{
  bool found(false),  is_border_h(false);

  //if the selected halfedge is not a border, stop and signal it.
  if(is_border(hc,*polyhedron()))
    is_border_h = true;
  else if(is_border(opposite(hc,*polyhedron()),*polyhedron()))
  {
    hc = opposite(hc,*polyhedron());
    is_border_h = true;
  }
  if(!is_border_h)
  {
    tempInstructions("Edge not selected : no shared border found.",
                     "Select the second edge. (2/2)");
    return false;
  }
  //if the halfedges are the same, stop and signal it.
  if(hc == t)
  {
    tempInstructions("Edge not selected : halfedges must be different.",
                     "Select the second edge. (2/2)");
    return false;
  }
  //if the halfedges are adjacent, stop and signal it.
  if(next(t, *item->polyhedron()) == hc || next(hc, *item->polyhedron()) == t)
  {
    tempInstructions("Edge not selected : halfedges must not be adjacent.",
                     "Select the second edge. (2/2)");
    return false;
  }

  //if the halfedges are not on the same border, stop and signal it.
  fg_halfedge_descriptor iterator = next(t, *item->polyhedron());
  while(iterator != t)
  {
    if(iterator == hc)
    {
      found = true;
      fg_halfedge_descriptor res =
          CGAL::Euler::add_face_to_border(t,hc, *item->polyhedron());

      if(CGAL::is_degenerate_triangle_face(res, *item->polyhedron(), get(CGAL::vertex_point, *item->polyhedron()), Kernel()))
      {
        CGAL::Euler::remove_face(res, *item->polyhedron());
        tempInstructions("Edge not selected : resulting facet is degenerated.",
                         "Select the second edge. (2/2)");
        return false;
      }
      break;
    }
    iterator = next(iterator, *item->polyhedron());
  }
  if(!found)
  {
    tempInstructions("Edge not selected : no shared border found.",
                     "Select the second edge. (2/2)");
    return false;
  }
  return true;
}

bool Scene_polyhedron_selection_item_priv::canAddFaceAndVertex(fg_halfedge_descriptor hc, fg_halfedge_descriptor t)
{
  bool found(false),  is_border_h(false);

  //if the selected halfedge is not a border, stop and signal it.
  if(is_border(hc,*polyhedron()))
    is_border_h = true;
  else if(is_border(opposite(hc,*polyhedron()),*polyhedron()))
  {
    hc = opposite(hc,*polyhedron());
    is_border_h = true;
  }
  if(!is_border_h)
  {
    tempInstructions("Edge not selected : no shared border found.",
                     "Select the second edge. (2/2)");
    return false;
  }
  //if the halfedges are the same, stop and signal it.
  if(hc == t)
  {
    tempInstructions("Edge not selected : halfedges must be different.",
                     "Select the second edge. (2/2)");
    return false;
  }

  //if the halfedges are not on the same border, stop and signal it.
  fg_halfedge_descriptor iterator = next(t, *item->polyhedron());
  while(iterator != t)
  {
    if(iterator == hc)
    {
      found = true;
      CGAL::Euler::add_vertex_and_face_to_border(t,hc, *item->polyhedron());
      break;
    }
    iterator = next(iterator, *item->polyhedron());
  }
  if(!found)
  {
    tempInstructions("Edge not selected : no shared border found.",
                     "Select the second edge. (2/2)");
    return false;
  }
  return true;
}

void Scene_polyhedron_selection_item::clearHL()
{
  HL_selected_edges.clear();
  HL_selected_facets.clear();
  HL_selected_vertices.clear();
  d->are_HL_buffers_filled = false;
  Q_EMIT itemChanged();
}
void Scene_polyhedron_selection_item::selected_HL(const std::set<fg_vertex_descriptor>& m)
{
//  HL_selected_edges.clear();
  HL_selected_facets.clear();
  HL_selected_vertices.clear();
  HL_selected_vertices.insert(*m.begin());

  d->are_HL_buffers_filled = false;
  Q_EMIT itemChanged();
}

void Scene_polyhedron_selection_item::selected_HL(const std::set<fg_face_descriptor>& m)
{
  HL_selected_edges.clear();
  HL_selected_facets.clear();
  HL_selected_vertices.clear();
  HL_selected_facets.insert(*m.begin());
  d->are_HL_buffers_filled = false;
  Q_EMIT itemChanged();
}

void Scene_polyhedron_selection_item::selected_HL(const std::set<fg_edge_descriptor>& m)
{
  HL_selected_edges.clear();
  HL_selected_facets.clear();
  HL_selected_vertices.clear();
  HL_selected_edges.insert(*m.begin());
  d->are_HL_buffers_filled = false;
  Q_EMIT itemChanged();
}

void Scene_polyhedron_selection_item::init(Scene_face_graph_item* poly_item, QMainWindow* mw)
{
  this->poly_item = poly_item;
  d->poly =poly_item->polyhedron();
  connect(poly_item, SIGNAL(item_is_about_to_be_changed()), this, SLOT(poly_item_changed()));
  //parameters type must be of the same name here and there, so they must be hardcoded.
  connect(&k_ring_selector, SIGNAL(selected(const std::set<fg_vertex_descriptor>&)), this,
    SLOT(selected(const std::set<fg_vertex_descriptor>&)));

  connect(&k_ring_selector, SIGNAL(selected(const std::set<fg_face_descriptor>&)), this,
    SLOT(selected(const std::set<fg_face_descriptor>&)));

  connect(&k_ring_selector, SIGNAL(selected(const std::set<fg_edge_descriptor>&)), this,
    SLOT(selected(const std::set<fg_edge_descriptor>&)));

  connect(&k_ring_selector, SIGNAL(selected_HL(const std::set<fg_vertex_descriptor>&)), this,
          SLOT(selected_HL(const std::set<fg_vertex_descriptor>&)));

  connect(&k_ring_selector, SIGNAL(selected_HL(const std::set<fg_face_descriptor>&)), this,
          SLOT(selected_HL(const std::set<fg_face_descriptor>&)));

  connect(&k_ring_selector, SIGNAL(selected_HL(const std::set<fg_edge_descriptor>&)), this,
          SLOT(selected_HL(const std::set<fg_edge_descriptor>&)));
  connect(&k_ring_selector, SIGNAL(clearHL()), this,
          SLOT(clearHL()));
  connect(poly_item, SIGNAL(selection_done()), this, SLOT(update_poly()));
  connect(&k_ring_selector, SIGNAL(endSelection()), this,SLOT(endSelection()));
  connect(&k_ring_selector, SIGNAL(toogle_insert(bool)), this,SLOT(toggle_insert(bool)));
  connect(&k_ring_selector,SIGNAL(isCurrentlySelected(Scene_facegraph_item_k_ring_selection*)), this, SIGNAL(isCurrentlySelected(Scene_facegraph_item_k_ring_selection*)));
   k_ring_selector.init(poly_item, mw, Active_handle::VERTEX, -1);
  connect(&k_ring_selector, SIGNAL(resetIsTreated()), this, SLOT(resetIsTreated()));
  QGLViewer* viewer = *QGLViewer::QGLViewerPool().begin();
  d->manipulated_frame = new ManipulatedFrame();
  viewer->installEventFilter(this);
  mw->installEventFilter(this);
}

void Scene_polyhedron_selection_item::select_all_NT()
{
  BOOST_FOREACH(fg_face_descriptor fd, faces(*polyhedron())){
    if(! is_triangle(halfedge(fd,*polyhedron()), *polyhedron()))
    selected_facets.insert(fd);
  }
  invalidateOpenGLBuffers();
  Q_EMIT itemChanged();
}

void Scene_polyhedron_selection_item::selection_changed(bool b)
{
  QGLViewer* v = *QGLViewer::QGLViewerPool().begin();
  CGAL::Three::Viewer_interface* viewer = dynamic_cast<CGAL::Three::Viewer_interface*>(v);
  if(!viewer)
      return;

  if(!b)
  {
    viewer->setBindingSelect();
  }
  else
  {
    viewer->setNoBinding();
  }
}

void Scene_polyhedron_selection_item::printPrimitiveId(QPoint p, CGAL::Three::Viewer_interface* viewer)
{
  d->item->polyhedron_item()->printPrimitiveId(p, viewer);
}
bool Scene_polyhedron_selection_item::printVertexIds(CGAL::Three::Viewer_interface* viewer) const
{
  return d->item->polyhedron_item()->printVertexIds(viewer);
  return false;
}
bool Scene_polyhedron_selection_item::printEdgeIds(CGAL::Three::Viewer_interface* viewer) const
{
  d->item->polyhedron_item()->printEdgeIds(viewer);
  return false;
}
bool Scene_polyhedron_selection_item::printFaceIds(CGAL::Three::Viewer_interface* viewer) const
{
  return d->item->polyhedron_item()->printFaceIds(viewer);
  return false;
}
void Scene_polyhedron_selection_item::printAllIds(CGAL::Three::Viewer_interface* viewer)
{
  d->item->polyhedron_item()->printAllIds(viewer);
}
bool Scene_polyhedron_selection_item::testDisplayId(double x, double y, double z, CGAL::Three::Viewer_interface* viewer)const
{
  return d->item->polyhedron_item()->testDisplayId(x, y, z, viewer);
  return false;
}

bool Scene_polyhedron_selection_item::shouldDisplayIds(CGAL::Three::Scene_item *current_item) const
{
  return d->item->polyhedron_item() == current_item;
  return false;
}

void Scene_polyhedron_selection_item::select_boundary()
{
  Face_graph* fg = polyhedron_item()->face_graph();
  BOOST_FOREACH(fg_halfedge_descriptor hd, halfedges(*fg))
  {
    if(is_border_edge(hd, *fg))
    {
      selected_edges.insert(edge(hd, *fg));
    }
  }
  invalidateOpenGLBuffers();
  redraw();
}

QString 
Scene_polyhedron_selection_item::toolTip() const
{
  if(!poly_item->polyhedron())
    return QString();

  return QObject::tr("<p>Selection <b>%1</b> (mode: %5, color: %6)</p>"
                     "<p>Number of vertices: %2<br />"
                     "Number of edges: %3<br />"
                     "Number of faces: %4</p>")
    .arg(this->name())
    .arg(selected_vertices.size())
    .arg(selected_edges.size())
    .arg(selected_facets.size())
    .arg(this->renderingModeName())
    .arg(this->color().name());
}
