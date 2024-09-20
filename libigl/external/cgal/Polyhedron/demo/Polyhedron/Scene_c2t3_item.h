#ifndef SCENE_C2T3_ITEM_H
#define SCENE_C2T3_ITEM_H

#include "Scene_c2t3_item_config.h"
#include "C2t3_type.h"
#include <iostream>
#include  <CGAL/Three/Scene_item.h>
#include <qgl.h>
#include <QtCore/qglobal.h>

class SCENE_C2T3_ITEM_EXPORT Scene_c2t3_item : public CGAL::Three::Scene_item
{
  Q_OBJECT
public:
  Scene_c2t3_item(const C2t3& c2t3)
    : c2t3_(c2t3)
  {
  }

  ~Scene_c2t3_item()
  {
  }

  C2t3& c2t3() {
    return c2t3_;
  }

  const C2t3& c2t3() const {
    return c2t3_;
  }
  bool isFinite() const { return true; }
  bool isEmpty() const {
    return c2t3().triangulation().number_of_vertices() == 0;
  }

  void compute_bbox() const {
    if(isEmpty())
      _bbox =  Bbox();
    else {
      bool first = true;
      CGAL::Bbox_3 result;
      for(Tr::Finite_vertices_iterator
            vit = ++c2t3().triangulation().finite_vertices_begin(),
            end = c2t3().triangulation().finite_vertices_end();
          vit != end; ++vit)
      {
        if(first) {
          result = vit->point().bbox();
          first = false;
        } else { 
          result = result + vit->point().bbox();
        }
      }
      _bbox = Bbox(result.xmin(), result.ymin(), result.zmin(),
                  result.xmax(), result.ymax(), result.zmax());
    }
  }


  Scene_c2t3_item* clone() const {
    return 0;
  }

  QString toolTip() const {
    return tr("<p><b>2D complex in a 3D triangulation</b></p>"
              "<p>Number of vertices: %1<br />"
              "Number of surface facets: %2<br />")
      .arg(c2t3().triangulation().number_of_vertices())
      .arg(c2t3().number_of_facets());
  }

  // Indicate if rendering mode is supported
  bool supportsRenderingMode(RenderingMode m) const {
    return (m != Gouraud && m!=PointsPlusNormals && m!=ShadedPoints); // CHECK THIS!
  }

  void draw() const {
   /* ::glBegin(GL_TRIANGLES);
    for(C2t3::Facet_iterator
          fit = c2t3().facets_begin(),
          end = c2t3().facets_end();
        fit != end; ++fit)
    {
      const Tr::Cell_handle& cell = fit->first;
      const int& index = fit->second;
      const Tr::Geom_traits::Point_3& pa = cell->vertex((index+1)&3)->point();
      const Tr::Geom_traits::Point_3& pb = cell->vertex((index+2)&3)->point();
      const Tr::Geom_traits::Point_3& pc = cell->vertex((index+3)&3)->point();
      draw_triangle(pa, pb, pc);
    }
    ::glEnd();*/
  }

private:
  static void draw_triangle(const Tr::Point& pa,
                            const Tr::Point& pb,
                            const Tr::Point& pc) {
    Tr::Geom_traits::Vector_3 n = cross_product(pb - pa, pc -pa);
    n = n / CGAL::sqrt(n*n);
    ::glNormal3d(n.x(),n.y(),n.z());

    ::glVertex3d(pa.x(),pa.y(),pa.z());
    ::glVertex3d(pb.x(),pb.y(),pb.z());
    ::glVertex3d(pc.x(),pc.y(),pc.z());
  }

private:
  C2t3 c2t3_;
};

#endif // SCENE_C2T3_ITEM
