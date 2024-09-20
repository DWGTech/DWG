#ifndef CGAL_NEF_VISUAL_HULL_CREATOR_H
#define CGAL_NEF_VISUAL_HULL_CREATOR_H

#include <CGAL/Object.h>
#include <CGAL/intersections.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/IO/Qt_widget_Nef_3.h>
#include <CGAL/Modifier_base.h>
#include <qapplication.h>

namespace CGAL {

template <typename SNC_>
class visual_hull_creator : public CGAL::Modifier_base<SNC_> {
  
  typedef SNC_  SNC_structure;
  typedef typename SNC_structure::Kernel Kernel;
  typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
  typedef CGAL::SNC_decorator<SNC_structure> SNC_decorator;
  typedef SNC_decorator Base;
  typedef typename SNC_decorator::SNC_constructor SNC_constructor;
  typedef typename SNC_structure::SM_decorator SM_decorator;

  typedef typename SNC_structure::Vertex_handle Vertex_handle;
  typedef typename SNC_structure::SVertex_iterator SVertex_iterator;
  typedef typename SNC_structure::Point_3 Point_3;
  typedef typename SNC_structure::Plane_3 Plane_3;
  typedef typename SNC_structure::Vector_3 Vector_3;
  typedef typename SNC_structure::Ray_3 Ray_3;
  typedef typename SNC_structure::Sphere_point Sphere_point;

  Point_3 room_min, room_max;
  Point_3 c_pos;
  Vertex_handle camera;
  Plane_3 cut;
  SNC_structure* sncp;
  std::list<std::list<Point_3> > polygon_list;   
    
 private:
  Plane_3 find_cutoff_plane() {
    
    bool compute_halfsphere[3][2];
    for(int i=0; i<6; i++)
      compute_halfsphere[i/2][i%2] = false;

    SVertex_iterator sv;
    SM_decorator SD(&*camera);
    CGAL_forall_svertices(sv,SD) {
      if(!compute_halfsphere[0][0])
	if(sv->point().hx()>0)
	  compute_halfsphere[0][0] = true;
      if(!compute_halfsphere[0][1])
	if(sv->point().hx()<0)
	  compute_halfsphere[0][1] = true;    
      if(!compute_halfsphere[1][0])
	if(sv->point().hy()>0)
	  compute_halfsphere[1][0] = true;
      if(!compute_halfsphere[1][1])
	if(sv->point().hy()<0)
	  compute_halfsphere[1][1] = true;    
      if(!compute_halfsphere[2][0])
	if(sv->point().hz()>0)
	  compute_halfsphere[2][0] = true;
      if(!compute_halfsphere[2][1])
	if(sv->point().hz()<0)
	  compute_halfsphere[2][1] = true;    
    }

    if(!compute_halfsphere[0][1])
      return Plane_3(Point_3(room_max.hx(),0,0,room_max.hw()),Vector_3(1,0,0));
    if(!compute_halfsphere[0][0])
      return Plane_3(Point_3(room_min.hx(),0,0,room_min.hw()),Vector_3(-1,0,0));
    if(!compute_halfsphere[1][1])
      return Plane_3(Point_3(0,room_min.hy(),0,room_max.hw()),Vector_3(0,1,0));
    if(!compute_halfsphere[1][0])
      return Plane_3(Point_3(0,room_min.hy(),0,room_min.hw()),Vector_3(0,-1,0));
    if(!compute_halfsphere[2][1])
      return Plane_3(Point_3(0,0,room_max.hz(),room_max.hw()),Vector_3(0,0,1));
    if(!compute_halfsphere[2][0])
      return Plane_3(Point_3(0,0,room_min.hz(),room_min.hw()),Vector_3(0,0,-1));

    CGAL_error_msg("wrong sphere map");
    return Plane_3();
  }


 public:
  visual_hull_creator(Point_3 min, Point_3 max, Point_3 position, 
		      std::list<std::list<Point_3> > p) : 
    room_min(min), room_max(max), c_pos(position), polygon_list(p) { }
    
    /*
  void recompute_scene() {
    std::cerr << "recompute scene" << std::endl;
    SNC_structure s_tmp(*sncp());
    SNC_constructor C(s_tmp);
    C.clear_external_structure();
    Nef_polyhedron tmp(s_tmp,new typename Nef_polyhedron::SNC_point_locator_default, false);
    tmp.build_external_structure();
    N_list.push_back(tmp);
  }
    */
    
  void operator()(SNC_structure& snc) {

    sncp = &snc;
    snc.clear();

    camera = sncp->new_vertex(c_pos);
    camera->mark() = true;

    typename std::list< std::list<Point_3> >::iterator li;
    for(li=polygon_list.begin(); li!=polygon_list.end(); ++li)
      if(li==polygon_list.begin()) {
	add_outer_cycle_to_camera(li->begin(), li->end());
      } else {
	add_inner_cycle_to_camera(li->begin(), li->end());
      }
    
    for(li=polygon_list.begin(); li!=polygon_list.end(); ++li)
      if(li==polygon_list.begin()) {
	create_outer_cycles_opposites(li->begin(), li->end());
      } else {
	create_inner_cycles_opposites(li->begin(), li->end());
      }
  }

  template<typename Forward_iterator>
  void add_outer_cycle_to_camera(Forward_iterator begin, Forward_iterator end) {
    
    SNC_constructor C(*sncp);
    std::list<Sphere_point> spoints;
    
    Forward_iterator si, si_prev, si_next;
    for(si=begin;si!=end;++si) {
      spoints.push_back(Sphere_point(*si-camera->point()));
    }

    C.add_outer_sedge_cycle(camera, spoints.begin(), spoints.end(),false);
  }

  template<typename Forward_iterator>
  void create_outer_cycles_opposites(Forward_iterator begin, Forward_iterator end) {
    
    SNC_constructor C(*sncp);
    std::list<Sphere_point> spoints;
    
    cut = find_cutoff_plane();
    std::list<Point_3> points_on_plane;
    Forward_iterator pi;
    for(pi=begin;pi!=end;++pi) {
      Ray_3 r(camera->point(), *pi-camera->point());
      CGAL::Object io=CGAL::intersection(r,cut);
      Point_3 ip;
      assign(ip,io);
      points_on_plane.push_back(ip);
    }

    typename std::list<Point_3>::iterator si,si_next,si_prev;
    si_next = points_on_plane.begin();
    si_prev = points_on_plane.end();
    --si_prev;
    for(si=points_on_plane.begin();si!=points_on_plane.end();++si) {
      ++si_next;
      if(si_next==points_on_plane.end()) si_next=points_on_plane.begin();
      spoints.clear();
      spoints.push_back(*si_prev-*si);
      spoints.push_back(*si_next-*si);
      spoints.push_back(camera->point()-*si);
      bool orient(CGAL::orientation(*si_prev, 
				    *si, 
				    *si_next,camera->point()) == CGAL::POSITIVE);
      C.add_outer_sedge_cycle(sncp->new_vertex(*si),spoints.begin(), spoints.end(),orient); 
      ++si_prev;
      if(si_prev==points_on_plane.end()) si_prev=points_on_plane.begin();
    }
  }

  template<typename Forward_iterator>
    void add_inner_cycle_to_camera(Forward_iterator begin, Forward_iterator end) {

    SNC_constructor C(*sncp);
    std::list<Sphere_point> spoints;
    
    Forward_iterator si, si_prev, si_next;
    for(si=begin;si!=end;++si) {
      spoints.push_back(Sphere_point(*si-camera->point()));
    }

    C.add_inner_sedge_cycle(camera, spoints.begin(), spoints.end(),false,true);
  }

  template<typename Forward_iterator>
    void create_inner_cycles_opposites(Forward_iterator begin, Forward_iterator end) {

    SNC_constructor C(*sncp);
    std::list<Sphere_point> spoints;
    
    //    Plane_3 cut = find_cutoff_plane();
    std::list<Point_3> points_on_plane;

    Forward_iterator pi;
    for(pi=begin;pi!=end;++pi) {
      Ray_3 r(camera->point(), *pi-camera->point());
      CGAL::Object io=CGAL::intersection(r,cut);
      Point_3 ip;
      assign(ip,io);
      points_on_plane.push_back(ip);
    }

    typename std::list<Point_3>::iterator si,si_next,si_prev;
    si_next = points_on_plane.begin();
    si_prev = points_on_plane.end();
    --si_prev;
    for(si=points_on_plane.begin();si!=points_on_plane.end();++si) {
      ++si_next;
      if(si_next==points_on_plane.end()) si_next=points_on_plane.begin();
      spoints.clear();
      spoints.push_back(*si_prev-*si);
      spoints.push_back(*si_next-*si);
      spoints.push_back(camera->point()-*si);
      bool orient(CGAL::orientation(*si_prev, 
				    *si, 
				    *si_next,camera->point()) == CGAL::POSITIVE);
      C.add_inner_sedge_cycle(sncp->new_vertex(*si),spoints.begin(), spoints.end(),orient,false);
      ++si_prev;
      if(si_prev==points_on_plane.end()) si_prev=points_on_plane.begin();
    }
  }

  /*
  void print_off() {
   
    if(result.is_simple()) {
      Polyhedron P;
      result.convert_to_Polyhedron(P);
      std::cout << P;
    } else
      std::cerr << "result is not simple" << std::endl;
    
    int argc = 1;
    char* argv="Visual Hull";
    QApplication a(argc,&argv);
    CGAL::Qt_widget_Nef_3<Nef_polyhedron>* w = 
      new CGAL::Qt_widget_Nef_3<Nef_polyhedron>(result);
    a.setMainWidget(w);
    w->show();
    a.exec();
  }
  */
};

} //namespace CGAL
#endif // CGAL_NEF_VISUAL_HULL_CREATOR_H
