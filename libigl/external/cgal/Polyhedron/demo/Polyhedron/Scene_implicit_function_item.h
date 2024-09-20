#ifndef SCENE_IMPLICIT_FUNCTION_ITEM_H
#define SCENE_IMPLICIT_FUNCTION_ITEM_H

#include <CGAL/Three/Scene_item.h>
#include <CGAL/Three/Scene_interface.h>
#include "Scene_implicit_function_item_config.h"
#include "implicit_functions/Implicit_function_interface.h"
#include "Color_ramp.h"
#include <QGLViewer/manipulatedFrame.h>
#include <Viewer.h>

#define SCENE_IMPLICIT_GRID_SIZE 120

class Viewer_interface;
struct Scene_implicit_function_item_priv;
class Texture{
private:
     int Width;
     int Height;
     int size;
    GLubyte *data;
public:
    Texture(int w, int h)
    {
        Width = w;
        Height = h;
        size = 3*Height*Width;
        data = new GLubyte[size];
    }
    int getWidth() const {return Width;}
    int getHeight() const {return Height;}
    int getSize() const {return size;}
    void setData(int i, int j, int r, int g, int b){
      data[j*Width*3 +i*3] = GLubyte(r);
      data[j*Width*3 +i*3+1] = GLubyte(g);
      data[j*Width*3 +i*3+2] = GLubyte(b);}
    GLubyte* getData(){return data; }

};
class SCENE_IMPLICIT_FUNCTION_ITEM_EXPORT Scene_implicit_function_item 
  : public CGAL::Three::Scene_item
{
  Q_OBJECT
  
  typedef qglviewer::ManipulatedFrame ManipulatedFrame;
  
public:
  Scene_implicit_function_item(Implicit_function_interface*);
  virtual ~Scene_implicit_function_item();
  
  Implicit_function_interface* function() const ;

  bool isFinite() const { return true; }
  bool isEmpty() const { return false; }
  void compute_bbox() const;

  Scene_implicit_function_item* clone() const { return NULL; }

  // rendering mode
  virtual bool supportsRenderingMode(RenderingMode m) const;
  virtual bool manipulatable() const { return true; }
  virtual ManipulatedFrame* manipulatedFrame();
  


  // actually draw() is also overloaded to detect when the cut plane is moved
  virtual void draw()const {}
  virtual void draw(CGAL::Three::Viewer_interface*) const;
  virtual void drawEdges(CGAL::Three::Viewer_interface*) const;

  virtual QString toolTip() const;
  virtual void invalidateOpenGLBuffers();
public Q_SLOTS:
  void plane_was_moved();
  void compute_function_grid() const;
  void updateCutPlane();

protected:
  friend struct Scene_implicit_function_item_priv;
  Scene_implicit_function_item_priv* d;
};

#endif // SCENE_IMPLICIT_FUNCTION_ITEM
