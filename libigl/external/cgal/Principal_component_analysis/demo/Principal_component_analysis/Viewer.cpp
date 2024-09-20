#include "Viewer.h"
#include "Scene.h"

Viewer::Viewer(QWidget* parent)
  : QGLViewer(parent),
    m_pScene(NULL)
{
}

void Viewer::setScene(Scene* pScene)
{
  this->m_pScene = pScene;
}

void Viewer::draw()
{
  QGLViewer::draw();
  if(m_pScene != NULL)
  {
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        m_pScene->draw(this);
  }
}

void Viewer::initializeGL()
{
  QGLViewer::initializeGL();
  makeCurrent();
  initializeOpenGLFunctions();
  setBackgroundColor(::Qt::white);
}

