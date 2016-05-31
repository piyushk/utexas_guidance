#include <iostream>
#include <qapplication.h>
#include <QGLViewer/qglviewer.h>
#include <utexas_guidance/graph/graph.h>

utexas_guidance::Graph graph_;

class GraphViewer : public QGLViewer {
  protected :
    virtual void draw();
    virtual void init();
};

// Draws a spiral
void GraphViewer::draw() {
  utexas_guidance::draw(graph_);
}

void GraphViewer::init() {
  // Restore previous viewer state.
  restoreStateFromFile();

  glDisable(GL_LIGHTING);
  glPointSize(10.0f);
  glLineWidth(3.0f);
  setGridIsDrawn(false);
}

int main(int argc, char *argv[]) {
  if (argc < 2 || argc > 2) {
    std::cerr << "Usage: " << argv[0] << " <graph-file>" << std::endl;
    return -1;
  }

  utexas_guidance::readGraphFromFile(std::string(argv[1]), graph_);

  QApplication application(argc, argv);

  GraphViewer viewer;

  viewer.setWindowTitle("out");

  viewer.show();

  return application.exec();
}
