#include <class_loader/class_loader.h>
#include <utexas_planning/visualizers/no_visualization.h>
#include <qapplication.h>

namespace utexas_guidance {

  Visualizer::~Visualizer() {}

  void Visualizer::init(int argc, char* argv[]) {
    QApplication application(argc, argv);
    viewer_.reset(new Viewer);
    viewer_->setWindowTitle("out");
    viewer_->show();
  }
  void Visualizer::startEpisode(const State::ConstPtr& /* start_state */) {}
  void Visualizer::updateState(const State::ConstPtr& /* state */, float /* timeout */) {}

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_guidance::Visualizer, utexas_planning::Visualizer);

#include <iostream>
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



  return application.exec();
}

