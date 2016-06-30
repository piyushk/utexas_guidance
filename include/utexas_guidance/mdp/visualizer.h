#ifndef UTEXAS_GUIDANCE_VISUALIZER_H
#define UTEXAS_GUIDANCE_VISUALIZER_H

#include <QGLViewer/qglviewer.h>
#include <utexas_guidance/core/visualizer.h>

namespace utexas_guidance {

  class StateViewer : public QGLViewer {

    protected:

      virtual void init();
      virtual void draw();
  };

  class Visualizer : public utexas_planning::Visualizer {

    public:

      typedef boost::shared_ptr<Visualizer> Ptr;
      typedef boost::shared_ptr<const Visualizer> ConstPtr;

      virtual ~Visualizer();
      virtual void init(int argc, char* argv[]);
      virtual void startEpisode(const State::ConstPtr& start_state);
      virtual void updateState(const State::ConstPtr& state, float timeout = 0.0f);

    protected:

      StateViewer::Ptr viewer_;
  };
  
} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_VISUALIZER_H */
