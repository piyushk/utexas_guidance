#ifndef UTEXAS_GUIDANCE_VISUALIZER_H
#define UTEXAS_GUIDANCE_VISUALIZER_H

#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/thread.hpp>
#include <QGLViewer/qglviewer.h>
#include <utexas_guidance/graph/graph.h>
#include <utexas_guidance/mdp/structures.h>
#include <utexas_planning/core/visualizer.h>
#include <qapplication.h>

namespace utexas_guidance {

  class StateViewer : public QGLViewer {

    public:

      virtual ~StateViewer();

      typedef boost::shared_ptr<StateViewer> Ptr;
      typedef boost::shared_ptr<const StateViewer> ConstPtr;

      virtual void startEpisode(const utexas_planning::State::ConstPtr& start_state);
      virtual void updateState(const utexas_planning::State::ConstPtr& state, float timeout = 0.0f);

    protected:

      virtual void drawState(const State::ConstPtr& state);
      virtual void drawInterpolatedState(const State::ConstPtr& state1,
                                         const State::ConstPtr& state2,
                                         float ratio);
      void initializeGraph(const Graph& graph);

      Graph graph_;

      boost::mutex mutex_;
      State::ConstPtr current_state_;
      State::ConstPtr next_state_;
      float time_to_next_state_;

      boost::posix_time::ptime time_at_state_update_;

      virtual void init();
      virtual void draw();
  };

  class Visualizer : public utexas_planning::Visualizer {

    public:

      typedef boost::shared_ptr<Visualizer> Ptr;
      typedef boost::shared_ptr<const Visualizer> ConstPtr;

      virtual ~Visualizer();
      virtual void init(int argc, char* argv[]);
      virtual void startEpisode(const utexas_planning::State::ConstPtr& start_state);
      virtual void updateState(const utexas_planning::State::ConstPtr& state, float timeout = 0.0f);
      virtual void exec();

    protected:

      void initializeGraph(const Graph);

      StateViewer::Ptr viewer_;
      boost::shared_ptr<QApplication> application_;

  };

} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_VISUALIZER_H */
