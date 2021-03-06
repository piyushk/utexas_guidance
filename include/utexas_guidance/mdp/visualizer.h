#ifndef UTEXAS_GUIDANCE_VISUALIZER_H
#define UTEXAS_GUIDANCE_VISUALIZER_H

#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/thread.hpp>
#include <QGLViewer/qglviewer.h>
#include <utexas_guidance/graph/graph.h>
#include <utexas_guidance/mdp/structures.h>
#include <utexas_guidance/mdp/transition_model.h>
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

      void initializeModel(const Graph& graph, 
                           const MotionModel::ConstPtr& motion_model,
                           const HumanDecisionModel::ConstPtr& human_decision_model,
                           const TaskGenerationModel::ConstPtr& task_generation_model);

    protected:

      Point3f getLocation(int loc_u, int loc_v, float loc_p);
      Point3f getInterpolatedLocation(const Point3f& loc1, const Point3f& loc2, float ratio);
      virtual void drawState(const State::ConstPtr& state);
      virtual void drawInterpolatedState(const State::ConstPtr& state1,
                                         const State::ConstPtr& state2,
                                         float ratio,
                                         float time);
      Graph graph_;

      boost::mutex mutex_;
      State::ConstPtr current_state_;
      State::ConstPtr next_state_;
      float time_to_next_state_;

      TaskGenerationModel::ConstPtr task_generation_model_;
      MotionModel::ConstPtr motion_model_;
      HumanDecisionModel::ConstPtr human_decision_model_;

      boost::posix_time::ptime time_at_state_update_;

      virtual void animate();
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

      void initializeModel(const Graph& graph, 
                           const MotionModel::ConstPtr& motion_model,
                           const HumanDecisionModel::ConstPtr& human_decision_model,
                           const TaskGenerationModel::ConstPtr& task_generation_model);

    protected:

      StateViewer::Ptr viewer_;
      boost::shared_ptr<QApplication> application_;

  };

} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_VISUALIZER_H */
