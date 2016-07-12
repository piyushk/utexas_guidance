#include <class_loader/class_loader.h>
#include <utexas_guidance/mdp/visualizer.h>
#include <utexas_planning/common/exceptions.h>

#include <GL/freeglut.h>

namespace utexas_guidance {

  StateViewer::~StateViewer() {}

  void StateViewer::startEpisode(const utexas_planning::State::ConstPtr& start_state) {
    boost::mutex::scoped_lock lock(mutex_);
    current_state_ = boost::dynamic_pointer_cast<const State>(start_state);
    if (!current_state_) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }
    next_state_.reset();
    time_at_state_update_ = boost::posix_time::microsec_clock::local_time();
    time_to_next_state_ = 0.0;
  }

  void StateViewer::updateState(const utexas_planning::State::ConstPtr& state_base, float timeout) {
    boost::mutex::scoped_lock lock(mutex_);
    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }
    if (next_state_) {
      current_state_ = next_state_;
    }
    if (timeout == 0.0f) {
      current_state_ = state;
    } else {
      next_state_ = state;
    }
    time_to_next_state_ = timeout;
    time_at_state_update_ = boost::posix_time::microsec_clock::local_time();
  }

  Point3f StateViewer::getLocation(int loc_u, int loc_p, float loc_p) {
    Point3f loc1 = getLocationFromGraphId(loc_u, graph_);
    Point3f loc2 = getLocationFromGraphId(loc_v, graph_);
    return getInterpolatedLocation(loc1, loc2, loc_p);
  }

  Point3f StateViewer::getInterpolatedLocation(const Point3f& loc1, const Point3f& loc2, float ratio) {
    Point3f interpolated_loc;
    interpolated_loc.set<0>((1.0f - ratio) * loc1.get<0> + ratio * loc2.get<0>);
    interpolated_loc.set<1>((1.0f - ratio) * loc1.get<1> + ratio * loc2.get<1>);
    interpolated_loc.set<2>((1.0f - ratio) * loc1.get<2> + ratio * loc2.get<2>);
    return interpolated_loc;
  }

  void StateViewer::drawState(const State::ConstPtr& state) {
    drawGraph(graph_);

    Point3f robot_offset(0.5f, -0.25f, -0.25f);
    Point3f human_offset(0.5f, 0.25f, 0.25f);
    Point3f direct_arrow_offset(1.0f, 0.25f, 0.25f);

    for (unsigned int request_idx = 0; request_idx < state->requests.size(); ++request_idx) {
      const RequestState& rq = state->requests[request_idx];
      Point3f request_loc = getLocation(rq.loc_prev, rq.loc_node, rq.loc_p);
      Point3f human_loc = request_loc;
      boost::geometry::add_point(human_loc, human_offset);
      if (rq.assist_type == DIRECT_PERSON) {
        Point3f direct_arrow_loc = request_loc;
        boost::geometry::add_point(direct_arrow_loc, direct_arrow_offset);
        Point3f intended_loc = getLocationFromGraphId(rq.assist_loc, graph_);
        boost::geometry::add_point(intended_loc, direct_arrow_offset);
        drawLine(direct_arrow_loc, intended_loc, 0.0f, 0.0f, 1.0f, 0.1f, true);
      }

      /* Draw arrow to goal. */
      Point3f dest_loc = getLocationFromGraphId(rq.dest, graph_);
      boost::geometry::add_point(dest_loc, human_offset);
      drawLine(dest_loc, human_loc, 0.0f, 0.0f, 1.0f, 0.1f, true, true);

      /* Finally, draw the person. */
      drawPerson(human_loc, 0.0f, 0.0f, 1.0f);
    }

    for (unsigned int robot_idx = 0; robot_idx < state->robots.size(); ++robot_idx) {
      const RobotState& rb = state->robots[robot_idx];
      Point3f robot_loc = getLocation(rq.loc_prev, rq.loc_node, rq.loc_p);
      boost::geometry::add_point(robot_loc, robot_offset);

      bool real_dest_arrow_dashed = rb.help_destination != NONE;
      bool draw_assign_arrow = rb.help_destination != NONE && !rb.is_leading_person &&


    }

  }

  void StateViewer::drawInterpolatedState(const State::ConstPtr& state1,
                                          const State::ConstPtr& state2,
                                          float ratio) {

  }

  void StateViewer::init() {
    boost::mutex::scoped_lock lock(mutex_);
    /* glDisable(GL_LIGHTING); */
    glLineWidth(1.0f);
    setGridIsDrawn(false);
  }

  void StateViewer::draw() {
    boost::mutex::scoped_lock lock(mutex_);
    boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();
    float time_since_state_update = (current_time - time_at_state_update_).total_milliseconds();
    if (!next_state_) {
      drawState(current_state_);
    } else if (time_since_state_update > time_to_next_state_) {
      current_state_ = next_state_;
      next_state_.reset();
      drawState(current_state_);
    } else {
      drawInterpolatedState(current_state_, next_state_, time_since_state_update / time_to_next_state_);
    }
  }

  void initializeGraph(const Graph& graph) {
    graph_ = graph;
  }

  Visualizer::~Visualizer() {}

  void Visualizer::init(int argc, char* argv[]) {
    QApplication application(argc, argv);
    glutInit(&argc, argv);
    viewer_.reset(new StateViewer);
    viewer_->setWindowTitle("GuidanceVisualizer");
    viewer_->show();
  }

  void Visualizer::startEpisode(const utexas_planning::State::ConstPtr& start_state) {
    viewer_->startEpisode(start_state);
  }

  void Visualizer::updateState(const utexas_planning::State::ConstPtr& state, float timeout) {
    viewer_->updateState(state, timeout);
  }

  void Visualizer::exec() {
    application_->exec();
  }

  void initializeGraph(const Graph& graph) {
    viewer_->initializeGraph(graph);
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_guidance::Visualizer, utexas_planning::Visualizer);
