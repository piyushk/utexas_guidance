#include <class_loader/class_loader.h>
#include <utexas_guidance/graph/graph.h>
#include <utexas_guidance/common.h>
#include <utexas_guidance/mdp/common.h>
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
    /* std::cout << "update state called" << std::endl; */
    boost::mutex::scoped_lock lock(mutex_);
    // std::cout << "  state update state with ";
    // state_base->serialize(std::cout);
    // std::cout << std::endl;
    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }
    if (next_state_) {
      current_state_ = next_state_;
    }
    if (timeout == 0.0f) {
      current_state_ = state;
      next_state_.reset();
    } else {
      next_state_ = state;
    }
    time_to_next_state_ = timeout;
    time_at_state_update_ = boost::posix_time::microsec_clock::local_time();
  }

  Point3f StateViewer::getLocation(int loc_u, int loc_v, float loc_p) {
    Point3f loc1 = getLocationFromGraphId(loc_u, graph_);
    Point3f loc2 = getLocationFromGraphId(loc_v, graph_);
    return getInterpolatedLocation(loc1, loc2, loc_p);
  }

  Point3f StateViewer::getInterpolatedLocation(const Point3f& loc1, const Point3f& loc2, float ratio) {
    Point3f interpolated_loc;
    interpolated_loc.set<0>((1.0f - ratio) * loc1.get<0>() + ratio * loc2.get<0>());
    interpolated_loc.set<1>((1.0f - ratio) * loc1.get<1>() + ratio * loc2.get<1>());
    interpolated_loc.set<2>((1.0f - ratio) * loc1.get<2>() + ratio * loc2.get<2>());
    return interpolated_loc;
  }

  void StateViewer::drawState(const State::ConstPtr& state) {
    drawInterpolatedState(state, state, 0.0f, 0.0f);
  }

  void StateViewer::drawInterpolatedState(const State::ConstPtr& state,
                                          const State::ConstPtr& state2,
                                          float ratio,
                                          float time) {

    utexas_guidance::draw(graph_);

    Point3f robot_offset(-0.25f, -0.25f, 0.5f);
    Point3f human_offset(0.25f, 0.25f, 0.5f);
    Point3f direct_arrow_offset(0.25f, 0.25f, 1.0f);

    for (unsigned int request_idx = 0; request_idx < state->requests.size(); ++request_idx) {
      const RequestState& rq = state->requests[request_idx];
      Point3f request_loc = getLocation(rq.loc_prev, rq.loc_node, rq.loc_p);
      Point3f human_loc = request_loc;

      if (ratio != 0.0f) {
        Point3f human2_loc = getLocationFromGraphId(rq.goal, graph_);
        for (int request_idx2 = request_idx; request_idx2 >= 0; --request_idx2) {
          if (request_idx2 >= state2->requests.size()) {
            continue;
          }

          const RequestState& rq2 = state2->requests[request_idx2];
          /* std::cout << rq2.request_id << " " << rq.request_id << std::endl; */
          if (rq2.request_id == rq.request_id) {
            /* std::cout << "in" << std::endl; */
            human2_loc = getLocation(rq2.loc_prev, rq2.loc_node, rq2.loc_p);
            break;
          }
        }
        human_loc = getInterpolatedLocation(human_loc, human2_loc, ratio);
      }

      boost::geometry::add_point(human_loc, human_offset);

      if (rq.assist_type == DIRECT_PERSON) {
        Point3f direct_arrow_loc = request_loc;
        Point3f intended_loc = getLocationFromGraphId(rq.assist_loc, graph_);
        float distance = boost::geometry::distance(direct_arrow_loc, intended_loc);
        intended_loc.set<0>(direct_arrow_loc.get<0>() + (1.0f / distance) * (intended_loc.get<0>() - direct_arrow_loc.get<0>()));
        intended_loc.set<1>(direct_arrow_loc.get<1>() + (1.0f / distance) * (intended_loc.get<1>() - direct_arrow_loc.get<1>()));
        intended_loc.set<2>(direct_arrow_loc.get<2>() + (1.0f / distance) * (intended_loc.get<2>() - direct_arrow_loc.get<2>()));
        boost::geometry::add_point(direct_arrow_loc, direct_arrow_offset);
        boost::geometry::add_point(intended_loc, direct_arrow_offset);
        drawLine(intended_loc, direct_arrow_loc, 0.0f, 0.0f, 1.0f, 0.1f, true);
      }

      /* Draw arrow to goal. */
      Point3f dest_loc = getLocationFromGraphId(rq.goal, graph_);
      boost::geometry::add_point(dest_loc, human_offset);
      drawLine(dest_loc, human_loc, 0.0f, 0.0f, 1.0f, 0.1f, true, true);

      /* Finally, draw the person. */
      drawPerson(human_loc, 0.0f, 0.0f, 1.0f);
    }

    for (unsigned int robot_idx = 0; robot_idx < state->robots.size(); ++robot_idx) {
      const RobotState& rb = state->robots[robot_idx];
      Point3f robot_loc = getLocation(rb.loc_u, rb.loc_v, rb.loc_p);

      float color_r = 0.0f;
      float color_g = 1.0f;
      float color_b = 0.0f;
      if (rb.help_destination != NONE) {
        color_r = 0.0f;
        color_g = 0.0f;
        color_b = 1.0f;
      } else if (isRobotExactlyAt(rb, rb.tau_d)) {
        color_r = 1.0f;
        color_g = 0.0f;
        color_b = 0.0f;
      }

      if (ratio != 0.0f) {
        State interpolated_state(*state);
        float unused_total_time;
        RNG rng(0);
        motion_model_->move(interpolated_state, 
                            human_decision_model_, 
                            task_generation_model_, 
                            rng /* shouldn't be used for robots. */,
                            unused_total_time,
                            time);

        const RobotState& rb2 = interpolated_state.robots[robot_idx];
        if (rb2.help_destination == NONE) {
          if (isRobotExactlyAt(rb2, rb2.tau_d)) {
            color_r = 1.0f;
            color_g = 0.0f;
            color_b = 0.0f;
          } else {
            color_r = 0.0f;
            color_g = 1.0f;
            color_b = 0.0f;
          }
        }
        robot_loc = getLocation(rb2.loc_u, rb2.loc_v, rb2.loc_p);

      }

      boost::geometry::add_point(robot_loc, robot_offset);

      drawRobot(robot_loc, color_r, color_g, color_b);
    }
  }

  void StateViewer::animate() {}

  void StateViewer::init() {
    boost::mutex::scoped_lock lock(mutex_);
    setGridIsDrawn(false);
    startAnimation();
  }

  void StateViewer::draw() {
    boost::mutex::scoped_lock lock(mutex_);
    if (current_state_) {
      glPushMatrix();
      glScalef(0.05f, 0.05f, 0.05f);
      boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();
      float time_since_state_update = (current_time - time_at_state_update_).total_milliseconds() / 1000.0f;
      if (!next_state_) {
        drawState(current_state_);
      } else if (time_since_state_update > time_to_next_state_) {
        current_state_ = next_state_;
        next_state_.reset();
        drawState(current_state_);
      } else {
        drawInterpolatedState(current_state_, 
                              next_state_, 
                              time_since_state_update / time_to_next_state_,
                              time_since_state_update);
      }
      glPopMatrix();
    }
  }

  void StateViewer::initializeModel(const Graph& graph, 
                                    const MotionModel::ConstPtr& motion_model,
                                    const HumanDecisionModel::ConstPtr& human_decision_model,
                                    const TaskGenerationModel::ConstPtr& task_generation_model) {
    graph_ = graph;
    motion_model_ = motion_model;
    human_decision_model_ = human_decision_model;
    task_generation_model_ = task_generation_model;
  }

  Visualizer::~Visualizer() {}

  void Visualizer::init(int argc, char* argv[]) {
    application_.reset(new QApplication(argc, argv));
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

  void Visualizer::initializeModel(const Graph& graph, 
                                   const MotionModel::ConstPtr& motion_model,
                                   const HumanDecisionModel::ConstPtr& human_decision_model,
                                   const TaskGenerationModel::ConstPtr& task_generation_model) {
    viewer_->initializeModel(graph,
                             motion_model,
                             human_decision_model,
                             task_generation_model);
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_guidance::Visualizer, utexas_planning::Visualizer);
