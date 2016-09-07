#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

#include <class_loader/class_loader.h>

#include <utexas_guidance/mdp/guidance_model.h>
#include <utexas_guidance/mdp/single_robot_solver.h>
#include <utexas_planning/common/exceptions.h>

namespace utexas_guidance {

  class ActionEquals {
    public:
      explicit ActionEquals(const Action& a) : a_(a) {}
      inline bool operator() (const utexas_planning::Action::ConstPtr& action_base) const { 
        Action::ConstPtr action = boost::dynamic_pointer_cast<const Action>(action_base);
        if (!action) {
          throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
        }
        return (*action == a_);
      }
    private:
      Action a_;
  };

  SingleRobotSolver::~SingleRobotSolver() {}

  void SingleRobotSolver::init(const utexas_planning::GenerativeModel::ConstPtr& model_base,
                               const YAML::Node& params,
                               const std::string& output_directory,
                               const boost::shared_ptr<RNG>& rng,
                               bool verbose) {

    params_.fromYaml(params);
    model_ = boost::dynamic_pointer_cast<const GuidanceModel>(model_base);
    if (!model_) {
      throw utexas_planning::DowncastException("utexas_planning::GenerativeModel", 
                                               "utexas_guidance::GuidanceModel");
    }
    model_->getUnderlyingGraph(graph_);
    rng_ = rng;
    getAllAdjacentVertices(adjacent_vertices_map_, graph_);
    getAllShortestPaths(shortest_distances_, shortest_paths_, graph_);
  }

  void SingleRobotSolver::performEpisodeStartProcessing(const utexas_planning::State::ConstPtr& start_state, 
                                                        float timeout) {}

  utexas_planning::Action::ConstPtr SingleRobotSolver::getBestAction(const utexas_planning::State::ConstPtr& state_base) const {
  
    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }

    std::vector<utexas_planning::Action::ConstPtr> actions;
    model_->getActionsAtState(state, actions);

    for (int i = 0; i < actions.size(); ++i) {
      Action::ConstPtr action = boost::dynamic_pointer_cast<const Action>(actions[i]);
      if (!action) {
        throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
      }
      if (action->type == LEAD_PERSON) {
        int next_node = 
          shortest_paths_[state->requests[action->request_id].loc_node][state->requests[action->request_id].goal][0];
        if (params_.h0_wait_for_new_request && state->requests[action->request_id].is_new_request) {
          next_node = state->requests[action->request_id].loc_node;
        }
        if (action->node == next_node) {
          return actions[i];
        }
      }
    }

    // Return the wait action. Don't take any other actions.
    return actions.back();
  }

  void SingleRobotSolver::performPreActionProcessing(const utexas_planning::State::ConstPtr& state,
                                                     const utexas_planning::Action::ConstPtr& prev_action,
                                                     float timeout) {
    if (params_.sleep_for_timeout) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(timeout * 1000.0f));
    }
  }

  void SingleRobotSolver::performPostActionProcessing(const utexas_planning::State::ConstPtr& state,
                                                      const utexas_planning::Action::ConstPtr& action,
                                                      float timeout) {
    if (params_.sleep_for_timeout) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(timeout * 1000.0f));
    }
  }

  std::string SingleRobotSolver::getName() const {
    return std::string("SingleRobot");
  }

} /* utexas_guidance */

CLASS_LOADER_REGISTER_CLASS(utexas_guidance::SingleRobotSolver, utexas_planning::AbstractPlanner)
