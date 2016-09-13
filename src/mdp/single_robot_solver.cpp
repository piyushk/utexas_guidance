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
                                                        float timeout) {
    if (params_.sleep_for_timeout) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(timeout * 1000.0f));
    }
  }

  utexas_planning::Action::ConstPtr SingleRobotSolver::getBestAction(const utexas_planning::State::ConstPtr& state_base) const {
  
    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }

    std::vector<utexas_planning::Action::ConstPtr> actions;
    model_->getActionsAtState(state, actions);

    // See which combination of releasing robots maximizes the solvers ability to lead to the goal.
    std::vector<std::vector<utexas_planning::Action::ConstPtr> > possible_release_combinations;
    possible_release_combinations.push_back(std::vector<utexas_planning::Action::ConstPtr>());

    for (int i = 0; i < actions.size(); ++i) {
      Action::ConstPtr action = boost::dynamic_pointer_cast<const Action>(actions[i]);
      if (!action) {
        throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
      }
      if (action->type == RELEASE_ROBOT) {
        int current_max_num_combos = possible_release_combinations.size();
        for (int j = 0; j < current_max_num_combos; ++j) {
          std::vector<utexas_planning::Action::ConstPtr> current_combo = possible_release_combinations[j];
          current_combo.push_back(action);
          possible_release_combinations.push_back(current_combo);
        }
      } else {
        // All RELEASE actions are bundled first.
        break;
      }
    }

    int max_lead_count = 0;
    int max_combo_idx = 0;
    for (int j = 1; j < possible_release_combinations.size(); ++j) {
      std::vector<utexas_planning::Action::ConstPtr> current_combo = possible_release_combinations[j];
      utexas_planning::State::ConstPtr current_state = state_base, next_state;
      for (int action_idx = 0; action_idx < current_combo.size(); ++action_idx) {
        float unused_reward, unused_timeout;
        int unused_depth_count;
        model_->takeAction(current_state, 
                           current_combo[action_idx],
                           unused_reward,
                           utexas_planning::RewardMetrics::Ptr(),
                           next_state,
                           unused_depth_count,
                           unused_timeout,
                           rng_);
        current_state = next_state;
      }

      // Now count the number of LEAD actions available at this state.
      std::vector<utexas_planning::Action::ConstPtr> current_actions;
      model_->getActionsAtState(current_state, current_actions);

      int lead_count = 0;
      for (int i = 0; i < current_actions.size(); ++i) {
        Action::ConstPtr action = boost::dynamic_pointer_cast<const Action>(current_actions[i]);
        if (!action) {
          throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
        }
        if (action->type == LEAD_PERSON) {
          ++lead_count;
        }
      }
      
      if (lead_count > max_lead_count) {
        max_combo_idx = j;
        max_lead_count = lead_count;
      }
    }

    if (max_combo_idx != 0) {
      // std::cout << "returning release combo" << std::endl;
      return possible_release_combinations[max_combo_idx][0];
    }

    // See if any robots need to be released so that the human can be lead.
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

    Action::ConstPtr last_action = boost::dynamic_pointer_cast<const Action>(actions.back());
    if (!last_action) {
      throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
    }
    if (last_action->type != WAIT && 
        (last_action->type != LEAD_PERSON && !isRobotExactlyAt(state->robots[last_action->robot_id], last_action->node))) {
      state->serialize(std::cout);
      std::cout << std::endl;
      last_action->serialize(std::cout);
      std::cout << std::endl;
      throw utexas_planning::IncorrectUsageException("SingleRobotSolver attempted to return last action, but it was not wait.");
    }

    // Return the wait action. Don't take any other actions.
    // return actions.back();
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
