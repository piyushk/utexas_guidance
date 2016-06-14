#include <boost/foreach.hpp>
#include <cassert>
#include <cmath>
#include <fstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include <utexas_guidance/exceptions.h>
#include <utexas_guidance/mdp/common.h>
#include <utexas_guidance/mdp/guidance_model.h>

#include <utexas_planning/common/exceptions.h>

namespace utexas_guidance {

  GuidanceModel::~GuidanceModel() {}

  void GuidanceModel::init(const YAML::Node& params,
                           const std::string& output_directory,
                           const boost::shared_ptr<RNG>& rng) {

    params_.fromYaml(params);

    /* Initialize all graphs and models. */
    // TODO Test for graph and task allocation file here.
    readGraphFromFile(params_.graph_file, graph_);
    human_decision_model_.reset(new HumanDecisionModel(graph_, params_.human_variance_multiplier));
    if (params_.task_model == FIXED_TASK_MODEL) {
      task_generation_model_.reset(new FixedTaskGenerationModel(params_.task_model_file,
                                                                params_.task_utility,
                                                                params_.task_time));
    } else if (params_.task_model == RANDOM_TASK_MODEL) {
      task_generation_model_.reset(new RandomTaskGenerationModel(params_.task_model_file,
                                                                 graph_,
                                                                 params_.task_utility,
                                                                 params_.task_time,
                                                                 params_.task_at_home_base_only));
    } else {
      throw IncorrectUsageException(std::string("Task model: ") + params_.task_model + std::string(" is not defined."));
    }
    motion_model_.reset(new MotionModel(graph_,
                                        params_.human_speed,
                                        params_.robot_speed,
                                        params_.elevator_human_speed,
                                        params_.elevator_robot_speed));

    num_vertices_ = boost::num_vertices(graph_);
    getAllAdjacentVertices(adjacent_vertices_map_, graph_);
    getAllShortestPaths(shortest_distances_, shortest_paths_, graph_);
  }

  std::string GuidanceModel::getName() const {
    return std::string("GuidanceModel");
  }

  bool GuidanceModel::isTerminalState(const utexas_planning::State::ConstPtr&) const {
    return false;
  }

  void GuidanceModel::takeAction(const utexas_planning::State::ConstPtr& state_base,
                                 const utexas_planning::Action::ConstPtr& action_base,
                                 float& reward,
                                 const utexas_planning::RewardMetrics::Ptr& reward_metrics,
                                 utexas_planning::State::ConstPtr& next_state_base,
                                 int& depth_count,
                                 float& post_action_timeout,
                                 boost::shared_ptr<RNG> rng) const {

    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }

    Action::ConstPtr action = boost::dynamic_pointer_cast<const Action>(action_base);
    if (!action) {
      throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
    }

    State::Ptr next_state(new State(*state));
    if (action->type != WAIT) {
      if (action->type == ASSIGN_ROBOT) {
        next_state->robots[action->robot_id].help_destination = action->node;
      } else if (action->type == RELEASE_ROBOT) {
        next_state->robots[action->robot_id].help_destination = NONE;
      } else /* if (action.type == DIRECT_PERSON || action.type == LEAD_PERSON) */ {
        next_state->requests[action->request_id].assist_type = action->type;
        next_state->requests[action->request_id].assist_loc = action->node;
      }
      reward = 0.0f;
      depth_count = 0;
      post_action_timeout = 0; // No more time to do planning.
    } else {

      motion_model_->move(*next_state, human_decision_model_, task_generation_model_, *rng, post_action_timeout);
      float time_loss = state->requests.size() * post_action_timeout;

      // TODO move these to part of RewardMetrics
      float utility_loss = 0.0f;
      for(unsigned int i = 0; i < next_state->robots.size(); ++i) {
        const RobotState& orig_robot = state->robots[i];
        const RobotState& robot = next_state->robots[i];
        if (robot.help_destination != NONE) {
          float distance_to_service_destination_before_action =
            getTrueDistanceTo(orig_robot.loc_u, orig_robot.loc_v, orig_robot.loc_p, orig_robot.tau_d, shortest_distances_);
          float distance_to_service_destination =
            getTrueDistanceTo(robot.loc_u, robot.loc_v, robot.loc_p, robot.tau_d, shortest_distances_);
          float extra_distance_to_service_destination =
            distance_to_service_destination - distance_to_service_destination_before_action;
          float extra_time_to_service_destination =
            extra_distance_to_service_destination / motion_model_->getRobotSpeed();
          float utility_loss_per_robot = (extra_time_to_service_destination + time_loss) * robot.tau_u;
          utility_loss += utility_loss_per_robot;
        }
      }

      reward = -(time_loss + utility_loss);
      depth_count = lrint(time_loss);
    }

    next_state_base = boost::static_pointer_cast<const State>(next_state);
  }

  void GuidanceModel::getColocatedRobotRequestIds(const State& state,
                                                  std::vector<std::pair<int, int> >& robot_request_ids) const {
    // Figure out if there is a robot at the current position
    for (int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
      if (state.robots[robot_id].request_id != NONE) {
        for (int request_id = 0; request_id < state.requests.size(); ++request_id) {
          if ((state.robots[robot_id].help_destination == state.requests[request_id].loc_node) &&
              isRobotExactlyAt(state.robots[robot_id], state.requests[request_id].loc_node)) {
            robot_request_ids.push_back(std::pair<int, int>(robot_id, request_id));
          }
        }
      }
    }
  }

  void GuidanceModel::getActionsAtState(const utexas_planning::State::ConstPtr& state_base,
                                        std::vector<utexas_planning::Action::ConstPtr>& actions) const {
    actions.clear();
    int action_counter = 0;

    // Wait is allowed at all times.
    Action::Ptr action(new Action(WAIT, 0, 0));
    actions.push_back(action);
    ++action_counter;

    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }

    // If every request is being handled by leading the person, no other action can be performed.
    bool all_requests_being_lead = true;
    BOOST_FOREACH(const RequestState& rs, state->requests) {
      if (rs.assist_type != LEAD_PERSON) {
        all_requests_being_lead = false;
        break;
      }
    }

    // No more actions available.
    if (all_requests_being_lead) {
      return;
    }

    // Allow robots to be assigned/released at any given location.
    for (unsigned int robot = 0; robot < state->robots.size(); ++robot) {
      if (state->robots[robot].help_destination == NONE) {
        actions.resize(action_counter + num_vertices_);
        for (unsigned int node = 0; node < num_vertices_; ++node) {
          actions[action_counter] = Action::Ptr(new Action(ASSIGN_ROBOT, robot, node));
          ++action_counter;
        }
      } else {
        actions.push_back(Action::Ptr(new Action(RELEASE_ROBOT, robot)));
        ++action_counter;
      }
    }

    // Allow leading/guiding via all colocated robots.
    std::vector<std::pair<int, int> > robot_request_ids;
    getColocatedRobotRequestIds(*state, robot_request_ids);
    for (unsigned int idx_num = 0; idx_num < robot_request_ids.size(); ++idx_num) {
      int robot_id = robot_request_ids[idx_num].first;
      /* const RobotState& robot = state->robots[robot_id]; */
      int request_id = robot_request_ids[idx_num].second;
      const RequestState& request = state->requests[request_id];
      actions.resize(action_counter + 2 * adjacent_vertices_map_[request.loc_node].size());
      for (unsigned int adj = 0; adj < adjacent_vertices_map_[request.loc_node].size(); ++adj) {
        actions[action_counter] =
          Action::Ptr(new Action(DIRECT_PERSON, robot_id, adjacent_vertices_map_[request.loc_node][adj], request_id));
        actions[action_counter + 1] =
          Action::Ptr(new Action(LEAD_PERSON, robot_id, adjacent_vertices_map_[request.loc_node][adj], request_id));
        action_counter += 2;
      }
    }
  }

} /* utexas_guidance */
