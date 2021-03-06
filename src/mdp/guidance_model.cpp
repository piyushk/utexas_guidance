#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <cassert>
#include <cmath>
#include <fstream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include <class_loader/class_loader.h>

#include <utexas_guidance/exceptions.h>
#include <utexas_guidance/mdp/common.h>
#include <utexas_guidance/mdp/guidance_model.h>
#include <utexas_guidance/mdp/visualizer.h>

#include <utexas_planning/common/exceptions.h>

/*
 * fix problems with conflating time and distance. double check everything.
 * Do visualization - much easier to debug. Do it with motion once time is available.
 */
namespace utexas_guidance {

  GuidanceModel::~GuidanceModel() {}

  void GuidanceModel::init(const YAML::Node& params,
                           const std::string& output_directory,
                           const boost::shared_ptr<RNG>& rng) {

    params_.fromYaml(params);
    std::cout << params_ << std::endl;

    /* Initialize all graphs and models. */
    // TODO Test for graph and task allocation file here.
    readGraphFromFile(params_.graph_file, graph_);
    human_decision_model_.reset(new HumanDecisionModel(graph_, 
                                                       params_.is_deterministic,
                                                       params_.human_variance_multiplier,
                                                       params_.human_mean_noise_multiplier));
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
                                        params_.elevator_robot_speed,
                                        params_.terminate_with_robot_present));

    num_vertices_ = boost::num_vertices(graph_);
    getAllAdjacentVertices(adjacent_vertices_map_, graph_);
    getAllShortestPaths(shortest_distances_, shortest_paths_, graph_);
  }

  std::string GuidanceModel::getName() const {
    return std::string("GuidanceModel");
  }

  bool GuidanceModel::isTerminalState(const utexas_planning::State::ConstPtr& state_base) const {
    if (params_.terminate_at_zero_requests) {
      State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
      if (!state) {
        throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
      }
      return state->requests.size() == 0;
    }
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
        if (action->type == LEAD_PERSON) {
          next_state->robots[action->robot_id].help_destination = action->node;
          next_state->robots[action->robot_id].is_leading_person = true;
          if (isRobotExactlyAt(next_state->robots[action->robot_id], action->node)) {
            next_state->requests[action->request_id].wait_time_left = 10.0f;
          }
        }
      }
      next_state->actions_since_wait.push_back(*action);
      reward = 0.0f;
      depth_count = 0;
      post_action_timeout = 0; // No more time to do planning.
    } else {

      float unhelpful_robot_penalty = 0.0f;

      std::vector<int> unhelpful_robot_ids;
      for (int last_action_idx = 0; last_action_idx < state->actions_since_wait.size(); ++last_action_idx) {
        if (state->actions_since_wait[last_action_idx].type == RELEASE_ROBOT) {
          unhelpful_robot_ids.push_back(state->actions_since_wait[last_action_idx].robot_id);
        } else if (state->actions_since_wait[last_action_idx].type == DIRECT_PERSON) {
          unhelpful_robot_ids.erase(std::remove(unhelpful_robot_ids.begin(), 
                                                unhelpful_robot_ids.end(), 
                                                state->actions_since_wait[last_action_idx].robot_id), 
                                    unhelpful_robot_ids.end());
        }
      }
      unhelpful_robot_penalty += params_.h4_unhelpful_robot_penalty * unhelpful_robot_ids.size();
      
      next_state->actions_since_wait.clear();

      motion_model_->move(*next_state, human_decision_model_, task_generation_model_, *rng, post_action_timeout);
      float time_loss = state->requests.size() * post_action_timeout;

      for (unsigned int i = 0; i < next_state->requests.size(); ++i) {
        next_state->requests[i].is_new_request = false;
      }

      if (isTerminalState(next_state)) {
        // Penalize for any assigned robots.
        for(unsigned int i = 0; i < next_state->robots.size(); ++i) {
          const RobotState& robot = next_state->robots[i];
          if (robot.help_destination != NONE) {
            unhelpful_robot_penalty  += params_.h4_unhelpful_robot_penalty;
          }
        }
      }

      float utility_loss = 0.0f;
      for(unsigned int i = 0; i < next_state->robots.size(); ++i) {
        const RobotState& orig_robot = state->robots[i];
        const RobotState& robot = next_state->robots[i];
        if (orig_robot.help_destination != NONE) {
          float distance_to_current_service_destination_before_action =
            getTrueDistanceTo(orig_robot.loc_u, orig_robot.loc_v, orig_robot.loc_p, robot.tau_d, shortest_distances_);
          /* std::cout << "d1: " << distance_to_current_service_destination_before_action << std::endl; */
          float distance_to_current_service_destination =
            getTrueDistanceTo(robot.loc_u, robot.loc_v, robot.loc_p, robot.tau_d, shortest_distances_);
          /* std::cout << "d2: " << distance_to_current_service_destination << std::endl; */
          float extra_distance_to_current_service_destination =
            distance_to_current_service_destination - distance_to_current_service_destination_before_action;
          float extra_time_to_current_service_destination =
            extra_distance_to_current_service_destination / motion_model_->getRobotSpeed();
          /* std::cout << "t1: " << extra_time_to_current_service_destination << std::endl; */
          float task_time_saved = 0.0f;
          // We could have at most moved forward one task.
          if (robot.tau_d != orig_robot.tau_d) {
            task_time_saved = (orig_robot.tau_total_task_time - orig_robot.tau_t + robot.tau_t);
          } else {
            task_time_saved = robot.tau_t - orig_robot.tau_t;
          }
          /* std::cout << "t2: " << -task_time_saved << std::endl; */
          /* std::cout << "t3: " << post_action_timeout << std::endl; */
          float utility_loss_per_robot = 
            (extra_time_to_current_service_destination + post_action_timeout - task_time_saved) * robot.tau_u;
          /* std::cout << "ul: " << utility_loss_per_robot << std::endl; */
          utility_loss += utility_loss_per_robot;
        }
      }

      reward = -(time_loss + utility_loss + unhelpful_robot_penalty);
      depth_count = lrint(post_action_timeout);

      if (reward_metrics) {
        boost::shared_ptr<RewardMetrics> reward_metrics_derived =
          boost::dynamic_pointer_cast<RewardMetrics>(reward_metrics);
        if (!reward_metrics_derived) {
          throw utexas_planning::DowncastException("utexas_planning::RewardMetrics", "utexas_guidance::RewardMetrics");
        }
        reward_metrics_derived->utility_loss -= utility_loss;
        reward_metrics_derived->time_loss -= time_loss;
        reward_metrics_derived->reward_normalized += reward / params_.normalizer;
        reward_metrics_derived->utility_loss_normalized -= utility_loss / params_.normalizer;
        reward_metrics_derived->time_loss_normalized -= time_loss / params_.normalizer;
      }

    }

    next_state_base = boost::static_pointer_cast<const utexas_planning::State>(next_state);
  }

  void GuidanceModel::unrollState(const State& state, State& root_state) const {
    root_state = state;
    for (int action_idx = 0; action_idx < state.actions_since_wait.size(); ++action_idx) {
      const Action& action = root_state.actions_since_wait[action_idx];
      if (action.type == ASSIGN_ROBOT) {
       root_state.robots[action.robot_id].help_destination = action.old_help_destination;
      } else if (action.type == RELEASE_ROBOT) {
        root_state.robots[action.robot_id].help_destination = action.old_help_destination;
      } else {
        root_state.requests[action.request_id].assist_type = NONE;
        root_state.requests[action.request_id].assist_loc = NONE;
        if (action.type == LEAD_PERSON) {
          root_state.robots[action.robot_id].help_destination = action.old_help_destination;
          root_state.robots[action.robot_id].is_leading_person = false;
        }
      }
    }
    root_state.actions_since_wait.clear();
  }

  void GuidanceModel::getActionsAtState(const utexas_planning::State::ConstPtr& state_base,
                                        std::vector<utexas_planning::Action::ConstPtr>& actions) const {
    actions.clear();
    int action_counter = 0;

    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }

    Action last_action(WAIT);
    if (state->actions_since_wait.size() != 0) {
      last_action = state->actions_since_wait.back();
    }

    // DECOMPOSITION ORDERING IS RELEASE -> DIRECT -> LEAD -> ASSIGN

    // RELEASE_ROBOT
    if (last_action.type == WAIT || last_action.type == RELEASE_ROBOT) {
      int min_releasable_idx = 0;
      if (last_action.type == RELEASE_ROBOT) {
        min_releasable_idx = last_action.robot_id + 1;
      }
      for (unsigned int robot = min_releasable_idx; robot < state->robots.size(); ++robot) {
        if ((state->robots[robot].help_destination != NONE) &&
            !(state->robots[robot].is_leading_person)) {
          actions.push_back(Action::Ptr(new Action(RELEASE_ROBOT, 
                                                   robot, 
                                                   NONE,
                                                   NONE,
                                                   state->robots[robot].help_destination)));
          ++action_counter;
        }
      }
    }

    // TODO: Heuristic - Parameterize.
    int max_assigned_robots = state->robots.size();
    if (params_.h1_max_assigned_robots != MAX_ASSIGNED_ROBOTS_NOLIMIT) {
      max_assigned_robots = 
        (params_.h1_max_assigned_robots == MAX_ASSIGNED_ROBOTS_SAME_AS_REQUESTS) ? 
        state->requests.size() : params_.h1_max_assigned_robots;

    }
    for (int robot_id = 0; robot_id < state->robots.size(); ++robot_id) {
      if (state->robots[robot_id].help_destination != NONE) {
        --max_assigned_robots;
      }
    }

    // Any robot that has been released cannot be assigned - it should have been assigned or lead
    // directly without calling release.
    std::vector<bool> assignable_robot_ids(state->robots.size(), true);
    for (int last_action_idx = 0; last_action_idx < state->actions_since_wait.size(); ++last_action_idx) {
      if (state->actions_since_wait[last_action_idx].type == RELEASE_ROBOT ||
          state->actions_since_wait[last_action_idx].type == LEAD_PERSON) {
        assignable_robot_ids[state->actions_since_wait[last_action_idx].robot_id] = false;
      }
    }

    // DIRECT_PERSON or LEAD_PERSON
    bool direct_action_available = false;
    if (last_action.type == WAIT || last_action.type == RELEASE_ROBOT || 
        last_action.type == DIRECT_PERSON || last_action.type == LEAD_PERSON) {
      int min_directable_idx = 0;
      if (last_action.type == DIRECT_PERSON || last_action.type == LEAD_PERSON) {
        min_directable_idx = last_action.request_id + 1;
      }
      for (int request_id = min_directable_idx; request_id < state->requests.size(); ++request_id) {
        if (state->requests[request_id].loc_p == 1.0f && 
            state->requests[request_id].assist_type == NONE) {
          // See if robot(s) are colocated with this request.
          for (int robot_id = 0; robot_id < state->robots.size(); ++robot_id) {
            float loc_p = state->robots[robot_id].loc_p;
            int exact_loc = (loc_p == 0.0f) ? state->robots[robot_id].loc_u :
              (loc_p == 1.0f) ? state->robots[robot_id].loc_v : NONE;
            if (state->requests[request_id].wait_time_left == 0.0f &&
                exact_loc == state->requests[request_id].loc_node) {

              direct_action_available = true;
              
              // LEAD_PERSON
              if (assignable_robot_ids[robot_id] &&
                  (state->robots[robot_id].help_destination != NONE ||
                   max_assigned_robots > 0)) {

                if (params_.h0_lead_for_new_request && state->requests[request_id].is_new_request) {
                  int next_node = 
                    shortest_paths_[state->requests[request_id].loc_node][state->requests[request_id].goal][0];
                  actions.push_back(Action::Ptr(new Action(LEAD_PERSON, robot_id, next_node, request_id)));
                  ++action_counter;
                  return;
                }

                actions.push_back(Action::Ptr(new Action(LEAD_PERSON, robot_id, exact_loc, request_id)));
                ++action_counter;

                if (params_.h0_wait_for_new_request && state->requests[request_id].is_new_request) {
                  return;
                }

                actions.resize(action_counter + adjacent_vertices_map_[exact_loc].size());
                for (unsigned int adj = 0; adj < adjacent_vertices_map_[exact_loc].size(); ++adj) {
                  actions[action_counter] =
                    Action::Ptr(new Action(LEAD_PERSON, 
                                           robot_id, 
                                           adjacent_vertices_map_[exact_loc][adj], 
                                           request_id,
                                           state->robots[robot_id].help_destination
                                           ));
                  ++action_counter;
                }
              }

              // DIRECT_PERSON
              actions.resize(action_counter + adjacent_vertices_map_[exact_loc].size());
              for (unsigned int adj = 0; adj < adjacent_vertices_map_[exact_loc].size(); ++adj) {
                actions[action_counter] =
                  Action::Ptr(new Action(DIRECT_PERSON, robot_id, adjacent_vertices_map_[exact_loc][adj], request_id));
                ++action_counter;
              }

            }
          }
        }
        if (params_.h3_force_assistance && direct_action_available) {
          return;
        }
      }
    }

    // ASSIGN_ROBOT
    int min_assignable_idx = 0;
    if (last_action.type == ASSIGN_ROBOT) {
      min_assignable_idx = last_action.robot_id + 1;
    }
    for (int robot_id = min_assignable_idx; robot_id < state->robots.size(); ++robot_id) {
      if (!(state->robots[robot_id].is_leading_person) && 
          (max_assigned_robots > 0 || state->robots[robot_id].help_destination != NONE) &&
          assignable_robot_ids[robot_id]) {
        if (!params_.h2_only_allow_adjacent_assignment) {
          actions.resize(action_counter + num_vertices_);
          for (unsigned int node = 0; node < num_vertices_; ++node) {
            actions[action_counter] = Action::Ptr(new Action(ASSIGN_ROBOT, 
                                                             robot_id, 
                                                             node,
                                                             NONE,
                                                             state->robots[robot_id].help_destination
                                                             ));
            ++action_counter;
          }
        } else {
          float loc_p = state->robots[robot_id].loc_p;
          int exact_loc = (loc_p == 0.0f) ? state->robots[robot_id].loc_u :
            (loc_p == 1.0f) ? state->robots[robot_id].loc_v : NONE;
          if (exact_loc != NONE) {
            actions.resize(action_counter + adjacent_vertices_map_[exact_loc].size() + 1);
            for (unsigned int adj = 0; adj < adjacent_vertices_map_[exact_loc].size(); ++adj) {
              actions[action_counter] =
                Action::Ptr(new Action(ASSIGN_ROBOT, 
                                       robot_id, 
                                       adjacent_vertices_map_[exact_loc][adj],
                                       NONE,
                                       state->robots[robot_id].help_destination
                                       ));
              ++action_counter;
            }
            actions[action_counter] = Action::Ptr(new Action(ASSIGN_ROBOT, 
                                                             robot_id, 
                                                             exact_loc,
                                                             NONE,
                                                             state->robots[robot_id].help_destination
                                                             ));
            ++action_counter;
          } else {
            actions.resize(action_counter + 2);
            actions[action_counter] = Action::Ptr(new Action(ASSIGN_ROBOT, 
                                                             robot_id, 
                                                             state->robots[robot_id].loc_u,
                                                             NONE,
                                                             state->robots[robot_id].help_destination
                                                             ));
            ++action_counter;
            actions[action_counter] = Action::Ptr(new Action(ASSIGN_ROBOT, 
                                                             robot_id, 
                                                             state->robots[robot_id].loc_v,
                                                             NONE,
                                                             state->robots[robot_id].help_destination
                                                             ));
            ++action_counter;
          }
        }
      }
    }

    // WAIT
    actions.push_back(Action::Ptr(new Action(WAIT)));
  }

  utexas_planning::State::ConstPtr GuidanceModel::getStartState(long seed) {
    State::Ptr state(new State);
    RNG rng(seed);

    /* Fill in the robots portion first. */
    state->robots.resize(task_generation_model_->getNumRobots());
    for (unsigned int robot_idx = 0; robot_idx < state->robots.size(); ++robot_idx) {
      state->robots[robot_idx].loc_u = state->robots[robot_idx].loc_v =
        task_generation_model_->getStartingLocationForRobot(robot_idx);
      state->robots[robot_idx].loc_p = 0.0f;
      state->robots[robot_idx].tau_d = NONE;
      task_generation_model_->generateNewTaskForRobot(robot_idx, state->robots[robot_idx], rng);
      state->robots[robot_idx].is_leading_person = false;
      state->robots[robot_idx].help_destination = NONE;
    }

    /* Ignore seed if debug parameters set. */
    if (!params_.start_idxs.empty() || !params_.goal_idxs.empty()) {
      if (params_.start_idxs.empty()) {
        throw IncorrectUsageException("GuidanceModel: Empty start_idxs and non-empty goal_idxs provided via params!");
      } else if (params_.goal_idxs.empty()) {
        throw IncorrectUsageException("GuidanceModel: Non-empty start_idxs and empty goal_idxs provided via params!");
      }
      std::vector<std::string> start_idxs;
      std::vector<std::string> goal_idxs;
      boost::split(start_idxs, params_.start_idxs, boost::is_any_of(" ,;:"));
      boost::split(goal_idxs, params_.goal_idxs, boost::is_any_of(" ,;:"));
      if (start_idxs.size() != goal_idxs.size()) {
        throw IncorrectUsageException("GuidanceModel: start_idxs and goal_idxs provided via params are diff length!");
      }
      state->requests.resize(start_idxs.size());
      try {
        for (unsigned int i = 0; i < start_idxs.size(); ++i) {
          state->requests[i].loc_node = boost::lexical_cast<int>(start_idxs[i]);
          if (state->requests[i].loc_node < 0 || state->requests[i].loc_node >= num_vertices_) {
            throw IncorrectUsageException(std::string("GuidanceModel: A start_idx provided is not in range {0,...,") +
                                          boost::lexical_cast<std::string>(num_vertices_) + std::string("}"));
          }
          state->requests[i].goal = boost::lexical_cast<int>(goal_idxs[i]);
          if (state->requests[i].goal < 0 || state->requests[i].goal >= num_vertices_) {
            throw IncorrectUsageException(std::string("GuidanceModel: A goal_idx provided is not in range {0,...,") +
                                          boost::lexical_cast<std::string>(num_vertices_) + std::string("}"));
          }
        }
      } catch (const boost::bad_lexical_cast& e) {
        throw IncorrectUsageException("GuidanceModel: unable to parse start_idxs and/or goal_idxs as list of ints!");
      }
    } else {
      state->requests.resize(params_.initial_num_requests);
      params_.start_idxs = "";
      params_.goal_idxs = "";
      for (unsigned int i = 0; i < state->requests.size(); ++i) {
        // Make sure that this request doesn't start at the same location as another request.
        while (true) {
          state->requests[i].loc_node = state->robots[rng.randomInt(state->robots.size() - 1)].loc_u;
          bool loc_diff_other_requests = true;
          for (unsigned int j = 0; j < i; ++j) {
            if (state->requests[i].loc_node == state->requests[j].loc_node) {
              loc_diff_other_requests = false;
              break;
            }
          }
          if (loc_diff_other_requests) {
            break;
          }
        }
        // Make sure goal is different from start.
        state->requests[i].goal = rng.randomInt(num_vertices_ - 1);
        while (state->requests[i].goal == state->requests[i].loc_node) {
          state->requests[i].goal = rng.randomInt(num_vertices_ - 1);
        }

        params_.start_idxs += boost::lexical_cast<std::string>(state->requests[i].loc_node);
        params_.goal_idxs += boost::lexical_cast<std::string>(state->requests[i].goal);
        if (i != state->requests.size() - 1) {
          params_.start_idxs += ",";
          params_.goal_idxs += ",";
        }
      }
    }

    params_.normalizer = 0.0f;
    for (unsigned int i = 0; i < state->requests.size(); ++i) {
      state->requests[i].is_new_request = true;
      state->requests[i].request_id = generateNewRequestId();
      state->requests[i].loc_prev = state->requests[i].loc_node;
      state->requests[i].loc_p = 1.0f;
      state->requests[i].assist_type = NONE;
      state->requests[i].assist_loc = NONE;
      params_.normalizer = std::max(params_.normalizer, 
                                    shortest_distances_[state->requests[i].loc_node][state->requests[i].goal] / params_.human_speed);
    }



    // for (unsigned int i = 0; i < state->requests.size(); ++i) {
    //   for (unsigned int robot_idx = 0; robot_idx < state->robots.size(); ++robot_idx) {
    //     if ((state->robots[robot_idx].help_destination == NONE) &&
    //         (state->robots[robot_idx].loc_u == state->requests[i].loc_node)) {
    //       state->robots[robot_idx].help_destination = state->robots[robot_idx].loc_u;
    //     }
    //   }
    // }

    return state;
  }

  float GuidanceModel::getInitialTimeout() const {
    return params_.initial_planning_time;
  }

  void GuidanceModel::initializeVisualizer(const utexas_planning::Visualizer::Ptr& visualizer_base) const {
    Visualizer::Ptr visualizer = boost::dynamic_pointer_cast<Visualizer>(visualizer_base);
    if (visualizer) {
      visualizer->initializeModel(graph_, motion_model_, human_decision_model_, task_generation_model_);
    } /* else do nothing, incompatible visualizer. shouldn't throw an error. */
  }

  void GuidanceModel::getUnderlyingGraph(Graph& graph) const {
    graph = graph_;
  }

  MotionModel::ConstPtr GuidanceModel::getUnderlyingMotionModel() const {
    return motion_model_;
  }

  TaskGenerationModel::ConstPtr GuidanceModel::getUnderlyingTaskModel() const {
    return task_generation_model_;
  }

  std::map<std::string, std::string> GuidanceModel::getParamsAsMap() const {
    return params_.asMap();
  }

  utexas_planning::RewardMetrics::Ptr GuidanceModel::getRewardMetricsAtEpisodeStart() const {
    RewardMetrics::Ptr metric(new RewardMetrics);
    metric->utility_loss = 0.0f;
    metric->time_loss = 0.0f;
    metric->reward_normalized = 0.0f;
    metric->utility_loss_normalized = 0.0f;
    metric->time_loss_normalized = 0.0f;
    return metric;
  }
} /* utexas_guidance */

CLASS_LOADER_REGISTER_CLASS(utexas_guidance::GuidanceModel, utexas_planning::GenerativeModel);
