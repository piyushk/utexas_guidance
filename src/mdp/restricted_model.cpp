#include <utexas_guidance/mdp/common.h>
#include <utexas_guidance/mdp/restricted_model.h>
#include <utexas_planning/common/exceptions.h>

namespace utexas_guidance {

  void RestrictedModel::init(const YAML::Node& params,
                             const std::string& output_directory,
                             const boost::shared_ptr<RNG>& rng) {
    GuidanceModel::init(params, output_directory, rng);
    restricted_model_params_.fromYaml(params);
  }

  std::string RestrictedModel::getName() const {
    return std::string("RestrictedModel");
  }

  void RestrictedModel::getActionsAtState(const utexas_planning::State::ConstPtr& state_base,
                                     std::vector<utexas_planning::Action::ConstPtr>& actions) const {

    ExtendedState::ConstPtr state = boost::dynamic_pointer_cast<const ExtendedState>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }

    // Heuristic 3: Compute which actions are allowed given a strict ordering.
    bool direct_person_allowed = !restricted_model_params_.h3_restrict_ordering ||
      (state->prev_action.type == WAIT || state->prev_action.type == DIRECT_PERSON);
    bool lead_person_allowed = direct_person_allowed || state->prev_action.type == LEAD_PERSON;
    bool release_robot_allowed = lead_person_allowed || state->prev_action.type == RELEASE_ROBOT;
    bool assign_robot_allowed = true;
    bool wait_allowed = true;

    // Heuristic 2: Set the max number of assignable locations.
    bool max_assigned_robots = restricted_model_params_.h2_max_assigned_robots;
    if (max_assigned_robots == MAX_ASSIGNED_ROBOTS_NOLIMIT) {
      max_assigned_robots = state->robots.size();
    } else if (max_assigned_robots == MAX_ASSIGNED_ROBOTS_SAME_AS_REQUESTS) {
      max_assigned_robots = std::min(state->robots.size(), state->requests.size());
    }

    actions.clear();
    int action_counter = 0;

    actions.resize(state->robots.size());
    int assigned_robots = 0;
    std::vector<bool> assignable_locations(num_vertices_, true);
    for (unsigned int robot = 0; robot < state->robots.size(); ++robot) {
      if (state->robots[robot].help_destination != NONE) {
        assigned_robots += 1;
        // Heuristic 4: Don't assign robots to locations to which they've already been assigned.
        if (restricted_model_params_.h4_disallow_multiple_assignments) {
          assignable_locations[state->robots[robot].help_destination] = false;
        }
        if (!(state->robots[robot].is_leading_person)) {
          actions[action_counter] = Action::Ptr(new Action(RELEASE_ROBOT, robot));
          ++action_counter;
        }
      }
    }

    // Heuristic 7: Prevent assignement to released locations.
    if (restricted_model_params_.h7_prevent_assignment_to_release_locs) {
      for (unsigned int released_loc_idx = 0; released_loc_idx < state->released_locations.size(); ++released_loc_idx) {
        assignable_locations[state->released_locations[released_loc_idx]] = false;
      }
    }

    // Heuristic 1: Autoselect robot for location.
    if (assign_robot_allowed) {
      if (restricted_model_params_.h1_autoselect_robot_for_destinations) {
        if (assigned_robots < max_assigned_robots) {
          actions.resize(action_counter + num_vertices_);
          for (unsigned int node = 0; node < num_vertices_; ++node) {
            if (assignable_locations[node]) {
              actions[action_counter] = Action::Ptr(new Action(ASSIGN_ROBOT, NONE, node));
              ++action_counter;
            }
          }
        }
      } else {
        actions.resize(action_counter + (state->robots.size() * num_vertices_));
        for (unsigned int robot = 0; robot < state->robots.size(); ++robot) {
          if ((assigned_robots < max_assigned_robots) &&
              (state->robots[robot].help_destination == NONE)) {
            actions.resize(action_counter + num_vertices_);
            for (unsigned int node = 0; node < num_vertices_; ++node) {
              if (assignable_locations[node]) {
                actions[action_counter] = Action::Ptr(new Action(ASSIGN_ROBOT, robot, node));
                ++action_counter;
              }
            }
          }
        }
      }
    }

    // Allow leading/guiding via all colocated robots.
    std::vector<std::pair<int, int> > robot_request_ids;
    getColocatedRobotRequestIds(*state, robot_request_ids);
    for (unsigned int idx_num = 0; idx_num < robot_request_ids.size(); ++idx_num) {
      int robot_id = robot_request_ids[idx_num].first;
      int request_id = robot_request_ids[idx_num].second;
      const RequestState& request = state->requests[request_id];
      if (direct_person_allowed) {
        actions.resize(action_counter + adjacent_vertices_map_[request.loc_node].size());
        for (unsigned int adj = 0; adj < adjacent_vertices_map_[request.loc_node].size(); ++adj) {
          actions[action_counter] =
            Action::Ptr(new Action(DIRECT_PERSON, robot_id, adjacent_vertices_map_[request.loc_node][adj], request_id));
          ++action_counter;
        }
        wait_allowed = !restricted_model_params_.h6_force_assistance;
      }
      if (lead_person_allowed) {
        actions.resize(action_counter + adjacent_vertices_map_[request.loc_node].size());
        for (unsigned int adj = 0; adj < adjacent_vertices_map_[request.loc_node].size(); ++adj) {
          actions[action_counter] =
            Action::Ptr(new Action(LEAD_PERSON, robot_id, adjacent_vertices_map_[request.loc_node][adj], request_id));
          ++action_counter;
        }
        wait_allowed = !restricted_model_params_.h6_force_assistance;
      }
    }

    // Wait is allowed at all times.
    if (wait_allowed) {
      Action::Ptr action(new Action(WAIT, 0, 0));
      actions.push_back(action);
      ++action_counter;
    }

  }


  void RestrictedModel::takeAction(const utexas_planning::State::ConstPtr& state_base,
                                   const utexas_planning::Action::ConstPtr& action_base,
                                   float& reward,
                                   const utexas_planning::RewardMetrics::Ptr& reward_metrics,
                                   utexas_planning::State::ConstPtr& next_state_base,
                                   int& depth_count,
                                   float& post_action_timeout,
                                   boost::shared_ptr<RNG> rng) const {

    ExtendedState::ConstPtr state = boost::dynamic_pointer_cast<const ExtendedState>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::ExtendedState");
    }

    Action::ConstPtr action = boost::dynamic_pointer_cast<const Action>(action_base);
    if (!action) {
      throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
    }

    Action::Ptr mutable_action(new Action(*action));
    if (mutable_action->type == ASSIGN_ROBOT && mutable_action->robot_id == NONE) {
      /* mutable_action->robot_id = selectBestRobotForTask(*state, action->node); */
    }

    GuidanceModel::takeAction(state_base,
                              boost::static_pointer_cast<const utexas_planning::Action>(mutable_action),
                              reward,
                              reward_metrics,
                              next_state_base,
                              depth_count,
                              post_action_timeout,
                              rng);

    ExtendedState::ConstPtr next_state = boost::dynamic_pointer_cast<const ExtendedState>(next_state_base);
    if (!next_state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::ExtendedState");
    }

    ExtendedState::Ptr mutable_next_state(new ExtendedState(*next_state));
    mutable_next_state->prev_action = *action;
    if (action->type == WAIT) {
      mutable_next_state->released_locations.clear();
    } else {
      mutable_next_state->released_locations = state->released_locations;
      if (action->type == RELEASE_ROBOT) {
        mutable_next_state->released_locations.push_back(state->robots[action->robot_id].help_destination);
      }
    }

    next_state_base = boost::static_pointer_cast<const utexas_planning::State>(mutable_next_state);
  }

  // int RestrictedModel::selectBestRobotForTask(const ExtendedState& state, int destination) const {

  //   std::vector<float> utility_loss(state.robots.size(), std::numeric_limits<float>::max());
  //   std::vector<float> time(state.robots.size(), std::numeric_limits<float>::max());

  //   float human_speed = motion_model_->getHumanSpeed();
  //   float robot_speed = motion_model_->getRobotSpeed();
  //   float time_to_destination = shortest_distances_[state.loc_node][destination] / human_speed;

  //   for (int i = 0; i < state.robots.size(); ++i) {
  //     if (state.robots[i].help_destination != NONE) {
  //       // This robot is already helping the human, and cannot be used.
  //       continue;
  //     }

  //     // Calculate this robot's time to its current destination.
  //     float orig_distance = getTrueDistanceTo(state.robots[i].loc_u,
  //                                             state.robots[i].loc_v,
  //                                             state.robots[i].loc_p,
  //                                             state.robots[i].tau_d,
  //                                             shortest_distances_);
  //     float original_time = orig_distance / robot_speed;

  //     // Calculate time if it is diverted to given location.
  //     float new_distance_1 = getTrueDistanceTo(state.robots[i].loc_u,
  //                                              state.robots[i].loc_v,
  //                                              state.robots[i].loc_p,
  //                                              destination,
  //                                              shortest_distances_);
  //     float new_distance_2 = shortest_distances_[destination][state.robots[i].tau_d];
  //     float new_time = std::max(new_distance_1 / robot_speed, time_to_destination) + new_distance_2 / robot_speed;
  //     if (new_distance_1 / robot_speed <= time_to_destination) {
  //       // The robot will reach there in time
  //       utility_loss[i] = state.robots[i].tau_u * (new_time - original_time);
  //     }
  //     time[i] = new_distance_1 / robot_speed;
  //   }

  //   if (*(std::min_element(utility_loss.begin(), utility_loss.end())) != std::numeric_limits<float>::max()) {
  //     // reach_in_time = true;
  //     return std::min_element(utility_loss.begin(), utility_loss.end()) - utility_loss.begin();
  //   }
  //   // reach_in_time = false;
  //   return std::min_element(time.begin(), time.end()) - time.begin();

  // }

} /* utexas_guidance */
