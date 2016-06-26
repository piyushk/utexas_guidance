#include <utexas_guidance/mdp/restricted_model.h>

/*
 * max assigned robots.
 * max distance away
 * structure (DIRECTS then leads RELEASES then ASSIGNS then LEADS then WAIT)
 * Don't assign robot to location where it's been released from.
 * ensure if robot is colocated, then lead or guide is called prior to wait.
 * do not assign robot to a location where another robot is.
 *
 * only one we can't do is releasing a robot after directing a human, as that one's a bit iffy and will require saving
 * more information. add prev_action to guidance_model, and add heuristics to that instead of doing the separate class.
 * It'll make life easier. just make sure assigning a single robot instead of having to choose a robot  does not conflate time and distance.
 */

namespace utexas_guidance {

  std::string RestricteModel::getName() const {
    return std::string("RestrictedModel");
  }

  void RestrictedModel::getActionsAtState(const utexas_planning::State::ConstPtr& state_base,
                                          std::vector<Action>& actions) {

    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }

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

    // Allow robots to be assigned/released at any given location.
    for (unsigned int robot = 0; robot < state->robots.size(); ++robot) {
      if (state->robots[robot].help_destination == NONE) {
        actions.resize(action_counter + num_vertices_);
        for (unsigned int node = 0; node < num_vertices_; ++node) {
          actions[action_counter] = Action::Ptr(new Action(ASSIGN_ROBOT, robot, node));
          ++action_counter;
        }
      } else if (!(state->robots[robot].is_leading_person)) {
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
    // If direct and lead are legal actions, remove other actions.

    // Check if leading a person is allowed here.
    if (colocated_robot_id != NONE) {
      actions.resize(action_counter + adjacent_vertices_map_[state.loc_node].size());
      for (unsigned int adj = 0; adj < adjacent_vertices_map_[state.loc_node].size(); ++adj) {
        actions[action_counter] =
          Action(LEAD_PERSON, colocated_robot_id, adjacent_vertices_map_[state.loc_node][adj]);
        ++action_counter;
      }
    }

    // Check if assigning a robot is allowed here.
    std::vector<int> unassignable_locations;
    for (unsigned int robot = 0; robot < state.robots.size(); ++robot) {
      if (state.robots[robot].help_destination != NONE) {
        unassignable_locations.push_back(state.robots[robot].help_destination);
      }
    }

    if (unassignable_locations.size() < params_.max_assigned_robots) {
      unassignable_locations.insert(unassignable_locations.end(), state.released_locations.begin(), state.released_locations.end());
      for (unsigned int adj = 0; adj < action_vertices_map_[state.loc_node].size(); ++adj) {
        int node = action_vertices_map_[state.loc_node][adj];
        if (std::find(unassignable_locations.begin(), unassignable_locations.end(), node) ==
            unassignable_locations.end()) {
          // Note that the robot id is unknown here, so we'll use -1.
          actions.push_back(Action(ASSIGN_ROBOT, -1, node));
          ++action_counter;
        }
      }
    }

    // Check if releasing a robot is allowed here.
    if (state.prev_action.type == WAIT ||
        state.prev_action.type == RELEASE_ROBOT ||
        state.prev_action.type == DIRECT_PERSON) {
      for (unsigned int robot = 0; robot < state.robots.size(); ++robot) {
        if (state.robots[robot].help_destination != NONE) {
          actions.push_back(Action(RELEASE_ROBOT, robot));
        }
      }
    }

    // Finally see if directing a person is allowed here.
    if (colocated_robot_id != NONE && state.prev_action == WAIT) {
      actions.resize(action_counter + adjacent_vertices_map_[state.loc_node].size());
      for (unsigned int adj = 0; adj < adjacent_vertices_map_[state.loc_node].size(); ++adj) {
        actions[action_counter] =
          Action(DIRECT_PERSON, colocated_robot_id, adjacent_vertices_map_[state.loc_node][adj]);
        ++action_counter;
      }
    }

  }

  void RestrictedModel::getFirstAction(const ExtendedState &state, Action &action) {
    // This function cannot be optimized without maintaining state. Only present for legacy
    // reasons. Do not call it! For this reason, here is a very suboptimal implementation.
    std::vector<Action> actions;
    getActionsAtState(state, actions);
    action = actions[0];
  }

  bool RestrictedModel::getNextAction(const ExtendedState &state, Action &action) {
    // This function cannot be optimized without maintaining state. Only present for legacy
    // reasons. Do not call it!
    std::vector<Action> actions;
    getActionsAtState(state, actions);
    for (unsigned int action_id = 0; action_id < actions.size() - 1; ++action_id) {
      if (actions[action_id] == action) {
        action = actions[action_id + 1];
        return true;
      }
    }
    return false;
  }

  void RestrictedModel::getAllActions(const ExtendedState &state, std::vector<Action>& actions) {
    getActionsAtState(state, actions);
  }

  void RestrictedModel::takeAction(const ExtendedState &state,
                                   const Action &action,
                                   float &reward,
                                   ExtendedState &next_state,
                                   bool &terminal,
                                   int &depth_count,
                                   boost::shared_ptr<RNG> rng) {

    float unused_utility_loss, unused_time_loss;
    std::vector<State> unused_frame_vector;
    takeAction(state,
               action,
               reward,
               next_state,
               terminal,
               depth_count,
               rng,
               unused_time_loss,
               unused_utility_loss,
               unused_frame_vector);
  }

  void RestrictedModel::takeAction(const ExtendedState &state,
                                   const Action &action,
                                   float &reward,
                                   ExtendedState &next_state,
                                   bool &terminal,
                                   int &depth_count,
                                   boost::shared_ptr<RNG> &rng,
                                   float &time_loss,
                                   float &utility_loss,
                                   std::vector<State> &frame_vector) {


    Action mapped_action = action;
    if (mapped_action.type == ASSIGN_ROBOT && mapped_action.robot_id == -1) {
      bool unused_reach_in_time;
      mapped_action.robot_id = selectBestRobotForTask(state,
                                                      action.node,
                                                      avg_human_speed_,
                                                      avg_robot_speed_,
                                                      shortest_distances_,
                                                      unused_reach_in_time);
    }

    base_model_->takeAction(state,
                            mapped_action,
                            reward,
                            next_state,
                            terminal,
                            depth_count,
                            rng,
                            time_loss,
                            utility_loss,
                            frame_vector);

    // next_state.robot_provided_help = state.robot_provided_help;
    // if (action.type == RELEASE_ROBOT) {
    //   std::vector<int>::iterator it =
    //     std::find(next_state.robot_provided_help.begin(), next_state.robot_provided_help.end(), action.robot_id);
    //   if (it == next_state.robot_provided_help.end()) {
    //     reward -= 2.0f;
    //   } else {
    //     next_state.robot_provided_help.erase(it);
    //   }
    // }

    // if (action.type == LEAD_PERSON || action.type == DIRECT_PERSON) {
    //   next_state.robot_provided_help.push_back(action.robot_id);
    // }

    // float max_robot_utility = 0.0f;
    // for (int r = 0; r < state.robots.size(); ++r) {
    //   if (state.robots[r].help_destination != NONE && state.robots[r].tau_u > max_robot_utility) {
    //     max_robot_utility = state.robots[r].tau_u;
    //   }
    // }

    // if (max_robot_utility > 0.0f && utility_loss < 1e-3 && action.type == WAIT && state.assist_type != LEAD_PERSON) {
    //   reward -= 1000.0f;
    // }

    next_state.prev_action = action;
    if (action.type == WAIT) {
      next_state.released_locations.clear();
    } else if (action.type == RELEASE_ROBOT) {
      next_state.released_locations.push_back(state.robots[action.robot_id].help_destination);
    } else {
      next_state.released_locations = state.released_locations;
    }
  }

  void RestrictedModel::drawState(const State& state, cv::Mat& image) {
    base_model_->drawState(state, image);
  }

} /* utexas_guidance */
