#include <utexas_guidance/mdp/transition_model.h>

namespace utexas_guidance {

  HumanDecisionModel::HumanDecisionModel(const bwi_mapper::Graph& graph, float decision_variance_multiplier = 1.0f) :
      graph_(graph), decision_variance_multiplier_(decision_variance_multiplier) {
    computeAdjacentVertices(adjacent_vertices_map_, graph_);
    computeAdjacentVerticesOnSameFloor(adjacent_vertices_on_same_floor_map_, graph_);
  }

  ~HumanDecisionModel::HumanDecisionModel() {}

  int HumanDecisionModel::getNextNode(const RequestState& state, RNG &rng) {

    // In case no help has been provided to the human, the expected direction of motion can be given by.
    float expected_direction_of_motion;
    float expected_variance;

    if (state.assist_type == LEAD_PERSON) {
      // Assume model is deterministic, and follows the system's assistance perfectly.
      return state.assist_loc;
    } else if (state.assist_type == DIRECT_PERSON) {
      if (!onSameFloor(state.assist_loc, state.loc_node, graph_)) {
        // Got directions to go to a different floor via an elevator, pretty obvious.
        return state.assist_loc;
      } else {
        expected_variance = 0.05f * decision_variance_multiplier_;
        expected_direction_of_motion = bwi_mapper::getNodeAngle(state.loc_node, state.assist_loc, graph_);
      }
    } else /* no assistance provided. */ {
      // Do nothing. Use default values for expected variance.
      if (state.loc_prev == state.loc_node) {
        // This can only mean that this is the start state (as the person does not have loc_prev set), and the policy
        // called Wait without first calling LEAD or DIRECT, which is a terrible action since some free help went to
        // waste. Assume that the person will move completely randomly (including changing floors).
        int rand_idx = rng.randomInt(adjacent_vertices_map_[state.loc_node].size() - 1);
        return adjacent_vertices_map_[state.loc_node][rand_idx];
      } else if (!onSameFloor(state.loc_prev, state.loc_node, graph_)) {
        // The person changed floors, but no assistance was provided after changing floors, the person will likely
        // go anywhere, apart from going back into the elevator.

        int rand_idx = rng.randomInt(adjacent_vertices_on_same_floor_map_[state.loc_node].size() - 1);
        return adjacent_vertices_on_same_floor_map_[state.loc_node][rand_idx];
      } else {
        expected_direction_of_motion = bwi_mapper::getNodeAngle(state.loc_prev, state.loc_node, graph_);
        expected_variance = 0.1f * decision_variance_multiplier_;
      }
    }

    // Now assume that the person moves to one the adjacent locations
    float weight_sum = 0;
    std::vector<float> weights;
    BOOST_FOREACH(int adj, adjacent_vertices_on_same_floor_map_[state.loc_node]) {
      float next_state_direction = bwi_mapper::getNodeAngle(state.loc_node, adj, graph_);
      float angle_difference = getAbsoluteAngleDifference(next_state_direction, expected_direction_of_motion);

      // Compute the probability of this state
      float weight = expf(-powf(angle_difference, 2) / (2 * expected_variance));
      weights.push_back(weight);
      weight_sum += weight;
    }

    float probability_sum = 0;
    std::vector<float> probabilities;
    /* std::cout << "Transition probabilities: " << std::endl; */
    for (size_t probability_counter = 0; probability_counter < weights.size(); ++probability_counter) {
      /* std::cout << weights[probability_counter] << " " << weight_sum << std::endl; */
      double probability = 0.99 * (weights[probability_counter] / weight_sum) + 0.01 * (1.0f / weights.size());
      probability_sum += probability;
      if (probability_counter == weights.size() - 1) {
        // Account for floating point errors. No surprises!
        probability += 1.0f - probability_sum;
      }
      probabilities.push_back(probability);
      // std::cout << "  to " <<
      //   adjacent_vertices_on_same_floor_map_[next_state.graph_id][probability_counter] <<
      //   ": " << probability << std::endl;
    }

    return adjacent_vertices_on_same_floor_map_[state.loc_node][rng.select(probabilities)];
  }

  RandomTaskGenerationModel::RandomTaskGenerationModel(const std::vector<int> robot_home_base,
                                                       const bwi_mapper::Graph &graph,
                                                       float task_utility,
                                                       float task_time,
                                                       bool home_base_only) :
      robot_home_base_(robot_home_base),
      task_utility_(task_utility),
      task_time_(task_time),
      home_base_only_(home_base_only) {

    cacheNewGoalsByDistance(graph);
  }

  ~RandomTaskGenerationModel::RandomTaskGenerationModel() {}

  void RandomTaskGenerationModel::generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng) {
    // Optimized!!!
    if (home_base_only_) {
      robot.tau_d = robot_home_base_[robot_id];
    } else {
      int idx = robot_home_base_[robot_id];
      int graph_distance = rng.poissonInt(1);
      while(graph_distance >= goals_by_distance_[idx].size()) {
        graph_distance = rng.poissonInt(1);
      }
      std::vector<int>& possible_goals = goals_by_distance_[idx][graph_distance];
      robot.tau_d = *(possible_goals.begin() + rng.randomInt(possible_goals.size() - 1));
    }
    robot.tau_t = 0.0f;
    robot.tau_total_task_time = task_time_;
    robot.tau_u = task_utility_;
  }

  void RandomTaskGenerationModel::cacheNewGoalsByDistance(const bwi_mapper::Graph &graph) {

    // Select a random goal on the same floor.
    std::map<int, std::vector<int> > adjacent_vertices_map;
    computeAdjacentVerticesOnSameFloor(adjacent_vertices_map, graph);
    int num_vertices = boost::num_vertices(graph);

    goals_by_distance_.clear();
    goals_by_distance_.resize(num_vertices);
    for (int idx = 0; idx < num_vertices; ++idx) {
      std::set<int> closed_set;
      std::vector<int> current_set;
      current_set.push_back(idx);
      while (current_set.size() != 0) {
        goals_by_distance_[idx].push_back(current_set);
        std::set<int> open_set;
        BOOST_FOREACH(int c, current_set) {
          BOOST_FOREACH(int a, adjacent_vertices_map[c]) {
            if (std::find(closed_set.begin(), closed_set.end(), a) ==
                closed_set.end()) {
              open_set.insert(a);
            }
          }
        }
        closed_set.insert(current_set.begin(), current_set.end());
        current_set = std::vector<int>(open_set.begin(), open_set.end());
      }
    }
  }

  FixedTaskGenerationModel::FixedTaskGenerationModel(const std::string& task_file,
                                                     float task_utility,
                                                     float task_time) :
      task_utility_(task_utility), task_time_ {
    readFixedTasksFiles(task_file);
  }

  ~FixedTaskGenerationModel::FixedTaskGenerationModel() {}

  void FixedTaskGenerationModel::generateNewTaskForRobot(int robot_id, RobotState& robot, RNG& ) {
    // Optimized!!!
    if (robot.tau_d == TASK_UNINITIALIZED) {
      robot.tau_d = tasks_[robot_id][0];
    } else {
      bool task_found = false;
      for (int i = 0; i < tasks_[robot_id].size(); ++i) {
        if (robot.tau_d == tasks_[robot_id][i]) {
          int next_task_id = (i + 1) % tasks_[robot_id].size();
          robot.tau_d = tasks_[robot_id][next_task_id];
          task_found = true;
          break;
        }
      }
      if (!task_found) {
        throw utexas_guidance::IncorrectUsageException("Robot assigned task outside specified list in " +
                                                       "FixedGenerationTaskModel. Cannot assign new task!");
      }
    }

    robot.tau_t = 0.0f;
    robot.tau_total_task_time = task_time_;
    robot.tau_u = task_utility_;
  }

  MotionModel::MotionModel(const bwi_mapper::Graph graph,
                           float avg_robot_speed,
                           float avg_human_speed,
                           float avg_human_elevator_speed,
                           float avg_robot_elevator_speed) :
      graph_(graph),
      robot_speed_(avg_robot_speed),
      human_speed_(avg_human_speed),
      elevator_human_speed_(avg_human_elevator_speed),
      elevator_robot_speed_(avg_robot_elevator_speed) {
    computeShortestPath(shortest_distances_, shortest_paths_, graph_);
  }

  ~MotionModel::MotionModel() {}

  /**
   * \brief  Given a human decision model, as well as a task generation model, move the system state forwards in time
   *         until a human reaches a decision point. This function is called when the Wait action is used.
   */
  void MotionModel::move(State& state,
                    const HumanDecisionModel::ConstPtr& human_decision_model,
                    const TaskGenerationModel::ConstPtr& task_generation_model,
                    RNG &rng,
                    float &total_time) {

    total_time = std::numeric_limits<float>::max();
    std::vector<float> human_speeds(state.requests.size());

    /* First compute new human locations for humans that have exactly reached a particular graph node. */
    for (int i = 0; i < state.requests.size(); ++i) {
      RequestState& rs = state.requests[i];
      if (rs.loc_p == 1.0f) {
        // Choose the next node for this person
        rs.loc_p = 0.0f;
        rs.loc_prev = rs.loc_node;
        rs.loc_node = human_decision_model->getNextNode(rs, rng);
      }

      // Figure out what pace the human is gonna move in.
      if (!onSameFloor(rs.loc_prev, rs.loc_node, graph_)) {
        human_speeds[i] = (rs.assist_type == LEAD_PERSON) ? elevator_robot_speed_ : elevator_human_speed_;
      } else {
        human_speeds[i] = (rs.assist_type == LEAD_PERSON) ? robot_speed_ : human_speed_;
      }

      float time_to_dest = (1.0f - rs.loc_p) * shortest_distances_[rs.loc_prev][rs.loc_node] / human_speed;
      total_time = std::min(total_time, time_to_dest);
    }

    /* total_time now reflects when the first human is going to reach a destination node. Increment all humans by
     * this time. */
    for (int i = 0; i < state.requests.size(); ++i) {
      RequestState& rs = state.requests[i];
      float distance_covered = total_time * human_speeds[i];
      rs.loc_p += distance_covered / shortest_distances_[rs.loc_node][next_node]; // Should be atmost 1.

      // Atleast one of the following should be 1, but no real need to check that.
      if (rs.loc_p > 1.0f - 1e-6f) {
        rs.loc_p == 1.0f;
        rs.assist_type = NONE;
        rs.assist_loc = NONE;
      }
    }

    /* Move all robots using total_time.*/

    // Optimized!!!
    for (int i = 0; i < state.robots.size(); ++i) {
      RobotState& robot = state.robots[i];

      float robot_time_remaining = total_time;
      bool robot_in_use = robot.help_destination != NONE;
      int destination = (robot_in_use) ? robot.help_destination : robot.tau_d;

      // Get shortest path to destination, and figure out how much distance
      // of that path we can cover
      while (robot_time_remaining > 0.0f) {

        /* std::cout << robot.graph_id << " " << robot.precision << " " << destination << " " << robot.other_graph_node << std::endl; */
        // Check if the robot has already reached it's destination.
        if (isRobotExactlyAt(robot, destination)) {
          if (robot_in_use) {
            // Won't be doing anything more with this robot until the robot gets released.
            robot_time_remaining = 0.0f;
          } else {
            // The robot has reached its service task destination and is in the middle of performing the service
            // task.
            robot.tau_t += robot_time_remaining;
            robot_time_remaining = 0.0f;

            // Check if the robot can complete this task and move on to the next task.
            if (robot.tau_t > robot.tau_total_task_time) {
              robot_time_remaining = robot.tau_t - robot.tau_total_task_time;
              task_generation_model->generateNewTaskForRobot(i, robot, rng);
              // Update the destination for this robot.
              destination = robot.tau_d;
            }
          }
        } else {

          // Just in case the robot is exactly at v, let's switch the position around so that the robot is
          // exactly at u.
          if (isRobotExactlyAt(robot, robot.loc_v)) {
            robot.loc_u = robot.loc_v;
            robot.loc_p = 0.0f;
          }

          // If the robot is exactly at u, then compute the shortest path to the goal and move the robot along
          // this shortest path.
          if (isRobotExactlyAt(robot, robot.loc_u)) {
            // Move robot along shortest path from u.
            std::vector<size_t>& shortest_path = shortest_paths_[robot.loc_u][destination];
            robot.loc_v = shortest_path[0];
            bool in_elevator = !onSameFloor(robot.loc_u, robot.loc_v);
            float robot_speed = (in_elevator) ? elevator_robot_speed_ : robot_speed_;
            robot.loc_p += (robot_time_remaining * robot_speed) / shortest_distances_[robot.loc_u][robot.loc_v];
            robot_time_remaining = 0.0f;
            if (robot.loc_p > 1.0f) {
              robot_time_remaining =
                ((robot.loc_p - 1.0f) * (shortest_distances_[robot.loc_u][robot.loc_v])) / robot_speed;
              robot.loc_p = 1.0f;
            }
          } else {
            // The robot is somewhere in the middle of u and v.
            bool shortest_path_through_u = isShortestPathThroughLocU(robot.loc_u,
                                                                     robot.loc_v,
                                                                     robot.loc_p,
                                                                     destination,
                                                                     shortest_distances_);

            bool in_elevator = !onSameFloor(robot.loc_u, robot.loc_v);
            float robot_speed = (in_elevator) ? elevator_robot_speed_ : robot_speed_;

            if (shortest_path_through_u) {
              // Move robot to u.
              robot.loc_p -= (robot_time_remaining * robot_speed) / shortest_distances_[robot.loc_u][robot.loc_v];
              robot_time_remaining = 0.0f;
              if (robot.loc_p < 0.0f) {
                robot_time_remaining =
                  ((-robot.loc_p) * (shortest_distances_[robot.loc_u][robot.loc_v])) / robot_speed;
                robot.loc_p = 0.0f;
              }
            } else {
              // Move robot to v.
              robot.loc_p += (robot_time_remaining * robot_speed) / shortest_distances_[robot.loc_u][robot.loc_v];
              robot_time_remaining = 0.0f;
              if (robot.loc_p > 1.0f) {
                robot_time_remaining =
                  ((robot.loc_p - 1.0f) * (shortest_distances_[robot.loc_u][robot.loc_v])) / robot_speed;
                robot.loc_p = 1.0f;
              }
            }
          }
        }
      }

      // Account for any floating point errors, especially while leading a person.
      if (robot.loc_p > 1.0f - 1e-6f) {
        robot.loc_p = 1.0f;
      }

      if (robot.loc_p < 1e-6f) {
        robot.loc_p = 0.0f;
      }

    }

  }

  float MotionModel::getHumanSpeed() {
    return human_speed_;
  }

  float MotionModel::getHumanSpeedInElevator() {
    return elevator_human_speed_;
  }

  float MotionModel::getRobotSpeed() {
    return robot_speed_;
  }

  float MotionModel::getRobotSpeedInElevator() {
    return elevator_robot_speed_;
  }


} /* bwi_guidance */
