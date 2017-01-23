#include <boost/foreach.hpp>

#include <utexas_guidance/mdp/transition_model.h>
#include <utexas_guidance/exceptions.h>

#include <yaml-cpp/yaml.h>

namespace utexas_guidance {

  HumanDecisionModel::HumanDecisionModel(const Graph& graph, 
                                         bool is_deterministic,
                                         float decision_variance_multiplier, 
                                         float decision_mean_noise_multiplier) :
      graph_(graph), 
      is_deterministic_(is_deterministic),
      decision_variance_multiplier_(decision_variance_multiplier),
      decision_mean_noise_multiplier_(decision_mean_noise_multiplier) {
    getAllAdjacentVertices(adjacent_vertices_map_, graph_);
    getAllAdjacentVerticesOnSameFloor(adjacent_vertices_on_same_floor_map_, graph_);
  }

  HumanDecisionModel::~HumanDecisionModel() {}

  int HumanDecisionModel::getNextNode(const RequestState& state, RNG &rng) const {

    /* std::cout << "getNextNode" << std::endl; */
    // In case no help has been provided to the human, the expected direction of motion can be given by.
    float expected_direction_of_motion;
    float expected_variance;

    if (state.assist_type == LEAD_PERSON) {
      // Assume model is deterministic, and follows the system's assistance perfectly.
      /* std::cout << "1" << std::endl; */
      return state.assist_loc;
    } else if (state.assist_type == DIRECT_PERSON) {
      /* std::cout << "2" << std::endl; */
      if (!onSameFloor(state.assist_loc, state.loc_node, graph_)) {
        // Got directions to go to a different floor via an elevator, pretty obvious.
        return state.assist_loc;
      } else {
        expected_variance = 0.05f * decision_variance_multiplier_;
        expected_direction_of_motion = getNodeAngle(state.loc_node, state.assist_loc, graph_) +
          decision_mean_noise_multiplier_ * rng.normalFloat(0.0f, 1.0f);
      }
    } else /* no assistance provided. */ {
      // Do nothing. Use default values for expected variance.
      /* std::cout << state.loc_prev << " " << state.loc_node << std::endl; */
      if (state.loc_prev == state.loc_node) {

        /* std::cout << "in here" << std::endl; */
        // This can only mean that this is the start state (as the person does not have loc_prev set), and the policy
        // called Wait without first calling LEAD or DIRECT, which is a terrible action since some free help went to
        // waste. Assume that the person will move completely randomly (including changing floors).
        // Pseudo-random for different graph id, but deterministic given a graph id.
        int rand_idx = state.loc_node % adjacent_vertices_map_[state.loc_node].size();
        if (!is_deterministic_) {
          rand_idx = rng.randomInt(adjacent_vertices_map_[state.loc_node].size() - 1);
        }
        return adjacent_vertices_map_[state.loc_node][rand_idx];
      } else if (!onSameFloor(state.loc_prev, state.loc_node, graph_)) {
        // The person changed floors, but no assistance was provided after changing floors, the person will likely
        // go anywhere, apart from going back into the elevator.

        /* std::cout << "in here2" << std::endl; */
        // Pseudo-random for different graph id, but deterministic given a graph id.
        int rand_idx = state.loc_node % adjacent_vertices_on_same_floor_map_[state.loc_node].size();
        if (!is_deterministic_) {
          rand_idx = rng.randomInt(adjacent_vertices_on_same_floor_map_[state.loc_node].size() - 1);
        }
        return adjacent_vertices_on_same_floor_map_[state.loc_node][rand_idx];
      } else {
        expected_direction_of_motion = getNodeAngle(state.loc_prev, state.loc_node, graph_) +
          decision_mean_noise_multiplier_ * rng.normalFloat(0.0f, 1.0f);
        expected_variance = 0.1f * decision_variance_multiplier_;
      }
    }

    // Now assume that the person moves to one the adjacent locations
    float weight_sum = 0;
    std::vector<float> weights;
    BOOST_FOREACH(int adj, adjacent_vertices_on_same_floor_map_[state.loc_node]) {
      float next_state_direction = getNodeAngle(state.loc_node, adj, graph_);
      float angle_difference = getAbsoluteAngleDifference(next_state_direction, expected_direction_of_motion);

      // Compute the probability of this state
      float weight = expf(-powf(angle_difference, 2) / (2 * expected_variance));
      weights.push_back(weight);
      weight_sum += weight;
    }

    float probability_sum = 0;
    std::vector<float> probabilities;

    float max_probability = 0;
    int max_probability_idx = 0;
    /* std::cout << "Transition probabilities: " << std::endl; */
    for (size_t probability_counter = 0; probability_counter < weights.size(); ++probability_counter) {
      /* std::cout << weights[probability_counter] << " " << weight_sum << std::endl; */
      double probability = 0.99 * (weights[probability_counter] / weight_sum) + 0.01 * (1.0f / weights.size());
      probability_sum += probability;
      if (probability_counter == (weights.size() - 1)) {
        // Account for floating point errors. No surprises!
        probability += 1.0f - probability_sum;
      }
      probabilities.push_back(probability);

      if (probability > max_probability) {
        max_probability = probability;
        max_probability_idx = probability_counter;
      }
      // std::cout << "  to " <<
      //   adjacent_vertices_on_same_floor_map_[state.loc_node][probability_counter] <<
      //   ": " << probability << std::endl;
    }

    if (is_deterministic_) {
      return adjacent_vertices_on_same_floor_map_[state.loc_node][max_probability_idx];
    } 
    return adjacent_vertices_on_same_floor_map_[state.loc_node][rng.select(probabilities)];
  }

  TaskGenerationModel::~TaskGenerationModel() {}

  RandomTaskGenerationModel::RandomTaskGenerationModel(const std::string& robot_home_base_file,
                                                       const Graph &graph,
                                                       float task_utility,
                                                       float task_time,
                                                       bool home_base_only) :
      task_utility_(task_utility),
      task_time_(task_time),
      home_base_only_(home_base_only) {
    cacheNewGoalsByDistance(graph);

    try {
      YAML::Node config = YAML::LoadFile(robot_home_base_file);
      for (unsigned int i = 0; i < config.size(); ++i) {
        robot_home_base_.push_back(config[i]["home_base"].as<int>());
      }
    } catch (YAML::Exception& e) {
      throw IncorrectUsageException(std::string("Failed loading RandomTaskGenerationModel from file ") +
                                    robot_home_base_file + std::string("with YAML error: ") + e.what());
    }

  }

  RandomTaskGenerationModel::~RandomTaskGenerationModel() {}

  void RandomTaskGenerationModel::generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng) const {
    // Optimized!!!
    if (home_base_only_) {
      robot.tau_d = robot_home_base_[robot_id];
    } else {
      int idx = robot_home_base_[robot_id];
      int graph_distance = rng.poissonInt(1);
      while(graph_distance >= goals_by_distance_[idx].size()) {
        graph_distance = rng.poissonInt(1);
      }
      const std::vector<int>& possible_goals = goals_by_distance_[idx][graph_distance];
      robot.tau_d = *(possible_goals.begin() + rng.randomInt(possible_goals.size() - 1));
    }
    robot.tau_t = 0.0f;
    robot.tau_total_task_time = task_time_;
    robot.tau_u = task_utility_;
  }

  void RandomTaskGenerationModel::cacheNewGoalsByDistance(const Graph &graph) {

    // Select a random goal on the same floor.
    std::vector<std::vector<int> > adjacent_vertices_map;
    getAllAdjacentVerticesOnSameFloor(adjacent_vertices_map, graph);
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

  int RandomTaskGenerationModel::getNumRobots() const {
    return robot_home_base_.size();
  }

  int RandomTaskGenerationModel::getStartingLocationForRobot(int robot_idx) const {
    return robot_home_base_[robot_idx];
  }

  FixedTaskGenerationModel::FixedTaskGenerationModel(const std::string& task_file,
                                                     float task_utility,
                                                     float task_time) :
      task_utility_(task_utility), task_time_(task_time) {

    try {
      YAML::Node config = YAML::LoadFile(task_file);
      for (unsigned int robot_idx = 0; robot_idx < config.size(); ++robot_idx) {
        std::vector<int> path;
        for (unsigned int graph_idx = 0; graph_idx < config[robot_idx]["path"].size(); ++graph_idx) {
          path.push_back(config[robot_idx]["path"][graph_idx].as<int>());
        }
        tasks_.push_back(path);
      }
    } catch (YAML::Exception& e) {
      throw IncorrectUsageException(std::string("Failed loading FixedTaskGenerationModel from file ") +
                                    task_file + std::string("with YAML error: ") + e.what());
    }

  }

  FixedTaskGenerationModel::~FixedTaskGenerationModel() {}

  void FixedTaskGenerationModel::generateNewTaskForRobot(int robot_id, RobotState& robot, RNG& ) const {
    // Optimized!!!
    if (robot.tau_d == NONE) {
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
        throw utexas_guidance::IncorrectUsageException("Robot assigned task outside specified list in "
                                                       "FixedTaskGenerationModel. Cannot assign new task!");
      }
    }

    robot.tau_t = 0.0f;
    robot.tau_total_task_time = task_time_;
    robot.tau_u = task_utility_;
  }

  int FixedTaskGenerationModel::getNumRobots() const {
    return tasks_.size();
  }

  int FixedTaskGenerationModel::getStartingLocationForRobot(int robot_idx) const {
    return tasks_[robot_idx][0];
  }

  MotionModel::MotionModel(const Graph graph,
                           float avg_human_speed,
                           float avg_robot_speed,
                           float avg_human_elevator_speed,
                           float avg_robot_elevator_speed,
                           bool terminate_with_robot_present) :
      graph_(graph),
      human_speed_(avg_human_speed),
      robot_speed_(avg_robot_speed),
      elevator_human_speed_(avg_human_elevator_speed),
      elevator_robot_speed_(avg_robot_elevator_speed),
      terminate_with_robot_present_(terminate_with_robot_present) {
    getAllShortestPaths(shortest_distances_, shortest_paths_, graph_);
  }

  MotionModel::~MotionModel() {}

  static bool requestComplete(const RequestState& rs) {
    return (rs.loc_node == rs.goal && rs.loc_p == 1.0f);
  }

  /**
   * \brief  Given a human decision model, as well as a task generation model, move the system state forwards in time
   *         until a human reaches a decision point. This function is called when the Wait action is used.
   */
  void MotionModel::move(State& state,
                    const HumanDecisionModel::ConstPtr& human_decision_model,
                    const TaskGenerationModel::ConstPtr& task_generation_model,
                    RNG &rng,
                    float &total_time,
                    float max_time) const {

    total_time = std::numeric_limits<float>::max();
    std::vector<float> human_speeds(state.requests.size());

    /* First compute new human locations for humans that have exactly reached a particular graph node. */
    for (int request_id = 0; request_id < state.requests.size(); ++request_id) {
      RequestState& rs = state.requests[request_id];
      if (rs.loc_p == 1.0f) {
        // Choose the next node for this person
        int loc_prev = rs.loc_node;
        rs.loc_node = human_decision_model->getNextNode(rs, rng);
        if (rs.loc_node != loc_prev) {
          rs.loc_p = 0.0f;
        }
        rs.loc_prev = loc_prev;
      }

      // Figure out what pace the human is gonna move in.
      if (!onSameFloor(rs.loc_prev, rs.loc_node, graph_)) {
        human_speeds[request_id] = (rs.assist_type == LEAD_PERSON) ? elevator_robot_speed_ : elevator_human_speed_;
      } else {
        human_speeds[request_id] = (rs.assist_type == LEAD_PERSON) ? robot_speed_ : human_speed_;
      }

      float time_to_dest = (1.0f - rs.loc_p) * shortest_distances_[rs.loc_prev][rs.loc_node] / human_speeds[request_id];
      if (rs.assist_type == LEAD_PERSON && rs.loc_p == 1.0f && rs.loc_node == rs.assist_loc) {
        time_to_dest = rs.wait_time_left;
      }

      // std::cout << shortest_distances_[rs.loc_prev][rs.loc_node] << std::endl;
      // std::cout << human_speeds[request_id] << std::endl;
      total_time = std::min(total_time, time_to_dest);
    }

    if (max_time > 0.0f) {
      total_time = std::min(max_time, total_time);
    }

    std::vector<int> robot_release_candidate;
    /* total_time now reflects when the first human is going to reach a destination node. Increment all humans by
     * this time. */
    for (int request_id = 0; request_id < state.requests.size(); ++request_id) {
      RequestState& rs = state.requests[request_id];
      if (rs.loc_prev != rs.loc_node) {
        float distance_covered = total_time * human_speeds[request_id];
        rs.loc_p += distance_covered / shortest_distances_[rs.loc_prev][rs.loc_node]; // Should be atmost 1.
      } else {
        rs.wait_time_left -= total_time;
        if (rs.wait_time_left < 1e-6f) {
          rs.wait_time_left = 0.0f;
        }
      }

      // Atleast one of the following should be 1, but no real need to check that.
      if (rs.loc_p > 1.0f - 1e-6f) {
        rs.loc_p = 1.0f;
        if (rs.assist_type == LEAD_PERSON) {
          if (rs.wait_time_left == 0.0f) {// && requestComplete(rs)) {
            // Find robot that helped this person.
            for (int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
              RobotState& robot = state.robots[robot_id];
              if (robot.is_leading_person && robot.help_destination == rs.loc_node) {
                robot_release_candidate.push_back(robot_id);
              }
            }
            rs.assist_type = NONE;
            rs.assist_loc = NONE;
          }
        } else {
          rs.assist_type = NONE;
          rs.assist_loc = NONE;
        }
      }
    }

    /* Move all robots using total_time.*/

    // Optimized!!!
    for (int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
      RobotState& robot = state.robots[robot_id];

      float robot_time_remaining = total_time;
      bool robot_in_use = robot.help_destination != NONE;
      int destination = (robot_in_use) ? robot.help_destination : robot.tau_d;

      // Get shortest path to destination, and figure out how much distance
      // of that path we can cover
      while (robot_time_remaining > 0.0f) {

        /* std::cout << robot.graph_id << " " << robot.precision << " " << destination << " " << robot.other_graph_node << std::endl; */
        // Check if the robot has already reached it's destination.
        if (isRobotExactlyAt(robot, destination)) {
          if (robot_in_use && destination != robot.tau_d) {
            // Won't be doing anything more with this robot until the robot gets released.
            robot_time_remaining = 0.0f;
          } else {
            // The robot has reached its service task destination and is in the middle of performing the service
            // task.
            robot.tau_t += robot_time_remaining;
            robot_time_remaining = 0.0f;

            // Check if the robot can complete this task and move on to the next task.
            if (robot.tau_t >= robot.tau_total_task_time) {
              robot_time_remaining = robot.tau_t - robot.tau_total_task_time;
              task_generation_model->generateNewTaskForRobot(robot_id, robot, rng);
              // Update the destination for this robot.
              if (!robot_in_use) {
                destination = robot.tau_d;
              }
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
            const std::vector<int>& shortest_path = shortest_paths_[robot.loc_u][destination];
            robot.loc_v = shortest_path[0];
            bool in_elevator = !onSameFloor(robot.loc_u, robot.loc_v, graph_);
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

            bool in_elevator = !onSameFloor(robot.loc_u, robot.loc_v, graph_);
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

      // If a robot is exactly at his help destination, remove specific request_id allocation.
      if (std::find(robot_release_candidate.begin(), robot_release_candidate.end(), robot_id) !=
          robot_release_candidate.end()) {
        robot.is_leading_person = false;
        robot.help_destination = NONE;
      }
    }

    /* Remove all completed requests. */

    // Requests can only complete if a robot was at the goal as well.
    if (terminate_with_robot_present_) {
      for (int request_id = state.requests.size() - 1; request_id >= 0; --request_id) {
        RequestState& rs = state.requests[request_id];
        if (rs.loc_node == rs.goal && rs.loc_p == 1.0f) {
          // state.requests.erase(state.requests.begin() + request_id);
          for (int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
            if (isRobotExactlyAt(state.robots[robot_id], rs.loc_node)) {
              state.requests.erase(state.requests.begin() + request_id);
              break;
            }
          }
        }
      }
    } else {
      state.requests.erase(std::remove_if(state.requests.begin(), state.requests.end(), requestComplete),
                           state.requests.end());
    }

  }

  float MotionModel::getHumanSpeed() const {
    return human_speed_;
  }

  float MotionModel::getHumanSpeedInElevator() const {
    return elevator_human_speed_;
  }

  float MotionModel::getRobotSpeed() const {
    return robot_speed_;
  }

  float MotionModel::getRobotSpeedInElevator() const {
    return elevator_robot_speed_;
  }


} /* bwi_guidance */
