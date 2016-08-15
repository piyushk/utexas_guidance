#include <class_loader/class_loader.h>

#include <utexas_guidance/mdp/guidance_model.h>
#include <utexas_guidance/mdp/pdpt_solver.h>
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

  struct RobotFutureLocation {
    int loc;
    float time_arrive;
    float time_leave;
    int tau_d;
    float tau_u;
  };

  PDPTSolver::~PDPTSolver() {}

  void PDPTSolver::init(const utexas_planning::GenerativeModel::ConstPtr& model_base,
                               const YAML::Node& params,
                               const std::string& output_directory,
                               const boost::shared_ptr<RNG>& rng,
                               bool verbose) {

    model_ = boost::dynamic_pointer_cast<const GuidanceModel>(model_base);
    if (!model_) {
      throw utexas_planning::DowncastException("utexas_planning::GenerativeModel", 
                                               "utexas_guidance::GuidanceModel");
    }
    model_->getUnderlyingGraph(graph_);
    motion_model_ = model_->getUnderlyingMotionModel();
    task_model_ = model_->getUnderlyingTaskModel();
    rng_ = rng;
    getAllAdjacentVertices(adjacent_vertices_map_, graph_);
    getAllShortestPaths(shortest_distances_, shortest_paths_, graph_);
  }

  void PDPTSolver::performEpisodeStartProcessing(const utexas_planning::State::ConstPtr& start_state, 
                                                 float timeout) {}

  utexas_planning::Action::ConstPtr PDPTSolver::getBestAction(const utexas_planning::State::ConstPtr& state_base) const {
  
    State::ConstPtr current_state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!current_state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }

    State state;
    model_->unrollState(*current_state, state);

    float human_speed = motion_model_->getHumanSpeed();
    float robot_speed = motion_model_->getRobotSpeed();

    std::vector<Action::ConstPtr> actions;

    float max_lookahead_time = 150.0f;

    std::vector<std::vector<RobotFutureLocation> > robot_future_locations(state.robots.size());
    for (unsigned int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
      const RobotState& robot = state.robots[robot_id];
      std::vector<RobotFutureLocation>& robot_future_location = robot_future_locations[robot_id];
      if (!robot.is_leading_person) {

        // TODO: Make sure that this is not the only other robot colocated with a human.
        float total_robot_time = 0.0f;
        int current_loc = robot.loc_u;
        float current_destination = (robot.help_destination != NONE) ? robot.help_destination : robot.tau_d;

        RobotState current_task(robot);
        current_task.tau_total_task_time -= current_task.tau_t;

        if (!isRobotExactlyAt(robot, current_destination)) {
          float current_edge_distance = shortest_distances_[robot.loc_u][robot.loc_v];
          float distance_to_u = robot.loc_p * current_edge_distance;
          float distance_to_v = current_edge_distance - distance_to_u;
          float distance_from_u = shortest_distances_[robot.loc_u][current_destination] + distance_to_u;
          float distance_from_v = shortest_distances_[robot.loc_v][current_destination] + distance_to_v;

          if (distance_from_u < distance_from_v) {
            total_robot_time += distance_to_u / robot_speed;
          } else {
            current_loc = robot.loc_v;
            total_robot_time += distance_to_v / robot_speed;
          }
        } else {
          current_loc = current_destination;
        }

        while (total_robot_time <= max_lookahead_time) {
          RobotFutureLocation future_loc;
          future_loc.loc = current_loc;
          future_loc.time_arrive = total_robot_time;
          future_loc.time_leave = total_robot_time;
          future_loc.tau_d = current_task.tau_d;
          future_loc.tau_u = current_task.tau_u;
          if (current_loc == current_destination) {
            if (robot.help_destination != NONE) {
              total_robot_time += current_task.tau_total_task_time;
              future_loc.time_leave = total_robot_time;
              RNG unused_rng(0);
              task_model_->generateNewTaskForRobot(robot_id, current_task, unused_rng); // Fixed task only.
              current_destination = current_task.tau_d;
            } else {
              total_robot_time = max_lookahead_time;
              future_loc.time_leave = total_robot_time;
              robot_future_location.push_back(future_loc);
              break;
            }
          }
          robot_future_location.push_back(future_loc);
          int next_loc = shortest_paths_[current_loc][current_destination][0];
          total_robot_time += shortest_distances_[current_loc][next_loc] / robot_speed;
          current_loc = next_loc;
        }

        // std::cout << "For robot id " << robot_id << ":-" << std::endl;
        // for (int future_location_id = 0; future_location_id < robot_future_location.size(); ++future_location_id) {
        //   std::cout << "  " << robot_future_location[future_location_id].loc << "@" << 
        //     robot_future_location[future_location_id].time_arrive << "-" <<
        //     robot_future_location[future_location_id].time_leave << std::endl;
        // }
      }
    }

    /* Step 2 - For every request, calculate the intersection between the current request path, and each robot's path.
     * Make a special consideration for elevators. If there is an intersection, calculate how well would it do with a 
     * transfer. If transfer is better, go for it, otherwise continue leading the robot yourself. */

    // Lead via shortest path.
    // TODO: Lead is necessary, but we won't go into this loop if lead person has been called, and we might still need
    // to assign a robot. We'll have to write some code to detect states post lead action as well, alternatively this 
    // solver may not solve the restricted version of the MDP, since that will lead to many problems. so maybe assign a
    // robot first? Allow robots that have been assigned, but are not colocated or are currently leading person to
    // recreate action selection. First assign, then lead. 
    // a default policy. We could also potentially unroll the most likely trajectory of a human that's not being helped
    // as well, and there's no point going in if max assigned robots is being met as well

    for (int request_id = 0; request_id < state.requests.size(); ++request_id) {
      const RequestState& request = state.requests[request_id];

      // Can't do anything for this request.
      if (request.loc_p != 1.0f) {
        continue;
      }

      // Find colocated robot.
      int lead_robot_id = NONE;
      for (unsigned int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
        const RobotState& robot = state.robots[robot_id];
        // TODO: It may be necessary for the lead robot to be assigned to the current location. Test it out.
        if (isRobotExactlyAt(robot, request.loc_node)) {
          // && robot.help_destination == request.loc_node) 
          lead_robot_id = robot_id;
          break;
        }
      }

      if (lead_robot_id == NONE) {
        // There's nothing we can do for this human, as not robots are present. 
        // TODO: Maybe use an arbitrary model to predict the future path?
        continue;
      }

      // Figure out if you need to:
      //  - lead person to the goal.
      //  OR
      //  - assign robot at future position.
      //  - and/or wait at current position,

      // First calculate intersection point. If intersection is past one step of the request, or it does not exist, LEAD PERSON.
      // Once intersection point has been calculated - call assign robot if robot will get there first/leave it prior to
      // next call.

      // Calculate time to which we're interested, which is the time until the next lead action will be executed, and
      // see if some assignments need to take place in that time.
      float max_relative_reward = 0.0f;
      int exchange_robot_id = NONE;
      RobotFutureLocation best_intersection;
      float best_lead_time_to_intersection;
      bool best_use_elevator_hack;

      for (unsigned int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {

        if (robot_id == lead_robot_id) {
          continue;
        }

        // See if the current path of this robot is going to intersect with the request we're interested in.
        std::vector<RobotFutureLocation>& robot_future_location = robot_future_locations[robot_id];

        if (robot_future_location.size() == 0) {
          continue;
        } 

        // See if there is an intersection of the current request with this robot, and if there is, also record the
        // location prior to the intersection. If the location prior to the intersection is not on the same floor as the
        // intersection location, then final action will be direct. 
        // Also this feature should be exposed via a parameter.
        RobotFutureLocation intersection;
        intersection.loc = -1;
        int location_prior_to_intersection = -1;
        for (int future_location_id = 0; future_location_id < robot_future_location.size(); ++future_location_id) {

          // Figure out if there is a direct intersection.
          if (request.loc_node == robot_future_location[future_location_id].loc) {
            intersection = robot_future_location[future_location_id];
            break;
          }

          location_prior_to_intersection = request.loc_node;
          for (int path_counter_id = 0; 
               path_counter_id < shortest_paths_[request.loc_node][request.goal].size(); 
               ++path_counter_id) {
            if (shortest_paths_[request.loc_node][request.goal][path_counter_id] == 
                robot_future_location[future_location_id].loc) {
              intersection = robot_future_location[future_location_id];
              break;
            }
            location_prior_to_intersection = shortest_paths_[request.loc_node][request.goal][path_counter_id];
          }
        }

        bool use_elevator_hack = true;
        use_elevator_hack = use_elevator_hack && 
          location_prior_to_intersection != NONE &&
          !onSameFloor(location_prior_to_intersection,
                       intersection.loc,
                       graph_);

        float lead_time_to_previous_intersection = 0.0f;
        if (use_elevator_hack) {
          lead_time_to_previous_intersection = 
            robot_speed * shortest_distances_[request.loc_node][location_prior_to_intersection];
        }

        float direct_elev_lead_time_to_intersection = lead_time_to_previous_intersection + 
          human_speed * shortest_distances_[location_prior_to_intersection][intersection.loc];
        use_elevator_hack = use_elevator_hack &&
          direct_elev_lead_time_to_intersection > intersection.time_arrive;

        float reward = 0.0f;
        if (intersection.loc != -1) {
          std::cout << "Request " << request_id << ": have intersecting paths with robot " << robot_id << std::endl;
          // First calculate the reward loss due to wait at the intersection point.

          float lead_time_to_intersection = 0.0f;
          if (use_elevator_hack) {
            lead_time_to_intersection = direct_elev_lead_time_to_intersection;
            reward += shortest_distances_[location_prior_to_intersection][intersection.loc] / robot_speed -
              shortest_distances_[location_prior_to_intersection][intersection.loc] / human_speed;
            std::cout << "time gained by directing at the elevator. " << reward << std::endl;
          } else {
            lead_time_to_intersection = robot_speed * shortest_distances_[request.loc_node][intersection.loc];
          }

          // First compute reward due to waiting.
          if (lead_time_to_intersection > intersection.time_leave) {
            // We'll have to reserve the robot as soon as it arrives at the location.
            reward -= intersection.tau_u * (lead_time_to_intersection - intersection.time_arrive);
            std::cout << "  reward due to waiting for lead robot." << reward << std::endl;
          } else if (lead_time_to_intersection < intersection.time_arrive) {
            reward -= (1.0f + state.robots[lead_robot_id].tau_u) * (intersection.time_arrive - lead_time_to_intersection);
            std::cout << "  reward due to waiting for exchange robot." << reward << std::endl;
          } else {
            // The robot's doing work at the intersection location when the person arrives. There's no penalty.
            reward -= 0.0f;
          }

          // Next, calculate the reward loss by assigning the new robot. You need the destination of the robot at the
          // time of intersection. 
          float time_to_service_destination_before_action =
            robot_speed * shortest_distances_[intersection.loc][intersection.tau_d];
          float time_to_service_destination_after_action =
            robot_speed * shortest_distances_[request.goal][intersection.tau_d];
          float leading_time = 
            robot_speed * shortest_distances_[intersection.loc][request.goal];
          float extra_time_to_service_destination = 
            leading_time + time_to_service_destination_after_action - time_to_service_destination_before_action;
          reward -= intersection.tau_u * extra_time_to_service_destination;
          std::cout << "  extra time to service destination: " << extra_time_to_service_destination << std::endl;

          // Next calculate the reward gain by early relief of the original leading robot.
          time_to_service_destination_before_action =
            robot_speed * shortest_distances_[request.goal][state.robots[lead_robot_id].tau_d];
          time_to_service_destination_after_action =
            robot_speed * shortest_distances_[intersection.loc][state.robots[lead_robot_id].tau_d];
          float time_not_spent_leading = 
            robot_speed * shortest_distances_[intersection.loc][request.goal];
          float time_saved = 
            time_not_spent_leading + time_to_service_destination_before_action - time_to_service_destination_before_action;
          reward += state.robots[lead_robot_id].tau_u * time_saved;
          std::cout << "  time saved: " << time_saved << std::endl;

          if (reward > max_relative_reward) {
            std::cout << "exchanging robots. " << std::endl;
            max_relative_reward = reward;
            exchange_robot_id = robot_id;
            best_intersection = intersection;
            best_lead_time_to_intersection = lead_time_to_intersection;
            best_use_elevator_hack = use_elevator_hack;
          }

        }
      }

      if (exchange_robot_id != NONE) {
        std::cout << "Request " << request_id << ": replacing robot " << lead_robot_id << " with " << exchange_robot_id << std::endl;
        // Yay. It looks like we're going to exchange the robot. 
        // First deal with assigning the exchange robot if we're going to be too late getting there.
        // TODO: make the exchange_robot_id unavailable.
        if (state.robots[exchange_robot_id].tau_d != best_intersection.tau_d) {
          // Don't assign this robot yet.
          // TODO: This might be a mistake.
        } else {
          if (best_intersection.time_leave < best_lead_time_to_intersection) {
            actions.push_back(Action::ConstPtr(new Action(ASSIGN_ROBOT, 
                                                          exchange_robot_id, 
                                                          best_intersection.loc,
                                                          NONE,
                                                          state.robots[exchange_robot_id].help_destination)));
          }
        }

        // Now, if the person is not at the intersection loc, then get them there.
        if (request.loc_node != best_intersection.loc) {
          int next_node = 
            shortest_paths_[state.requests[request_id].loc_node][state.requests[request_id].goal][0];
          if (next_node == best_intersection.loc && best_use_elevator_hack) {
            actions.push_back(Action::ConstPtr(new Action(RELEASE_ROBOT, 
                                                          lead_robot_id, 
                                                          NONE, 
                                                          NONE, 
                                                          state.robots[lead_robot_id].help_destination))); 
            actions.push_back(Action::ConstPtr(new Action(DIRECT_PERSON, 
                                                          lead_robot_id, 
                                                          next_node, 
                                                          request_id, 
                                                          state.robots[lead_robot_id].help_destination))); 
          } else {
            actions.push_back(Action::ConstPtr(new Action(LEAD_PERSON, 
                                                          lead_robot_id, 
                                                          next_node, 
                                                          request_id, 
                                                          state.robots[lead_robot_id].help_destination))); 
          }
        } else {
          // See if you need to wait here for the exchange robot.
          if (!isRobotExactlyAt(state.robots[exchange_robot_id], request.loc_node)) {
            actions.push_back(Action::ConstPtr(new Action(LEAD_PERSON,
                                                          lead_robot_id,
                                                          request.loc_node,
                                                          request_id,
                                                          state.robots[lead_robot_id].help_destination)));
          } else {
            actions.push_back(Action::ConstPtr(new Action(RELEASE_ROBOT, 
                                                          lead_robot_id, 
                                                          NONE, 
                                                          NONE, 
                                                          state.robots[lead_robot_id].help_destination))); 
            // Continue leading the robot.
            int next_node = 
              shortest_paths_[state.requests[request_id].loc_node][state.requests[request_id].goal][0];
            actions.push_back(Action::ConstPtr(new Action(LEAD_PERSON, 
                                                          exchange_robot_id, 
                                                          next_node, 
                                                          request_id, 
                                                          state.robots[exchange_robot_id].help_destination))); 
          }
        }
      } else {
        // Continue leading the robot.
        int next_node = 
          shortest_paths_[state.requests[request_id].loc_node][state.requests[request_id].goal][0];
        actions.push_back(Action::ConstPtr(new Action(LEAD_PERSON, 
                                                      lead_robot_id, 
                                                      next_node, 
                                                      request_id, 
                                                      state.robots[lead_robot_id].help_destination))); 
      }

    }

    throw std::runtime_error("blah!");

    return Action::ConstPtr(new Action(WAIT));
  }

  void PDPTSolver::performPreActionProcessing(const utexas_planning::State::ConstPtr& state,
                                              const utexas_planning::Action::ConstPtr& prev_action,
                                              float timeout) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(timeout * 1000.0f));
  }

  void PDPTSolver::performPostActionProcessing(const utexas_planning::State::ConstPtr& state,
                                               const utexas_planning::Action::ConstPtr& action,
                                               float timeout) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(timeout * 1000.0f));
  }

  std::string PDPTSolver::getName() const {
    return std::string("PDPTSolver");
  }

} /* utexas_guidance */

CLASS_LOADER_REGISTER_CLASS(utexas_guidance::PDPTSolver, utexas_planning::AbstractPlanner)
