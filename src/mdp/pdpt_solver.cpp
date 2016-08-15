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
  }

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

    float robot_speed = motion_model_->getRobotSpeed();

    float max_lookahed_time = 150.0f;

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
          float distance_from_u = shortest_distances_[robot.loc_u][robot.current_destination] + distance_to_u;
          float distance_from_v = shortest_distances_[robot.loc_v][robot.current_destination] + distance_to_v;

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
          if (current_loc == current_destination) {
            if (robot.help_destination != NONE) {
              total_robot_time += current_task.tau_total_task_time;
              future_loc.time_leave = total_robot_time;
              robot_future_location.push_back(future_loc);
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
          int next_loc = shortest_paths_[current_loc][current_destination][0];
          total_robot_time += shortest_distances_[current_loc][next_loc] / robot_speed;
          current_loc = next_loc;
        }
        
        // std::cout << "For robot id " << robot_id << ":-" << std::endl;
        // for (int future_location_id = 0; future_location_id < robot_future_location.size(); ++future_location_id) {
        //   std::cout << "  " << robot_future_location[future_location_id].first << "@" << 
        //     robot_future_location[future_location_id].second << std::endl;
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
        const RobotState& robot = state.robots
        // TODO: It may be necessary for the lead robot to be assigned to the current location. Test it out.
        if (isRobotExactlyAt(robot, request.loc_node)) {
          // && robot.help_destination == request.loc_node) {
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

      // TODO: add some code to handle elevators as well.
      // Calculate time to which we're interested, which is the time until the next lead action will be executed, and
      // see if some assignments need to take place in that time.
      float max_relative_reward = 0.0f;
      int exchange_id = -1;
      int max_intersection_id = -1;
      bool assign_robot_to_intersection = false;

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
               path_counter_id < shortest_paths_[request.loc_node][request.goal]; 
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

        float relative_reward = 0.0f;
        if (intersection.loc != -1) {
          // First calculate the reward loss due to wait at the intersection point.
          float lead_time_to_previous_intersection = 0.0f;
          if (use_elevator_hack &&
              request_.loc_node != location_prior_to_intersection) {
            lead_time_to_previous_intersection = 
              robot_speed * shortest_distances_[request_.loc_node][location_prior_to_intersection];
          }

          float lead_time_to_intersection = 0.0f;
          if (request_.loc_node != intersection.loc) {
            if (use_elevator_hack) {
              lead_time_to_intersection = lead_time_to_previous_intersection + 
                human_speed * shortest_distances_[location_prior_to_intersection][intersection.loc];
            } else {
              lead_time_to_intersection = robot_speed * shortest_distances_[request_.loc_node][intersection.loc];
            }
          }

          // First compute reward due to waiting.
          if (lead_time_to_intersection > intersection.time_leave) {
            // TODO: This assumes the task utility is same across all background tasks. You need to store the task
            // utility per task as well. Overall it's starting to look like you need a better data structure than pair.
            reward -= state.robots[robot_id].tau_u * (lead_time_to_intersection - intersection.time_arrive);
            // This means that we cannot allow the robot to work there at all, as it'll leave as soon as it finishes.
          } else if (lead_time_to_intersection < intersection.time_arrive) {
            // TODO: This assumes the task utility is same across all background tasks. You need to store the task
            // utility per task as well. Overall it's starting to look like you need a better data structure than pair.
            reward -= (1.0f + state.robots[lead_robot_id].tau_u) * (intersection.time_arrive - lead_time_to_intersection);
          } else {
            // The robot's doing work at a location when the person arrives. There's no penalty.
            reward -= 0.0f;
          }

          // Next, calculate the reward loss by assigning the new robot. You need the destination of the robot at the
          // time of intersection. 
          // TODO: This is buggy. You don't have that destination yet.
          float time_to_service_destination_before_action =
            robot_speed * shortest_distances_[intersection.loc][intersection.tau_u];
          float time_to_service_destination_after_action =
            robot_speed * shortest_distances_[request->goal][intersection.tau_u];
          float leading_time = 
            robot_speed * shortest_distances_[intersection.loc][request->goal];
          float extra_time_to_service_destination = 
            leading_time + time_to_service_destination_after_action - time_to_service_destination_before_action;
          reward -= state.robots[robot_id].tau_u * extra_time_to_service_destination;

          // Next calculate the reward gain by early relief of the original leading robot.
          float time_to_service_destination_before_action =
            robot_speed * shortest_distances_[request->goal][state.robots[lead_robot_id].tau_d];
          float time_to_service_destination_after_action =
            robot_speed * shortest_distances_[intersection.loc][state.robots[lead_robot_id].tau_d];
          float time_not_spent_leading = 
            robot_speed * shortest_distances_[intersection.loc][request->goal];
          float time_saved = 
            time_not_spent_leading + time_to_service_destination_before_action - time_to_service_destination_before_action;
          reward += state.robots[lead_robot_id].tau_u * extra_time_to_service_destination;

          if (reward > max_relative_reward) {
            max_relative_reward = reward;
            exchange_id = robot_id;
            max_intersection.loc = intersection.loc;
            // TODO: this is buggy, should use end time at the location, similar to reward.
            if (robot_time_to_intersection < 
          }

        }
      }
        
      // Next, figure out when the exchange will happen, should it happen. If the current location is the exchange
      // location, see if wait (/and assign needs to be called - assign might need to be called to prevent the robot
      // from moving ahead before the wait terminates.
      // If the current location is not the wait location, see if assignment needs to be called to prevent the robot
      // from moving past the intersection point. This might be merged with the last condition.

      // Get shortest path to goal for this request, and see if swapping will help. 
      Action a(LEAD_PERSON, lead_robot_id, shortest_paths_[request.loc_node][request.goal][0], request_id);
      std::vector<utexas_planning::Action::ConstPtr>::const_iterator it = 
        std::find_if(actions.begin(), actions.end(), ActionEquals(a));
      if (it != actions.end()) {
        return *it;
      }
    }

    int num_assigned_robots = 0;
    for (unsigned int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
      if (state.robots[robot_id].help_destination != NONE) {
        ++num_assigned_robots;
      }
    }

    /* This logic isn't the best as the default policy, but better than nothing to prevent unnecessary robots. */
    if (num_assigned_robots > state.requests.size()) {
      for (unsigned int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
        if (state.robots[robot_id].help_destination != NONE &&
            !(state.robots[robot_id].is_leading_person)) {
          Action a(RELEASE_ROBOT, robot_id);
          std::vector<utexas_planning::Action::ConstPtr>::const_iterator it = 
            std::find_if(actions.begin(), actions.end(), ActionEquals(a));
          if (it != actions.end()) {
            return *it;
          }
        }
      }
    }

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
    return std::string("SingleRobot");
  }

} /* utexas_guidance */

CLASS_LOADER_REGISTER_CLASS(utexas_guidance::PDPTSolver, utexas_planning::AbstractPlanner)
