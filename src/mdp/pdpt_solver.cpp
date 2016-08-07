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
  
    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }
    float robot_speed = motion_model_->getRobotSpeed();

    std::vector<std::pair<int, int> > robot_request_ids;
    getColocatedRobotRequestIds(*state, robot_request_ids);

    std::vector<bool> robot_available(state->robots.size());
    
    std::vector<std::vector<std::pair<int, float> > > future_robot_locations(state->robots.size());
    for (unsigned int robot_idx = 0; robot_idx < state->robots.size(); ++robot_idx) {
      const RobotState& robot = state->robots[robot_idx];
      std::vector<std::pair<int, float> >& future_robot_location = future_robot_locations[robot_idx];
      if (robot.help_destination == NONE) {
        float total_robot_time = 0.0f;
        int current_loc = robot.loc_u;

        RobotState current_task(robot);
        current_task.tau_total_task_time -= current_task.tau_t;

        if (!isRobotExactlyAt(robot, robot.tau_d)) {
          float current_edge_distance = shortest_distances_[robot.loc_u][robot.loc_v];
          float distance_to_u = robot.loc_p * current_edge_distance;
          float distance_to_v = current_edge_distance - distance_to_u;
          float distance_from_u = shortest_distances_[robot.loc_u][robot.tau_d] + distance_to_u;
          float distance_from_v = shortest_distances_[robot.loc_v][robot.tau_d] + distance_to_v;

          if (distance_from_u < distance_from_v) {
            total_robot_time += distance_to_u / robot_speed;
          } else {
            current_loc = robot.loc_v;
            total_robot_time += distance_to_v / robot_speed;
          }
        } else {
          current_loc = robot.tau_d;
        }

        // TODO this time needs to be computed as longest edge distance / robot speed.
        while (total_robot_time <= 150.0f) {
          future_robot_location.push_back(std::make_pair<int, float>(current_loc, total_robot_time));
          if (current_loc == current_task.tau_d) {
            total_robot_time += current_task.tau_total_task_time;
            RNG unused_rng(0);
            task_model_->generateNewTaskForRobot(robot_idx, current_task, unused_rng); // Fixed task only.
          }
          int next_loc = shortest_paths_[current_loc][current_task.tau_d][0];
          total_robot_time += shortest_distances_[current_loc][next_loc] / robot_speed;
          current_loc = next_loc;
        }
        
        // std::cout << "For robot idx " << robot_idx << ":-" << std::endl;
        // for (int future_location_idx = 0; future_location_idx < future_robot_location.size(); ++future_location_idx) {
        //   std::cout << "  " << future_robot_location[future_location_idx].first << "@" << 
        //     future_robot_location[future_location_idx].second << std::endl;
        // }
      }
    }

    /* Step 2 - For every request, calculate the intersection between the current request path, and each robot's path.
     * Make a special consideration for elevators. If there is an intersection, calculate how well would it do with a 
     * transfer. If transfer is better, go for it, otherwise continue leading the robot yourself. */

    std::vector<utexas_planning::Action::ConstPtr> actions;
    model_->getActionsAtState(state, actions);

    // Lead via shortest path.
    // TODO: Lead is necessary, but we won't go into this loop if lead person has been called, and we might still need
    // to assign a robot. We'll have to write some code to detect states post lead action as well, alternatively this 
    // solver may not solve the restricted version of the MDP, since that will lead to many problems. so maybe assign a
    // robot first? Allow robots that have been assigned, but are not colocated or are currently leading person to
    // recreate action selection. First assign, then lead. 
    // a default policy. We could also potentially unroll the most likely trajectory of a human that's not being helped
    // as well, and there's no point going in if max assigned robots is being met as well
    std::vector<std::pair<int, int> > robot_request_ids;
    getColocatedRobotRequestIds(*state, robot_request_ids);
    for (unsigned int idx_num = 0; idx_num < robot_request_ids.size(); ++idx_num) {
      int lead_robot_idx = robot_request_ids[idx_num].first;
      int request_id = robot_request_ids[idx_num].second;
      const RequestState& request = state->requests[request_id];

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
      float max_time = 150.0f;
      std::vector<int> first_intersection(state->robots.size(), -1);
      float max_relative_reward = 0.0f;
      int exchange_idx = -1;
      int max_intersection_idx = -1;
      bool assign_robot_to_intersection = false;
      for (unsigned int robot_idx = 0; robot_idx < state->robots.size(); ++robot_idx) {
        std::vector<std::pair<int, float> >& future_robot_location = future_robot_locations[robot_idx];
        int interesection_idx = -1;
        float robot_time_to_intersection = 0.0f;
        float reward = 0.0f;
        for (int future_location_idx = 0; future_location_idx < future_robot_location.size(); ++future_location_idx) {
          if (future_robot_location[future_location_idx].second > max_time) {
            break;
          }
          if (std::find(shortest_paths_[request.loc_node][request.goal].begin(),
                        shortest_paths_[request.loc_node][request.goal].end(),
                        future_robot_location[future_location_idx].first) != 
              shortest_paths_[request.loc_node][request.goal].end()) {
            intersection_idx = future_robot_location[future_location_idx].first;
            robot_time_to_intersection = future_robot_location[future_location_idx].second;
            break;
          }
        }

        if (intersection_idx != -1) {
          // First calculate the reward loss due to wait at the intersection point.
          float lead_time_to_intersection = robot_speed * shortest_distances_[request_.loc_node][intersection_idx];
          if (lead_time_to_intersection > robot_time_to_intersection) {
            // TODO: This is buggy, since the robot can do work during this time. You need the to know if they're gonna
            // wait at each location they visit as well.
            // TODO: This assumes the task utility is same across all background tasks. You need to store the task
            // utility per task as well. Overall it's starting to look like you need a better data structure than pair.
            reward -= state->robots[robot_idx].tau_u * (lead_time_to_intersection - robot_time_to_intersection);
          } else {
            // TODO: This assumes the task utility is same across all background tasks. You need to store the task
            // utility per task as well. Overall it's starting to look like you need a better data structure than pair.
            reward -= (1.0f + state->robots[lead_robot_idx].tau_u) * (lead_time_to_intersection - robot_time_to_intersection);
          }

          // Next, calculate the reward loss by assigning the new robot. You need the destination of the robot at the
          // time of intersection. 
          // TODO: This is buggy. You don't have that destination yet.
          float time_to_service_destination_before_action =
            robot_speed * shortest_distances_[intersection_idx][destination_at_time_of_intersection];
          float time_to_service_destination_after_action =
            robot_speed * shortest_distances_[request->goal][destination_at_time_of_intersection];
          float leading_time = 
            robot_speed * shortest_distances_[intersection_idx][request->goal];
          float extra_time_to_service_destination = 
            leading_time + time_to_service_destination_after_action - time_to_service_destination_before_action;
          reward -= state->robots[robot_idx].tau_u * extra_time_to_service_destination;

          // Next calculate the reward gain by early relief of the original leading robot.
          float time_to_service_destination_before_action =
            robot_speed * shortest_distances_[request->goal][state->robots[lead_robot_idx].tau_d];
          float time_to_service_destination_after_action =
            robot_speed * shortest_distances_[intersection_idx][state->robots[lead_robot_idx].tau_d];
          float time_not_spent_leading = 
            robot_speed * shortest_distances_[intersection_idx][request->goal];
          float time_saved = 
            time_not_spent_leading + time_to_service_destination_before_action - time_to_service_destination_before_action;
          reward += state->robots[lead_robot_idx].tau_u * extra_time_to_service_destination;

          if (reward > max_relative_reward) {
            max_relative_reward = reward;
            exchange_idx = robot_idx;
            max_intersection_idx = intersection_idx;
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
      Action a(LEAD_PERSON, lead_robot_idx, shortest_paths_[request.loc_node][request.goal][0], request_id);
      std::vector<utexas_planning::Action::ConstPtr>::const_iterator it = 
        std::find_if(actions.begin(), actions.end(), ActionEquals(a));
      if (it != actions.end()) {
        return *it;
      }
    }

    int num_assigned_robots = 0;
    for (unsigned int robot_idx = 0; robot_idx < state->robots.size(); ++robot_idx) {
      if (state->robots[robot_idx].help_destination != NONE) {
        ++num_assigned_robots;
      }
    }

    /* This logic isn't the best as the default policy, but better than nothing to prevent unnecessary robots. */
    if (num_assigned_robots > state->requests.size()) {
      for (unsigned int robot_idx = 0; robot_idx < state->robots.size(); ++robot_idx) {
        if (state->robots[robot_idx].help_destination != NONE &&
            !(state->robots[robot_idx].is_leading_person)) {
          Action a(RELEASE_ROBOT, robot_idx);
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
