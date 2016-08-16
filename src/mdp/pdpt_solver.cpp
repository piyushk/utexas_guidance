#include <boost/foreach.hpp>
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

    std::vector<Action::ConstPtr> all_actions;

    std::vector<bool> robot_available(state.robots.size(), true);
    std::vector<int> lead_robot(state.requests.size(), NONE);

    for (int request_id = 0; request_id < state.requests.size(); ++request_id) {
      const RequestState& request = state.requests[request_id];

      // Can't do anything for this request.
      if (request.loc_p != 1.0f) {
        continue;
      }

      for (unsigned int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
        if (robot_available[robot_id]) {
          const RobotState& robot = state.robots[robot_id];
          // TODO: It may be necessary for the lead robot to be assigned to the current location. Test it out.
          if (isRobotExactlyAt(robot, request.loc_node)) {
            // && robot.help_destination == request.loc_node) 
            lead_robot[request_id] = robot_id;
            robot_available[robot_id] = false;
            break;
          }
        }
      }
    }

    float max_lookahead_time = 150.0f;

    std::vector<std::vector<RobotFutureLocation> > robot_future_locations(state.robots.size());
    for (unsigned int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
      const RobotState& robot = state.robots[robot_id];
      if (robot.is_leading_person) {
        robot_available[robot_id] = false;
      }
      
      if (robot_available[robot_id]) {

        std::vector<RobotFutureLocation>& robot_future_location = robot_future_locations[robot_id];

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
            if (robot.help_destination == NONE) {
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

        std::cout << "For robot id " << robot_id << ", " << robot.help_destination << ":-" << std::endl;
        for (int future_location_id = 0; future_location_id < robot_future_location.size(); ++future_location_id) {
          std::cout << "  " << robot_future_location[future_location_id].loc << "@" << 
            robot_future_location[future_location_id].time_arrive << "-" <<
            robot_future_location[future_location_id].time_leave << std::endl;
        }
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
      if (request.loc_p != 1.0f && lead_robot[request_id] != NONE) {
        continue;
      }

      float max_reward = 0.0f;
      int exchange_robot_id = NONE;
      int lead_robot_id = lead_robot[request_id];

      State start_state(state);
      start_state.requests[0] = start_state.requests[request_id];
      start_state.requests.resize(1);

      std::vector<Action::ConstPtr> actions, actions_till_first_wait;
      /* Calculate the reward accrued by simply leading the robot to the goal. */
      for (int path_idx = 0; path_idx < shortest_paths_[request.loc_node][request.goal].size(); ++path_idx) {
        int next_loc = shortest_paths_[request.loc_node][request.goal][path_idx];
        actions.push_back(Action::ConstPtr(new Action(LEAD_PERSON,
                                                      lead_robot_id,
                                                      next_loc,
                                                      0)));
        // (actions.back())->serialize(std::cout);
        // std::cout << std::endl;
      }
      max_reward = getRewardFromTrajectory(start_state, actions, actions_till_first_wait);

      std::cout << "lead reward for request " << request_id << " is " << max_reward << std::endl;

      for (unsigned int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {

        if (!robot_available[robot_id]) {
          continue;
        }

        /* std::cout << "finding intersection with robot " << robot_id << std::endl; */

        // See if the current path of this robot is going to intersect with the request we're interested in.
        std::vector<RobotFutureLocation>& robot_future_location = robot_future_locations[robot_id];

        // See if there is an intersection of the current request with this robot, and if there is, also record the
        // location prior to the intersection. If the location prior to the intersection is not on the same floor as the
        // intersection location, then final action will be direct. 
        // Also this feature should be exposed via a parameter.
        RobotFutureLocation intersection;
        intersection.loc = -1;
        int location_prior_to_intersection = -1;
        for (int future_location_id = 0; 
             future_location_id < robot_future_location.size() && intersection.loc == -1; 
             ++future_location_id) {

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

        if (intersection.loc != -1) {

           /* Compute some statistics about the intersection. */
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


          /* Compute the actions necessary for this transfer to take place, just for this request. */
          float time = 0.0f;
          int current_loc = request.loc_node;
          bool robot_assigned = false;
          actions.clear();
          for (int path_idx = 0; path_idx < shortest_paths_[request.loc_node][intersection.loc].size(); ++path_idx) {
            int next_loc = shortest_paths_[request.loc_node][intersection.loc][path_idx];
            if (use_elevator_hack && path_idx == shortest_paths_[request.loc_node][intersection.loc].size() - 1) {
              actions.push_back(Action::ConstPtr(new Action(RELEASE_ROBOT,
                                                            lead_robot_id)));
              actions.push_back(Action::ConstPtr(new Action(DIRECT_PERSON,
                                                            lead_robot_id,
                                                            next_loc,
                                                            0)));
              time += human_speed * shortest_distances_[current_loc][next_loc];
            } else {
              actions.push_back(Action::ConstPtr(new Action(LEAD_PERSON,
                                                            lead_robot_id,
                                                            next_loc,
                                                            0)));
              time += robot_speed * shortest_distances_[current_loc][next_loc];
            }
            if (!robot_assigned && (time >= intersection.time_leave)) {
              actions.push_back(Action::ConstPtr(new Action(ASSIGN_ROBOT,
                                                            robot_id,
                                                            next_loc)));
              robot_assigned = true;
            }
            current_loc = shortest_paths_[request.loc_node][intersection.loc][path_idx];
          }

          while (time < intersection.time_arrive) {
            actions.push_back(Action::ConstPtr(new Action(LEAD_PERSON,
                                                          lead_robot_id,
                                                          intersection.loc,
                                                          0)));
            time += 10.0f;
          }

          if (!use_elevator_hack) {
            actions.push_back(Action::ConstPtr(new Action(RELEASE_ROBOT,
                                                          lead_robot_id)));
          }

          for (int path_idx = 0; path_idx < shortest_paths_[intersection.loc][request.goal].size(); ++path_idx) {
            int next_loc = shortest_paths_[intersection.loc][request.goal][path_idx];
            actions.push_back(Action::ConstPtr(new Action(LEAD_PERSON,
                                                          robot_id,
                                                          next_loc,
                                                          0)));
          }

          /* std::cout << "  computed action policy: " << std::endl; */
          
          // BOOST_FOREACH(const Action::ConstPtr& action, actions) {
          //   std::cout << "    ";
          //   action->serialize(std::cout);
          //   std::cout << std::endl; 
          // }
          
          /* std::cout << "  computing reward for robot " << robot_id << std::endl; */

          std::vector<Action::ConstPtr> actions_till_first_wait_temp;
          float reward = getRewardFromTrajectory(start_state, actions, actions_till_first_wait_temp);

          std::cout << "  reward if exchanged with robot " << robot_id << " is " << reward << std::endl;

          if (reward > max_reward) {
            std::cout << "exchanging robots. " << std::endl;
            max_reward = reward;
            exchange_robot_id = robot_id;
            actions_till_first_wait = actions_till_first_wait_temp;
          }

        }
      }

      all_actions.insert(all_actions.end(), actions_till_first_wait.begin(), actions_till_first_wait.end());
      if (exchange_robot_id != NONE) {
        robot_available[exchange_robot_id] = false;
      }

    }

    // std::cout << "Planning on taking actions: " << std::endl;
    // BOOST_FOREACH(const Action::ConstPtr& action, all_actions) {
    //   std::cout << "  ";
    //   action->serialize(std::cout);
    //   std::cout << std::endl;
    // }
    // throw std::runtime_error("blah!");

    std::vector<Action::ConstPtr> actions_at_current_state;
    
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

  float PDPTSolver::getRewardFromTrajectory(const State& state, 
                                            const std::vector<Action::ConstPtr> &actions,
                                            std::vector<Action::ConstPtr> &actions_till_first_wait) const {
    State::ConstPtr state_ptr(new State(state));
    utexas_planning::State::ConstPtr next_state_ptr;

    int action_counter = 0;
    float reward = 0.0f;
    bool first_wait_executed = false, final_wait_executed = false;
    actions_till_first_wait.clear();
    while(action_counter < actions.size() || !final_wait_executed) {
      std::vector<utexas_planning::Action::ConstPtr> actions_at_state;
      model_->getActionsAtState(state_ptr, actions_at_state);
      Action::ConstPtr action(new Action(WAIT));
      if (action_counter < actions.size()) {
        // std::cout << "hunting from action ";
        // actions[action_counter]->serialize(std::cout);
        // std::cout << " in:- " << std::endl; 
        bool action_found = false;
        for (int a = 0; a < actions_at_state.size(); ++a) {
          // std::cout << "  ";
          // actions_at_state[a]->serialize(std::cout);
          // std::cout << std::endl;
          Action::ConstPtr action_d = boost::dynamic_pointer_cast<const Action>(actions_at_state[a]);
          Action::Ptr action_mutable(new Action(*action_d));
          action_mutable->old_help_destination = NONE;
          if (!action_d) {
            throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
          }
          if (*action_mutable == *actions[action_counter]) {
            action = action_d;
            action_found = true;
            break;
          }
        }
        if (action_found) {
          ++action_counter;
        } else {
          first_wait_executed = true;
        }
      } else {
        first_wait_executed = true;
        final_wait_executed = true;
      }

      if (!first_wait_executed) {
        actions_till_first_wait.push_back(action);
      }

      // action->serialize(std::cout);
      // std::cout << std::endl;

      // throw std::runtime_error("blah!");

      int unused_depth_count;
      float step_reward, unused_post_action_timeout;
      boost::shared_ptr<RNG> unused_rng(new RNG(0));
      utexas_planning::RewardMetrics::Ptr unused_reward_metrics;
      model_->takeAction(state_ptr, 
                         action,
                         step_reward, 
                         unused_reward_metrics,
                         next_state_ptr,
                         unused_depth_count,
                         unused_post_action_timeout,
                         unused_rng);
      state_ptr = boost::dynamic_pointer_cast<const State>(next_state_ptr);
      reward += step_reward;
    }

    return reward;
  }

} /* utexas_guidance */

CLASS_LOADER_REGISTER_CLASS(utexas_guidance::PDPTSolver, utexas_planning::AbstractPlanner)
