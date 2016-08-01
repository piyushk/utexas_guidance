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

        // TODO parametrize this time.
        while (total_robot_time < 150.0f) {
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
    std::vector<std::pair<int, int> > robot_request_ids;
    getColocatedRobotRequestIds(*state, robot_request_ids);
    for (unsigned int idx_num = 0; idx_num < robot_request_ids.size(); ++idx_num) {
      int robot_id = robot_request_ids[idx_num].first;
      int request_id = robot_request_ids[idx_num].second;
      const RequestState& request = state->requests[request_id];
      // Get shortest path to goal for this request, and see if swapping will help. 
      Action a(LEAD_PERSON, robot_id, shortest_paths_[request.loc_node][request.goal][0], request_id);
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
