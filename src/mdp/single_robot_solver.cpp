#include <class_loader/class_loader.h>

#include <utexas_guidance/mdp/guidance_model.h>
#include <utexas_guidance/mdp/single_robot_solver.h>
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

  SingleRobotSolver::~SingleRobotSolver() {}

  void SingleRobotSolver::init(const utexas_planning::GenerativeModel::ConstPtr& model_base,
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
    rng_ = rng;
    getAllAdjacentVertices(adjacent_vertices_map_, graph_);
    getAllShortestPaths(shortest_distances_, shortest_paths_, graph_);
  }

  void SingleRobotSolver::performEpisodeStartProcessing(const utexas_planning::State::ConstPtr& start_state, 
                                                        float timeout) {}

  utexas_planning::Action::ConstPtr SingleRobotSolver::getBestAction(const utexas_planning::State::ConstPtr& state_base) const {
  
    State::ConstPtr state = boost::dynamic_pointer_cast<const State>(state_base);
    if (!state) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }

    std::vector<utexas_planning::Action::ConstPtr> actions;
    model_->getActionsAtState(state, actions);

    // Lead via shortest path.
    std::vector<std::pair<int, int> > robot_request_ids;
    getColocatedRobotRequestIds(*state, robot_request_ids);
    for (unsigned int idx_num = 0; idx_num < robot_request_ids.size(); ++idx_num) {
      int robot_id = robot_request_ids[idx_num].first;
      int request_id = robot_request_ids[idx_num].second;
      const RequestState& request = state->requests[request_id];
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

  void SingleRobotSolver::performPreActionProcessing(const utexas_planning::State::ConstPtr& state,
                                                     const utexas_planning::Action::ConstPtr& prev_action,
                                                     float timeout) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(timeout * 1000.0f));
  }

  void SingleRobotSolver::performPostActionProcessing(const utexas_planning::State::ConstPtr& state,
                                                      const utexas_planning::Action::ConstPtr& action,
                                                      float timeout) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(timeout * 1000.0f));
  }

  std::string SingleRobotSolver::getName() const {
    return std::string("SingleRobot");
  }

} /* utexas_guidance */

CLASS_LOADER_REGISTER_CLASS(utexas_guidance::SingleRobotSolver, utexas_planning::AbstractPlanner)
