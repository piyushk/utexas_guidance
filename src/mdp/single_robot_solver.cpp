#include <class_loader/class_loader.h>

#include <utexas_guidance/mdp/guidance_model.h>
#include <utexas_guidance/mdp/single_robot_solver.h>
#include <utexas_planning/common/exceptions.h>

namespace utexas_guidance {

  SingleRobotSolver::~SingleRobotSolver() {}

  void SingleRobotSolver::init(const utexas_planning::GenerativeModel::ConstPtr& model_base,
                               const YAML::Node& params,
                               const std::string& output_directory,
                               const boost::shared_ptr<RNG>& rng,
                               bool verbose) {

    GuidanceModel::ConstPtr model = boost::dynamic_pointer_cast<const GuidanceModel>(model_base);
    if (!model) {
      throw utexas_planning::DowncastException("utexas_planning::GenerativeModel", 
                                               "utexas_guidance::GuidanceModel");
    }
    model->getUnderlyingGraph(graph_);
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

    // Lead via shortest path.
    std::vector<std::pair<int, int> > robot_request_ids;
    getColocatedRobotRequestIds(*state, robot_request_ids);
    for (unsigned int idx_num = 0; idx_num < robot_request_ids.size(); ++idx_num) {
      int robot_id = robot_request_ids[idx_num].first;
      int request_id = robot_request_ids[idx_num].second;
      const RequestState& request = state->requests[request_id];
      return Action::ConstPtr(new Action(LEAD_PERSON, robot_id, shortest_paths_[request.loc_node][request.goal][0], request_id));
    }

    return Action::ConstPtr(new Action(WAIT));
  }

  void SingleRobotSolver::performPreActionProcessing(const utexas_planning::State::ConstPtr& state,
                                                     const utexas_planning::Action::ConstPtr& prev_action,
                                                     float timeout) {}

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
