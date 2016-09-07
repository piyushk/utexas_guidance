#ifndef UTEXAS_GUIDANCE_MDP_SINGLE_ROBOT_SOLVER
#define UTEXAS_GUIDANCE_MDP_SINGLE_ROBOT_SOLVER

#include <utexas_guidance/mdp/structures.h>
#include <utexas_guidance/graph/graph.h>

#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/core/abstract_planner.h>

namespace utexas_guidance {

  class SingleRobotSolver : public utexas_planning::AbstractPlanner {

    public:

#define PARAMS(_) \
      _(bool,h0_wait_for_new_request,h0_wait_for_new_request,false) \
      _(bool,sleep_for_timeout,sleep_for_timeout,false) \

      Params_STRUCT(PARAMS)
#undef PARAMS

      typedef boost::shared_ptr<SingleRobotSolver> Ptr;
      typedef boost::shared_ptr<const SingleRobotSolver> ConstPtr;

      virtual ~SingleRobotSolver();

      virtual void init(const utexas_planning::GenerativeModel::ConstPtr& model,
                        const YAML::Node& params,
                        const std::string& output_directory,
                        const boost::shared_ptr<RNG>& rng = boost::shared_ptr<RNG>(),
                        bool verbose = false);

      virtual void performEpisodeStartProcessing(const utexas_planning::State::ConstPtr& start_state,
                                                 float timeout = utexas_planning::NO_TIMEOUT);

      virtual utexas_planning::Action::ConstPtr getBestAction(const utexas_planning::State::ConstPtr& state) const;

      virtual void performPreActionProcessing(const utexas_planning::State::ConstPtr& state,
                                              const utexas_planning::Action::ConstPtr& prev_action,
                                              float timeout = utexas_planning::NO_TIMEOUT);

      virtual void performPostActionProcessing(const utexas_planning::State::ConstPtr& state,
                                               const utexas_planning::Action::ConstPtr& action,
                                               float timeout = utexas_planning::NO_TIMEOUT);

      virtual std::string getName() const;

      Params params_;

    private:

      boost::shared_ptr<RNG> rng_;

      utexas_guidance::GuidanceModel::ConstPtr model_;
      Graph graph_;
      std::vector<std::vector<std::vector<int> > > shortest_paths_;
      std::vector<std::vector<float> > shortest_distances_;
      std::vector<std::vector<int> > adjacent_vertices_map_;

  };

} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_MDP_SINGLE_ROBOT_SOLVER */
