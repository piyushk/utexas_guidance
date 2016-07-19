#ifndef UTEXAS_GUIDANCE_GUIDANCE_MODEL_H
#define UTEXAS_GUIDANCE_GUIDANCE_MODEL_H

#include <utexas_guidance/mdp/structures.h>
#include <utexas_guidance/mdp/transition_model.h>
#include <utexas_guidance/mdp/visualizer.h>
#include <utexas_guidance/graph/graph.h>

#include <utexas_planning/common/params.h>
#include <utexas_planning/core/generative_model.h>

namespace utexas_guidance {

  const std::string FIXED_TASK_MODEL = "fixed";
  const std::string RANDOM_TASK_MODEL = "random";

  class GuidanceModel : public utexas_planning::GenerativeModel {

    public:

#define PARAMS(_) \
      _(std::string,graph_file,graph_file,"") \
      _(float,human_variance_multiplier,human_variance_multiplier,1.0f) \
      _(std::string,task_model,task_model,FIXED_TASK_MODEL) \
      _(std::string,task_model_file,task_model_file,"") \
      _(float,task_time,task_time,10.0f) \
      _(float,task_utility,task_utility,1.0f) \
      _(bool,task_at_home_base_only,task_at_home_base_only,false) /* For RandomTaskGenerationModel only. */ \
      _(int,initial_num_requests,initial_num_requests,1) /* Ignored if getStartState is not used. */ \
      _(std::string,start_idxs,start_idxs,"") /* Debugging only. */ \
      _(std::string,goal_idxs,goal_idxs,"") /* Debugging only. */ \
      _(bool,terminate_at_zero_requests,terminate_at_zero_requests,true) \
      _(float,human_speed,human_speed,1.0f) \
      _(float,robot_speed,robot_speed,0.5f) \
      _(float,elevator_human_speed,elevator_human_speed,0.3f) \
      _(float,elevator_robot_speed,elevator_robot_speed,0.2f) \
      _(float,initial_planning_time,initial_planning_time,10.0f) \

      Params_STRUCT(PARAMS)
#undef PARAMS

      typedef boost::shared_ptr<GuidanceModel> Ptr;
      typedef boost::shared_ptr<const GuidanceModel> ConstPtr;

      virtual ~GuidanceModel();

      virtual void init(const YAML::Node& params,
                        const std::string& output_directory,
                        const boost::shared_ptr<RNG>& rng = boost::shared_ptr<RNG>());

      virtual std::string getName() const;

      virtual bool isTerminalState(const utexas_planning::State::ConstPtr& state) const;

      virtual void takeAction(const utexas_planning::State::ConstPtr& state,
                              const utexas_planning::Action::ConstPtr& action,
                              float& reward,
                              const utexas_planning::RewardMetrics::Ptr& reward_metrics,
                              utexas_planning::State::ConstPtr& next_state,
                              int& depth_count,
                              float& post_action_timeout,
                              boost::shared_ptr<RNG> rng) const;

      virtual void getActionsAtState(const utexas_planning::State::ConstPtr& state,
                                     std::vector<utexas_planning::Action::ConstPtr>& actions) const;

      virtual utexas_planning::State::ConstPtr getStartState(long seed) const;

      virtual float getInitialTimeout() const;

      virtual void initializeVisualizer(const utexas_planning::Visualizer::Ptr& visualizer) const;
      virtual void getUnderlyingGraph(Graph& graph) const;

      // virtual std::map<std::string, std::string> getParamsAsMap() const;
      // virtual utexas_planning::RewardMetrics::Ptr getRewardMetricsAtEpisodeStart() const;

    protected:

      /* Some cached data. */
      std::vector<std::vector<std::vector<int> > > shortest_paths_;
      std::vector<std::vector<float> > shortest_distances_;
      std::vector<std::vector<int> > adjacent_vertices_map_;

      /* The three subcomponents of the transition model. */
      TaskGenerationModel::ConstPtr task_generation_model_;
      MotionModel::ConstPtr motion_model_;
      HumanDecisionModel::ConstPtr human_decision_model_;

      Graph graph_;

      Params params_;
      int num_vertices_;
  };

} /* bwi_guidance_solver */

#endif /* end of include guard: UTEXAS_GUIDANCE_GUIDANCE_MODEL_H */
