#ifndef UTEXAS_GUIDANCE_GUIDANCE_MODEL_H
#define UTEXAS_GUIDANCE_GUIDANCE_MODEL_H

#include <utexas_guidance/mdp/structures.h>
#include <utexas_guidance/mdp/transition_model.h>
#include <utexas_guidance/graph.h>

#include <utexas_planning/common/params.h>
#include <utexas_planning/core/generative_model.h>

namespace utexas_guidnace {

  const std::string FIXED_TASK_MODEL = "fixed";
  const std::string RANDOM_TASK_MODEL = "random";

  class GuidanceModel : public GenerativeModel {

    public:

#define PARAMS(_) \
      _(std::string,graph_file,graph_file,"") \
      _(float,human_variance_multiplier,human_variance_multiplier,1.0f) \
      _(std::string,task_model,task_model,FIXED_TASK_MODEL) \
      _(std::string,task_model_file,task_model_file"") \
      _(float,task_time,task_time,1.0f) \
      _(float,task_utility,task_utility,1.0f) \
      _(bool,task_at_home_base_only,task_at_home_base_only,false) /* For RandomTaskGenerationModel only. */ \
      _(int,start_idx,start_idx,NONE) \
      _(int,goal_idx,goal_idx,NONE) \
      _(float,human_speed,human_speed,1.0f) \
      _(float,robot_speed,robot_speed,1.0f) \
      _(float,avg_el,task_time,1.0f) \
      _(float,task_time,task_time,1.0f) \

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
      virtual std::map<std::string, std::string> getParamsAsMap() const;
      virtual RewardMetrics::Ptr getRewardMetricsAtEpisodeStart() const;

    private:

      /* Some private helper functions. */
      void getColocatedRobotIds(const State& state, std::vector<int> &robot_ids);
      void getActionsAtState(const State &state, std::vector<Action>& actions);

      /* Some cached data. */
      std::vector<std::vector<std::vector<size_t> > > shortest_paths_;
      std::vector<std::vector<float> > shortest_distances_;
      std::map<int, std::vector<int> > adjacent_vertices_map_;

      /* The three subcomponents of the transition model. */
      const TaskGenerationModel::Ptr task_generation_model_;
      const MotionModel::Ptr motion_model_;
      const HumanDecisionModel::Ptr human_decision_model_;

      bwi_mapper::Graph graph_;

      Params params_;
      int goal_idx_;
      int num_vertices_;
  };

} /* bwi_guidance_solver */

#endif /* end of include guard: UTEXAS_GUIDANCE_GUIDANCE_MODEL_H */
