#ifndef UTEXAS_GUIDANCE_RESTRICTED_MODEL_H
#define UTEXAS_GUIDANCE_RESTRICTED_MODEL_H

#include <utexas_guidance/mdp/extended_structures.h>
#include <utexas_guidance/mdp/guidance_model.h>

namespace utexas_guidance {

  const static int MAX_ASSIGNED_ROBOTS_NOLIMIT = -1;
  const static int MAX_ASSIGNED_ROBOTS_SAME_AS_REQUESTS = -2;

  class RestrictedModel : public GuidanceModel {

    public:

#define PARAMS(_) \
      _(bool,h1_autoselect_robot_for_destinations,h1_autoselect_robot_for_destinations,false) \
      _(int,h2_max_assigned_robots,h2_max_assigned_robots,MAX_ASSIGNED_ROBOTS_SAME_AS_REQUESTS) \
      _(bool,h3_restrict_ordering,h3_restrict_ordering,true) \
      _(bool,h4_disallow_multiple_assignments,h4_disallow_multiple_assignments,true) \
      _(bool,h6_force_assistance,h6_force_assistance,true) \
      _(bool,h7_prevent_assignment_to_release_locs,h7_prevent_assignment_to_release_locs,true) \

      Params_STRUCT(PARAMS)
#undef PARAMS

      typedef boost::shared_ptr<RestrictedModel> Ptr;
      typedef boost::shared_ptr<const RestrictedModel> ConstPtr;

      virtual ~RestrictedModel();

      virtual void init(const YAML::Node& params,
                        const std::string& output_directory,
                        const boost::shared_ptr<RNG>& rng = boost::shared_ptr<RNG>());

      virtual std::string getName() const;

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

    private:

      /* int selectBestRobotForTask(const ExtendedState& state, int destination) const; */

      Params restricted_model_params_;
  };

} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_RESTRICTED_MODEL_H */
