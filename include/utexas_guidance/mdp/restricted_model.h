#ifndef UTEXAS_GUIDANCE_RESTRICTED_MODEL
#define UTEXAS_GUIDANCE_RESTRICTED_MODEL

namespace utexas_guidance {

  class RestrictedModel : public RestrictedModel {

    public:

      typedef boost::shared_ptr<RestrictedModel> Ptr;
      typedef boost::shared_ptr<const RestrictedModel> ConstPtr;

      virtual ~RestrictedModel();

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

  };

} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_RESTRICTED_MODEL */
