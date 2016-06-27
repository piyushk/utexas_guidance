#ifndef UTEXAS_GUIDANCE_RESTRICTED_MODEL
#define UTEXAS_GUIDANCE_RESTRICTED_MODEL

namespace utexas_guidance {

  const static int MAX_ASSIGNED_ROBOTS_NOLIMIT = -1;
  const static int MAX_ASSIGNED_ROBOTS_SAME_AS_REQUESTS = -2;

  class RestrictedModel : public RestrictedModel {

    public:

#define PARAMS(_) \
      _(int,max_assigned_robots,max_assigned_robots,MAX_ASSIGNED_ROBOTS_SAME_AS_REQUESTS) \
      _(int,assign_n_adjacent,assign_n_adjacent,3) \

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

      Params restricted_model_params_;
  };

} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_RESTRICTED_MODEL */
