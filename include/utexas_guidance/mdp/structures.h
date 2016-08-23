#ifndef UTEXAS_GUIDANCE_STRUCTURES_H
#define UTEXAS_GUIDANCE_STRUCTURES_H

#include <ostream>
#include <boost/serialization/vector.hpp>

#include <utexas_guidance/graph/graph.h>
#include <utexas_planning/core/action.h>
#include <utexas_planning/core/state.h>

namespace utexas_guidance {

  const static int NONE = -1;

  /* Actions - need to be named differently from previous ActionTypes*/
  enum ActionType {
    WAIT = 0,
    ASSIGN_ROBOT = 1,
    DIRECT_PERSON = 2,
    RELEASE_ROBOT = 3,
    LEAD_PERSON = 4
  };

  class Action : public utexas_planning::Action {

    public:

      typedef boost::shared_ptr<Action> Ptr;
      typedef boost::shared_ptr<const Action> ConstPtr;

      Action(ActionType a = WAIT, 
             int robot_id = NONE, 
             int node = NONE, 
             int request_id = NONE, 
             int old_help_destination = NONE);
      virtual ~Action();

      virtual bool operator<(const utexas_planning::Action& other) const;
      virtual bool operator==(const utexas_planning::Action& other) const;
      virtual bool operator>(const utexas_planning::Action& other) const;
      virtual std::size_t hash() const;

      virtual void serialize(std::ostream& stream) const;
      virtual std::string getName() const;

      ActionType type;
      int robot_id; // with ASSIGN_ROBOT, identifies the robot
      int node; // with ASSIGN, identifies assigned location.
                // with DIRECT_PERSON or LEAD_PERSON, identifies the direction the robot should guide/lead to.
      int request_id; // with DIRECT_PERSON or LEAD_PERSON, identifies the request id being served.
      int old_help_destination; // with release, assign, or lead, indicates old help destination. This allows
                                // non wait actions to be invertible.

    private:

      friend class boost::serialization::access;
      template<class Archive> void serialize(Archive& ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(type);
        ar & BOOST_SERIALIZATION_NVP(robot_id);
        ar & BOOST_SERIALIZATION_NVP(node);
        ar & BOOST_SERIALIZATION_NVP(request_id);
        ar & BOOST_SERIALIZATION_NVP(old_help_destination);
      }

  };

  /* States */
  struct RobotState {

    int loc_u;
    int loc_v;
    float loc_p;

    int tau_d;
    float tau_t;
    float tau_total_task_time;
    float tau_u;

    bool is_leading_person;
    int help_destination;

  };

  bool operator<(const RobotState& l, const RobotState& r);
  bool operator==(const RobotState& l, const RobotState& r);
  bool operator>(const RobotState& l, const RobotState& r);
  std::ostream& operator<<(std::ostream& stream, const RobotState& rs);
  std::size_t hash_value(const RobotState &rs);

  struct RequestState {

    int request_id;

    int loc_prev;
    int loc_node;
    float loc_p; // Identifies distance traveled by person from loc_prev to loc_node. If 1.0f, then exactly at loc_node.

    int assist_type;
    int assist_loc;

    int goal;

  };

  bool operator==(const RequestState& l, const RequestState& r);
  bool operator<(const RequestState& l, const RequestState& r);
  bool operator>(const RequestState& l, const RequestState& r);
  std::ostream& operator<<(std::ostream& stream, const RequestState& rq);
  std::size_t hash_value(const RequestState &rq);
  int generateNewRequestId();

  class State : public utexas_planning::State {

  public:

    typedef boost::shared_ptr<State> Ptr;
    typedef boost::shared_ptr<const State> ConstPtr;

    virtual ~State();

    virtual bool operator<(const utexas_planning::State& other) const;
    virtual bool operator==(const utexas_planning::State& other) const;
    virtual bool operator>(const utexas_planning::State& other) const;
    virtual std::size_t hash() const;

    virtual void serialize(std::ostream& stream) const;
    virtual std::string getName() const;

    std::vector<RequestState> requests;
    std::vector<RobotState> robots;

    std::vector<Action> actions_since_wait;

  private:

    virtual utexas_planning::State::Ptr cloneImpl() const;

    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive& ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(State);
      ar & BOOST_SERIALIZATION_NVP(requests);
      ar & BOOST_SERIALIZATION_NVP(robots);
      ar & BOOST_SERIALIZATION_NVP(actions_since_wait);
    }

  };

  /* Reward Metrics - Things apart from reward that we care about. */
  class RewardMetrics : public utexas_planning::RewardMetrics {
    public:
      typedef boost::shared_ptr<RewardMetrics> Ptr;
      typedef boost::shared_ptr<const RewardMetrics> ConstPtr;

      float utility_loss;
      float time_loss;

      virtual std::map<std::string, std::string> asMap() const;
  };

} /* utexas_guidance*/

#endif /* end of include guard: UTEXAS_GUIDANCE_STRUCTURES_H */
