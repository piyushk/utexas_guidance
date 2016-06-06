#ifndef UTEXAS_GUIDANCE_STRUCTURES_H
#define UTEXAS_GUIDANCE_STRUCTURES_H

#include <ostream>
#include <boost/serialization/vector.hpp>

#include <utexas_guidance/graph/graph.h>
#include <utexas_planning/core/action.h>
#include <utexas_planning/core/state.h>

namespace utexas_guidance {

  /* Actions - need to be named differently from previous ActionTypes*/
  enum ActionType {
    WAIT = 0,
    ASSIGN_ROBOT = 1,
    DIRECT_PERSON = 2,
    RELEASE_ROBOT = 3,
    LEAD_PERSON = 4
  };

  class Action {

    public:

      typedef boost::shared_ptr<Action> Ptr;
      typedef boost::shared_ptr<const Action> ConstPtr;

      Action();
      Action(ActionType a, int robot_id = 0, int node = 0);
      virtual ~Action();

      virtual bool operator<(const utexas_planning::Action& other) const;
      virtual bool operator==(const utexas_planning::Action& other) const;
      virtual bool operator>(const utexas_planning::Action& other) const;
      virtual std::size_t hash() const;

      virtual void serialize(std::ostream& stream) const;
      virtual std::string getName() const;

      ActionType type;
      int robot_id; // with ASSIGN_ROBOT, identifies the robot
      int node; // with DIRECT_PERSON or LEAD_PERSON, identifies the direction the robot should guide/lead to.

    private:

      friend class boost::serialization::access;
      template<class Archive> void serialize(Archive& ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(type);
        ar & BOOST_SERIALIZATION_NVP(robot_id);
        ar & BOOST_SERIALIZATION_NVP(node);
      }

  };

  bool operator==(const Action& l, const Action& r);
  bool operator<(const Action& l, const Action& r);
  bool operator>(const Action& l, const Action& r);
  std::ostream& operator<<(std::ostream& stream, const Action& a);

  /* States */
  struct RobotState {

    int loc_u;
    int loc_v;
    float loc_p;

    int tau_d;
    float tau_t;
    float tau_total_task_time;
    int tau_u;

    int help_destination;

  };

  bool operator<(const RobotState& l, const RobotState& r);
  bool operator==(const RobotState& l, const RobotState& r);
  bool operator>(const RobotState& l, const RobotState& r);
  std::ostream& operator<<(std::ostream& stream, const RobotState& rs);
  std::size_t hash_value(const RobotState &rs);

  struct RequestState {

    int loc_node;
    int loc_prev;

    int assist_type;
    int assist_loc;

  };

  bool operator==(const RequestState& l, const RequestState& r);
  bool operator<(const RequestState& l, const RequestState& r);
  bool operator>(const RequestState& l, const RequestState& r);
  std::ostream& operator<<(std::ostream& stream, const RequestState& rq);
  std::size_t hash_value(const RequestState &rq);

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

  private:

    virtual utexas_planning::State::Ptr cloneImpl() const;

    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive& ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(State);
      ar & BOOST_SERIALIZATION_NVP(requests);
      ar & BOOST_SERIALIZATION_NVP(robots);
    }

  };

} /* utexas_guidance*/

#endif /* end of include guard: UTEXAS_GUIDANCE_STRUCTURES_H */
