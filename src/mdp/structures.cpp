#include <utexas_guidance/mdp/structures.h>
#include <utexas_planning/common/exceptions.h>

#define COMPARE(x) if((l.x) < (r.x)) return true; \
                   if((l.x) > (r.x)) return false;

#define COMPARE_MEMBER(x) if(x < (other.x)) return true; \
                          if(x > (other.x)) return false;

namespace utexas_guidance {

  /* Action */

  Action::Action() : type(WAIT), robot_id(0), node(0) {}
  Action::Action(ActionType a, int robot_id, int node) : type(a), robot_id(robot_id), node(node) {}
  Action::~Action() {}

  bool Action::operator<(const utexas_planning::Action& other_base) const {
    try {                                                                                                                
      const Action& other = dynamic_cast<const Action&>(other_base);                                             
      return ((type < other.type) ||
              ((type == other.type) && (robot_id < other.robot_id)) ||
              ((type == other.type) && (robot_id == other.robot_id) && (node < other.node)));
    } catch(const std::bad_cast& exp) {                                                                                  
      throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
    }
  }

  bool Action::operator==(const utexas_planning::Action& other_base) const {
    try {                                                                                                                
      const Action& other = dynamic_cast<const Action&>(other_base);                                             
      return ((type == other.type) && (robot_id == other.robot_id) && (node == other.node));
    } catch(const std::bad_cast& exp) {                                                                                  
      throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
    }
  }


  bool Action::operator>(const utexas_planning::Action& other) const {
    return (*this == other || *this < other) ? false : true;
  }

  const std::string ACTION_NAMES[] = {
    "WAIT",
    "ASSIGN_ROBOT",
    "DIRECT_PERSON",
    "RELEASE_ROBOT",
    "LEAD_PERSON"
  };

  std::size_t Action::hash() const {
    size_t seed = 0;
    boost::hash_combine(seed, type);
    boost::hash_combine(seed, robot_id);
    boost::hash_combine(seed, node);
    return seed;
  }

  void Action::serialize(std::ostream& stream) const {
    stream << "[" << ACTION_NAMES[type];
    if (type != WAIT) {
      stream << " " << robot_id;
      if (type != RELEASE_ROBOT) {
      stream << "->" << node;
      }
    }
    stream << "]";
  }

  /* RobotState */

  bool operator<(const RobotState& l, const RobotState& r) {
    COMPARE(loc_u);
    COMPARE(loc_v);
    COMPARE(loc_p);
    COMPARE(tau_d);
    COMPARE(tau_t);
    COMPARE(tau_total_task_time);
    COMPARE(tau_u);
    COMPARE(help_destination);
    return false;
  }

  bool operator==(const RobotState& l, const RobotState& r) {
    return ((l.loc_u == r.loc_u) && 
            (l.loc_v == r.loc_v) &&
            (l.loc_p == r.loc_p) && 
            (l.tau_d == r.tau_d) &&
            (l.tau_t == r.tau_t) &&
            (l.tau_total_task_time == r.tau_total_task_time) &&
            (l.tau_u == r.tau_u) &&
            (l.help_destination == r.help_destination));
  }

  bool operator>(const RobotState& l, const RobotState& r) {
    return (l == r || l < r) ? false : true;
  }

  std::ostream& operator<<(std::ostream& stream, const RobotState& rs) {
    stream << "[(" << rs.loc_u << "," << rs.loc_v << "," << rs.loc_p << "), (" <<
      rs.tau_d << "," <<  rs.tau_t << "," << rs.tau_total_task_time << "," << rs.tau_u << "), " <<
      rs.help_destination << "]";
    return stream;
  }

  size_t hash_value(const RobotState &rs) {
    size_t seed = 0;
    boost::hash_combine(seed, rs.loc_u);
    boost::hash_combine(seed, rs.loc_v);
    boost::hash_combine(seed, rs.loc_p);
    boost::hash_combine(seed, rs.tau_d);
    boost::hash_combine(seed, rs.tau_t);
    boost::hash_combine(seed, rs.tau_total_task_time);
    boost::hash_combine(seed, rs.tau_u);
    boost::hash_combine(seed, rs.help_destination);
    return seed;
  }

  /* RequestState */

  bool operator==(const RequestState& l, const RequestState& r) {
    return ((l.loc_node == r.loc_node) && 
            (l.loc_prev == r.loc_prev) &&
            (l.assist_type == r.assist_type) && 
            (l.assist_loc == r.assist_loc));
  }

  bool operator<(const RequestState& l, const RequestState& r) {
    COMPARE(loc_node);
    COMPARE(loc_prev);
    COMPARE(assist_type);
    COMPARE(assist_loc);
    return false;
  }

  bool operator>(const RequestState& l, const RequestState& r) {
    return (l == r || l < r) ? false : true;
  }

  std::ostream& operator<<(std::ostream& stream, const RequestState& rs) {
    stream << "(<" << rs.loc_node << "," << rs.loc_prev << ">, " <<
      "<"  << rs.assist_type << "," << rs.assist_loc << ">)";
    return stream;
  }

  size_t hash_value(const RequestState &rs) {
    size_t seed = 0;
    boost::hash_combine(seed, rs.loc_node);
    boost::hash_combine(seed, rs.loc_prev);
    boost::hash_combine(seed, rs.assist_type);
    boost::hash_combine(seed, rs.assist_loc);
    return seed;
  }

  /* State */

  bool State::operator<(const utexas_planning::State& other_base) const {
    try {                                                                                                                
      const State& other = dynamic_cast<const State&>(other_base);                                             

      // Compare requests size.
      COMPARE_MEMBER(requests.size());
      // Then check if the vector contents are different.
      for (unsigned int i = 0; i < requests.size(); ++i) {
        COMPARE_MEMBER(requests[i]);
      }

      // Check robot size (should never be different).
      COMPARE_MEMBER(robots.size());
      // Then check if the vector contents are different.
      for (unsigned int i = 0; i < robots.size(); ++i) {
        COMPARE_MEMBER(robots[i]);
      }

      return false;
    } catch(const std::bad_cast& exp) {                                                                                  
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }
  }

  bool State::operator==(const utexas_planning::State& other_base) const {
    try {                                                                                                                
      const State& other = dynamic_cast<const State&>(other_base);                                             
      return (requests == other.requests) && (robots == other.robots);
    } catch(const std::bad_cast& exp) {                                                                                  
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }
  }

  bool State::operator>(const utexas_planning::State& other) const {
    return (*this == other || *this < other) ? false : true;
  }

  std::size_t State::hash() const {
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(robots.begin(), robots.end()));
    boost::hash_combine(seed, boost::hash_range(requests.begin(), requests.end()));
    return seed;
  }

  void State::serialize(std::ostream& stream) const {
    stream << "[Req: ";
    for (unsigned int i = 0; i < requests.size(); ++i) {
      stream << requests[i] << ", ";
    }
    stream << "Robots: ";
    for (unsigned int i = 0; i < robots.size(); ++i) {
      stream << robots[i] << ", ";
    }
    stream << "]";
  }

} /* utexas_guidance */

#undef COMPARE
#undef COMPARE_MEMBER
