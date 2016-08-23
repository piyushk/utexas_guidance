#include <utexas_guidance/mdp/structures.h>
#include <utexas_planning/common/exceptions.h>

#define COMPARE(x) if((l.x) < (r.x)) return true; \
                   if((l.x) > (r.x)) return false;

#define COMPARE_MEMBER(x) if(x < (other.x)) return true; \
                          if(x > (other.x)) return false;

namespace utexas_guidance {

  /* Action */

  Action::Action(ActionType a, int robot_id, int node, int request_id, int old_help_destination) :
    type(a), robot_id(robot_id), node(node), request_id(request_id), old_help_destination(old_help_destination) {}
  Action::~Action() {}

  bool Action::operator<(const utexas_planning::Action& other_base) const {
    try {
      const Action& other = dynamic_cast<const Action&>(other_base);
      COMPARE_MEMBER(type);
      COMPARE_MEMBER(robot_id);
      COMPARE_MEMBER(node);
      COMPARE_MEMBER(request_id);
      COMPARE_MEMBER(old_help_destination);
      return false;
    } catch(const std::bad_cast& exp) {
      throw utexas_planning::DowncastException("utexas_planning::Action", "utexas_guidance::Action");
    }
  }

  bool Action::operator==(const utexas_planning::Action& other_base) const {
    try {
      const Action& other = dynamic_cast<const Action&>(other_base);
      return ((type == other.type) &&
              (robot_id == other.robot_id) &&
              (node == other.node) &&
              (request_id == other.request_id) &&
              (old_help_destination == other.old_help_destination));        
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
    std::size_t seed = 0;
    boost::hash_combine(seed, type);
    boost::hash_combine(seed, robot_id);
    boost::hash_combine(seed, node);
    boost::hash_combine(seed, request_id);
    boost::hash_combine(seed, old_help_destination);
    return seed;
  }

  void Action::serialize(std::ostream& stream) const {
    stream << "[" << ACTION_NAMES[type];
    if (type != WAIT) {
      stream << " " << robot_id;
      if (type == LEAD_PERSON || type == DIRECT_PERSON) {
        stream << ":" << request_id;
      }
      if (type != RELEASE_ROBOT) {
        stream << "->" << node;
      }
      // stream << "," << request_id << "," << old_help_destination;
    }
    stream << "]";
  }

  std::string Action::getName() const {
    return std::string("utexas_guidance::Action");
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
    COMPARE(is_leading_person);
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
            (l.is_leading_person == r.is_leading_person) &&
            (l.help_destination == r.help_destination));
  }

  bool operator>(const RobotState& l, const RobotState& r) {
    return (l == r || l < r) ? false : true;
  }

  std::ostream& operator<<(std::ostream& stream, const RobotState& rs) {
    stream << "[(" << rs.loc_u << "->" << rs.loc_v << "," << rs.loc_p << "), (" <<
      rs.tau_d << "," <<  rs.tau_t << "," << rs.tau_total_task_time << "," << rs.tau_u << "), " <<
      rs.is_leading_person << "->" << rs.help_destination << "]";
    return stream;
  }

  std::size_t hash_value(const RobotState &rs) {
    std::size_t seed = 0;
    boost::hash_combine(seed, rs.loc_u);
    boost::hash_combine(seed, rs.loc_v);
    boost::hash_combine(seed, rs.loc_p);
    boost::hash_combine(seed, rs.tau_d);
    boost::hash_combine(seed, rs.tau_t);
    boost::hash_combine(seed, rs.tau_total_task_time);
    boost::hash_combine(seed, rs.tau_u);
    boost::hash_combine(seed, rs.is_leading_person);
    boost::hash_combine(seed, rs.help_destination);
    return seed;
  }

  /* RequestState */

  bool operator<(const RequestState& l, const RequestState& r) {
    COMPARE(request_id);
    COMPARE(loc_node);
    COMPARE(loc_prev);
    COMPARE(loc_p);
    COMPARE(assist_type);
    COMPARE(assist_loc);
    COMPARE(goal);
    return false;
  }

  bool operator==(const RequestState& l, const RequestState& r) {
    return ((l.request_id == r.request_id) &&
            (l.loc_node == r.loc_node) &&
            (l.loc_prev == r.loc_prev) &&
            (l.loc_p == r.loc_p) &&
            (l.assist_type == r.assist_type) &&
            (l.assist_loc == r.assist_loc) &&
            (l.goal == r.goal));
  }

  bool operator>(const RequestState& l, const RequestState& r) {
    return (l == r || l < r) ? false : true;
  }

  std::ostream& operator<<(std::ostream& stream, const RequestState& rs) {
    stream << "(<" << rs.loc_prev << "->" << rs.loc_node << ","  << rs.loc_p << ">, " <<
      "<"  << rs.assist_type << "," << rs.assist_loc << ">, " << rs.goal << ")";
    return stream;
  }

  std::size_t hash_value(const RequestState &rs) {
    std::size_t seed = 0;
    boost::hash_combine(seed, rs.request_id);
    boost::hash_combine(seed, rs.loc_node);
    boost::hash_combine(seed, rs.loc_prev);
    boost::hash_combine(seed, rs.loc_p);
    boost::hash_combine(seed, rs.assist_type);
    boost::hash_combine(seed, rs.assist_loc);
    boost::hash_combine(seed, rs.goal);
    return seed;
  }

  int generateNewRequestId() {
    static int request_id = 0;
    return request_id++;
  }

  /* State */

  State::~State() {}

  bool State::operator<(const utexas_planning::State& other_base) const {
    try {
      const State& other = dynamic_cast<const State&>(other_base);

      // Compare requests size.
      COMPARE_MEMBER(actions_since_wait.size());
      for (unsigned int i = 0; i < actions_since_wait.size(); ++i) {
        COMPARE_MEMBER(actions_since_wait[i]);
      }

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
      return (requests == other.requests) && 
        (robots == other.robots) && 
        (actions_since_wait == other.actions_since_wait);
    } catch(const std::bad_cast& exp) {
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::State");
    }
  }

  bool State::operator>(const utexas_planning::State& other) const {
    return (*this == other || *this < other) ? false : true;
  }

  std::size_t State::hash() const {
    std::size_t seed = 0;
    boost::hash_range(seed, robots.begin(), robots.end());
    boost::hash_range(seed, requests.begin(), requests.end());
    for (unsigned int i = 0; i < actions_since_wait.size(); ++i) {
      boost::hash_combine(seed, actions_since_wait[i].hash());
    }
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
    stream << "ActionSinceWait: ";
    for (unsigned int i = 0; i < actions_since_wait.size(); ++i) {
      actions_since_wait[i].serialize(stream);
      stream << ", ";
    }
    stream << "]";
  }

  std::string State::getName() const {
    return std::string("utexas_guidance::State");
  }

  utexas_planning::State::Ptr State::cloneImpl() const {
    State::Ptr clone(new State);
    clone->requests = requests;
    clone->robots = robots;
    clone->actions_since_wait = actions_since_wait;
    return clone;
  }

  /* Reward Metrics - Things apart from reward that we care about. */
  std::map<std::string, std::string> RewardMetricsasMap() const {
    std::map<std::string, std::string> metric_map;
    metric_map["utility_loss"] = boost::lexical_cast<std::string>(utility_loss);
    metric_map["time_loss"] = boost::lexical_cast<std::string>(time_loss);
    metric_map["reward_normalized"] = boost::lexical_cast<std::string>(time_loss);
    metric_map["utility_loss_normalized"] = boost::lexical_cast<std::string>(time_loss);
    metric_map["time_loss_normalized"] = boost::lexical_cast<std::string>(time_loss);
    return metric_map;
  }

} /* utexas_guidance */

#undef COMPARE
#undef COMPARE_MEMBER
