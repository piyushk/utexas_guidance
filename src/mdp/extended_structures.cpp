#include <utexas_guidance/mdp/extended_structures.h>
#include <utexas_planning/common/exceptions.h>

#define COMPARE_MEMBER(x) if(x < (other.x)) return true; \
                          if(x > (other.x)) return false;

namespace utexas_guidance {

  /* ExtendedState */

  ExtendedState::ExtendedState() {} 
  ExtendedState::ExtendedState(const State& state) : State(state), prev_action(WAIT) {}

  ExtendedState::~ExtendedState() {}

  bool ExtendedState::operator<(const utexas_planning::State& other_base) const {
    try {                                                                                                                
      const ExtendedState& other = dynamic_cast<const ExtendedState&>(other_base);                                             
      if (State::operator==(other)) {
        COMPARE_MEMBER(prev_action.type);
        COMPARE_MEMBER(prev_action.robot_id);
        COMPARE_MEMBER(prev_action.node);
        COMPARE_MEMBER(released_locations.size());

        for (unsigned int i = 0; i < released_locations.size(); ++i) {
          COMPARE_MEMBER(released_locations[i]);
        }
        return false;
      } else {
        return State::operator<(other);
      }
    } catch(const std::bad_cast& exp) {                                                                                  
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::ExtendedState");
    }
  }

  bool ExtendedState::operator==(const utexas_planning::State& other_base) const {
    try {                                                                                                                
      const ExtendedState& other = dynamic_cast<const ExtendedState&>(other_base);                                             
      return (requests == other.requests) && (robots == other.robots);
    } catch(const std::bad_cast& exp) {                                                                                  
      throw utexas_planning::DowncastException("utexas_planning::State", "utexas_guidance::ExtendedState");
    }
  }

  std::size_t ExtendedState::hash() const {
    std::size_t seed = State::hash();
    boost::hash_combine(seed, prev_action.hash());
    boost::hash_combine(seed, prev_action.hash());
    boost::hash_range(seed, released_locations.begin(), released_locations.end());
    return seed;
  }

  utexas_planning::State::Ptr ExtendedState::cloneImpl() const {
    ExtendedState::Ptr clone(new ExtendedState);
    clone->requests = requests;
    clone->robots = robots;
    clone->prev_action = prev_action;
    clone->released_locations = released_locations;
    return clone;
  }

}
