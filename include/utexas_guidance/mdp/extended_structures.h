#ifndef UTEXAS_GUIDANCE_EXTENDED_STRUCTURS_H
#define UTEXAS_GUIDANCE_EXTENDED_STRUCTURS_H

#include <utexas_guidance/mdp/structures.h>

namespace utexas_guidance {

  class ExtendedState : public State {

  public:

    typedef boost::shared_ptr<ExtendedState> Ptr;
    typedef boost::shared_ptr<const ExtendedState> ConstPtr;

    ExtendedState();
    ExtendedState(const State& state);

    virtual ~ExtendedState();

    virtual bool operator<(const utexas_planning::State& other) const;
    virtual bool operator==(const utexas_planning::State& other) const;
    virtual std::size_t hash() const;

    std::vector<int> assigned_robots;
    /* Locations that have been released since the last Wait action. No robot can be assigned to these locations. */
    std::vector<int> released_locations;
    std::vector<int> unhelpful_robots;

  private:

    virtual utexas_planning::State::Ptr cloneImpl() const;

    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive& ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(State);
      ar & BOOST_SERIALIZATION_NVP(requests);
      ar & BOOST_SERIALIZATION_NVP(robots);
      ar & BOOST_SERIALIZATION_NVP(assigned_robots);
      ar & BOOST_SERIALIZATION_NVP(released_locations);
      ar & BOOST_SERIALIZATION_NVP(unhelpful_robots);
    }

  };
} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_EXTENDED_STRUCTURS_H */
