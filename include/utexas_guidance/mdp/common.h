#ifndef UTEXAS_GUIDANCE_MDP_COMMON_H
#define UTEXAS_GUIDANCE_MDP_COMMON_H

#include <utexas_guidance/mdp/structures.h>

#include <fstream>
#include <vector>

namespace utexas_guidance {

  const static int NONE = -1;

  bool isShortestPathThroughLocU(int loc_u,
                                 int loc_v,
                                 float loc_p,
                                 int destination,
                                 const std::vector<std::vector<float> > &shortest_distances);

  float getTrueDistanceTo(int loc_u,
                          int loc_v,
                          float loc_p,
                          int destination,
                          const std::vector<std::vector<float> > &shortest_distances);

  float getTrueDistanceTo(int loc_u,
                          int loc_v,
                          float loc_p,
                          int destination,
                          const std::vector<std::vector<float> > &shortest_distances,
                          bool &shortest_path_through_u);

  inline bool isRobotExactlyAt(const RobotState& robot, int loc) {
    return (((robot.loc_u == loc) && (robot.loc_p == 0.0f)) ||
            ((robot.loc_v == loc) && (robot.loc_p == 1.0f)));
  }

} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_MDP_COMMON_H */
