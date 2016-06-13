#ifndef UTEXAS_GUIDANCE_COMMON_H
#define UTEXAS_GUIDANCE_COMMON_H

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

  int selectBestRobotForTask(const State& state, 
                             int destination, 
                             float human_speed, 
                             float robot_speed,
                             const std::vector<std::vector<float> > &shortest_distances,
                             bool &reach_in_time);

  inline void readRobotHomeBase(const std::string& robot_home_base_file, std::vector<int> &robot_home_base) {
    robot_home_base.clear();
    std::ifstream fin(robot_home_base_file.c_str());
    int home_base;
    fin >> home_base;
    while (!fin.eof()) {
      robot_home_base.push_back(home_base);
      fin >> home_base;
    }
    fin.close();
  }

  inline int getColocatedRobotId(const State& state, const RequestState &rs) {
    for (int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
      if ((state.robots[robot_id].help_destination == rs.loc_node) &&
          isRobotExactlyAt(state.robots[robot_id], rs.loc_node) &&
          (rs.loc_p == 1.0f)) {
        return robot_id;
      }
    }
    return NONE;
  }

} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_COMMON_H */
