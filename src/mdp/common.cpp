#include <utexas_guidance/mdp/common.h>

namespace utexas_guidance {

  bool isShortestPathThroughLocU(int loc_u,
                                 int loc_v,
                                 float loc_p,
                                 int destination,
                                 const std::vector<std::vector<float> > &shortest_distances) {
    bool ret_val;
    getTrueDistanceTo(loc_u, loc_v, loc_p, destination, shortest_distances, ret_val);
    return ret_val;
  }

  float getTrueDistanceTo(int loc_u,
                          int loc_v,
                          float loc_p,
                          int destination,
                          const std::vector<std::vector<float> > &shortest_distances) {
    bool unused_ret_val;
    return getTrueDistanceTo(loc_u, loc_v, loc_p, destination, shortest_distances, unused_ret_val);
  }

  float getTrueDistanceTo(int loc_u,
                          int loc_v,
                          float loc_p,
                          int destination,
                          const std::vector<std::vector<float> > &shortest_distances,
                          bool &shortest_path_through_u) {

    float current_edge_distance = shortest_distances[loc_u][loc_v];
    float distance_to_u = loc_p * current_edge_distance;
    float distance_to_v = current_edge_distance - distance_to_u;

    float distance_from_u = shortest_distances[loc_u][destination] + distance_to_u;
    float distance_from_v = shortest_distances[loc_v][destination] + distance_to_v;
    shortest_path_through_u = (distance_from_u < distance_from_v);

    return (shortest_path_through_u) ? distance_from_u : distance_from_v;
  }

  void getColocatedRobotRequestIds(const State& state,
                                   std::vector<std::pair<int, int> >& robot_request_ids) {

    // Figure out if there is a robot at the current position
    for (int robot_id = 0; robot_id < state.robots.size(); ++robot_id) {
      if (!(state.robots[robot_id].is_leading_person)) {
        for (int request_id = 0; request_id < state.requests.size(); ++request_id) {
          if ((state.robots[robot_id].help_destination == state.requests[request_id].loc_node) &&
              isRobotExactlyAt(state.robots[robot_id], state.requests[request_id].loc_node) &&
              (state.requests[request_id].loc_p == 1.0f) &&
              (state.requests[request_id].assist_type == NONE)) {
            robot_request_ids.push_back(std::pair<int, int>(robot_id, request_id));
          }
        }
      }
    }

  }

} /* utexas_guidance */
