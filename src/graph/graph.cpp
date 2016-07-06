#include <yaml-cpp/yaml.h>
#include <fstream>
#include <boost/geometry/geometry.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/foreach.hpp>

#include <GL/freeglut.h>

#include <utexas_guidance/graph/graph.h>

namespace utexas_guidance {

  void draw(const Graph& graph,
            float scale,
            float linecolor_r,
            float linecolor_g,
            float linecolor_b,
            float vertexcolor_r,
            float vertexcolor_g,
            float vertexcolor_b,
            bool put_all_edges,
            std::vector<std::pair<int, int> > specific_edges) {

    glDisable(GL_LIGHTING);
    Graph::vertex_iterator vi, vend;
    int count = 0;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi, ++count) {
      Point3f location = graph[*vi].location;
      // Draw the edges from this vertex
      std::vector<int> adj_vertices;
      getAdjacentVertices(count, graph, adj_vertices);
      BOOST_FOREACH(int adj_vtx, adj_vertices) {
        if (adj_vtx > count) {
          bool allow_edge = put_all_edges;
          allow_edge = allow_edge || std::find(specific_edges.begin(), specific_edges.end(),
                                               std::make_pair(count, adj_vtx)) != specific_edges.end();
          allow_edge = allow_edge || std::find(specific_edges.begin(), specific_edges.end(),
                                               std::make_pair(adj_vtx, count)) != specific_edges.end();
          if (allow_edge) {
            Point3f location2 = getLocationFromGraphId(adj_vtx, graph);
            glBegin(GL_LINES);
            glColor3f(linecolor_r, linecolor_g, linecolor_b);
            float loc1[] = {location.get<0>() * scale, location.get<1>() * scale, location.get<2>() * scale};
            float loc2[] = {location2.get<0>() * scale, location2.get<1>() * scale, location2.get<2>() * scale};
            glVertex3fv(loc1);
            glVertex3fv(loc2);
            glEnd();
          }
        }
      }
    }

    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
      Point3f location = graph[*vi].location;
      glPushMatrix();
      glColor3f(vertexcolor_r, vertexcolor_g, vertexcolor_b);
      glTranslatef(location.get<0>() * scale, location.get<1>() * scale, location.get<2>() * scale);
      glScalef(0.09f,-0.08f, location.get<2>() * scale);
      unsigned char text[] = "text\0"; 
      glutStrokeString(GLUT_STROKE_MONO_ROMAN, text);
      /* glutSolidSphere(0.2f * scale, 10, 10); */
      glPopMatrix();
    }

    glEnable(GL_LIGHTING);
  }

  // void drawArrowOnImage(cv::Mat &image, const cv::Point3f &arrow_center, float orientation,
  //                       const cv::Scalar &color, int size, int thickness) {

  //   cv::Point arrow_start = arrow_center +
  //     cv::Point3f(size * cosf(orientation + M_PI/2),
  //                 size * sinf(orientation + M_PI/2));
  //   cv::Point arrow_end = arrow_center -
  //     cv::Point3f(size * cosf(orientation + M_PI/2),
  //                 size * sinf(orientation + M_PI/2));

  //   cv::line(image, arrow_start, arrow_end, color, thickness, CV_AA);

  //   // http://mlikihazar.blogspot.com/2013/02/draw-arrow-opencv.html
  //   cv::Point p(arrow_start), q(arrow_end);

  //   //Draw the first segment
  //   float angle = atan2f(p.y - q.y, p.x - q.x);
  //   p.x = (int) (q.x + (size/2 - 1) * cos(angle + M_PI/4));
  //   p.y = (int) (q.y + (size/2 - 1) * sin(angle + M_PI/4));
  //   cv::line(image, p, q, color, thickness, CV_AA);

  //   //Draw the second segment
  //   p.x = (int) (q.x + (size/2 + 1) * cos(angle - M_PI/4));
  //   p.y = (int) (q.y + (size/2 + 1) * sin(angle - M_PI/4));
  //   cv::line(image, p, q, color, thickness, CV_AA);

  // }

  // void drawArrowOnGraph(cv::Mat &image, const Graph& graph,
  //     std::pair<int, float> arrow, uint32_t map_width, uint32_t map_height,
  //     cv::Scalar color, uint32_t orig_x, uint32_t orig_y) {

  //   float orientation = arrow.second;
  //   Point3f loc = getLocationFromGraphId(arrow.first, graph);
  //   cv::Point node_loc(loc.x + orig_x, loc.y + orig_y);
  //   cv::Point map_center(orig_x + map_width / 2, orig_y + map_height / 2);

  //   cv::Point arrow_center_1 = node_loc +
  //     cv::Point(25 * cosf(orientation), 25 * sinf(orientation));
  //   cv::Point arrow_center_2 = node_loc -
  //     cv::Point(25 * cosf(orientation), 25 * sinf(orientation));
  //   cv::Point arrow_center = (boost::geometry::distance(arrow_center_2, map_center) <
  //       boost::geometry::distance(arrow_center_1, map_center)) ? arrow_center_2 :
  //     arrow_center_1;

  //   drawArrowOnImage(image, arrow_center, orientation, color, 20, 3);

  // }

  // void drawCircleOnGraph(cv::Mat &image, const Graph& graph,
  //     int node, cv::Scalar color,
  //     uint32_t orig_x, uint32_t orig_y) {
  //   Point3f loc = getLocationFromGraphId(node, graph);
  //   cv::Point circle_loc(loc.x + orig_x, loc.y + orig_y);
  //   cv::circle(image, circle_loc, 15, color, 2, CV_AA);
  // }

  // void drawSquareOnGraph(cv::Mat &image, const Graph& graph,
  //     int node, cv::Scalar color,
  //     uint32_t orig_x, uint32_t orig_y, int size, int thickness) {
  //   Point3f loc = getLocationFromGraphId(node, graph);
  //   cv::Point square_loc(loc.x + orig_x, loc.y + orig_y);
  //   cv::Rect rect(square_loc.x - size/2, square_loc.y - size/2, size, size);
  //   cv::rectangle(image, rect, color, thickness, CV_AA);
  // }

  void writeGraphToFile(const std::string &filename, const Graph& graph) {

    std::map<Graph::vertex_descriptor, int> vertex_map;
    int count = 0;
    Graph::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
      vertex_map[*vi] = count;
      count++;
    }

    count = 0;
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
      out << YAML::BeginMap;
      Point3f loc = graph[*vi].location;
      out << YAML::Key << "id" << YAML::Value << count;
      out << YAML::Key << "x" << YAML::Value << loc.get<0>();
      out << YAML::Key << "y" << YAML::Value << loc.get<1>();
      out << YAML::Key << "z" << YAML::Value << loc.get<2>();
      out << YAML::Key << "edges" << YAML::Value << YAML::BeginSeq;
      Graph::adjacency_iterator ai, aend;
      for (boost::tie(ai, aend) = boost::adjacent_vertices((Graph::vertex_descriptor)*vi, graph); ai != aend; ++ai) {
        out << vertex_map[*ai];
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
      count++;
    }
    out << YAML::EndSeq;

    std::ofstream fout(filename.c_str());
    fout << out.c_str();
    fout.close();
  }

  void readGraphFromFile(const std::string &filename, Graph& graph) {

    std::vector<Point3f> vertices;
    std::vector<std::vector<int> > edges;

    std::ifstream fin(filename.c_str());
    YAML::Node doc;
    doc = YAML::Load(fin);
    for (int i = 0; i < doc.size(); ++i) {
      Point3f point;
      point.set<0>(doc[i]["x"].as<float>());
      point.set<1>(doc[i]["y"].as<float>());
      point.set<2>(doc[i]["z"].as<float>());
      vertices.push_back(point);
      std::vector<int> v_edges;
      const YAML::Node& edges_node = doc[i]["edges"];
      for (int j = 0; j < edges_node.size(); ++j) {
        int t = edges_node[j].as<int>();
        if (t > i) { // Only add edge one way
          v_edges.push_back(t);
        }
      }
      edges.push_back(v_edges);
    }
    fin.close();

    // Construct the graph object
    for (unsigned int i = 0; i < vertices.size(); ++i) {
      Graph::vertex_descriptor vi = boost::add_vertex(graph);
      graph[vi].location = vertices[i];
    }

    for (unsigned int i = 0; i < edges.size(); ++i) {
      for (unsigned int j = 0; j < edges[i].size(); ++j) {
        Graph::vertex_descriptor vi,vj;
        vi = boost::vertex(i, graph);
        vj = boost::vertex(edges[i][j], graph);
        Graph::edge_descriptor e; bool b;
        boost::tie(e,b) = boost::add_edge(vi, vj, graph);

        // TODO elevators will need additional weight here.
        graph[e].weight = boost::geometry::distance(graph[vi].location, graph[vj].location);
      }
    }

    // std::cout << "Read graph with " << vertices.size() << " vertices." << std::endl;
  }

  Point3f getLocationFromGraphId(int idx, const Graph& graph) {
    Graph::vertex_descriptor i = boost::vertex(idx, graph);
    return graph[i].location;
  }

  int getClosestIdOnGraph(const Point3f &point, const Graph &graph, float threshold) {
    Graph::vertex_iterator vi, vend;
    int count = 0, min_idx = -1;
    float min_distance = std::numeric_limits<float>::max();
    for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; ++vi) {
      Point3f location = graph[*vi].location;
      float distance = boost::geometry::distance(point, location);
      if (distance <= min_distance) {
        min_distance = distance;
        min_idx = count;
      }
      count++;
    }
    if (min_distance < threshold || threshold == 0.0f) {
      return min_idx;
    } else {
      return -1;
    }
  }

  int getClosestIdOnGraphFromEdge(const Point3f& point,
      const Graph &graph, int prev_graph_id) {

    Point3f location = getLocationFromGraphId(prev_graph_id, graph);

    int other_graph_id = getClosestEdgeOnGraphGivenId(point, graph, prev_graph_id);
    Point3f other_location = getLocationFromGraphId(other_graph_id, graph);

    if (boost::geometry::distance(point, location) < boost::geometry::distance(point, other_location)) {
      return prev_graph_id;
    } else {
      return other_graph_id;
    }

  }

  int getClosestEdgeOnGraphGivenId(const Point3f& point, const Graph &graph, int one_graph_id) {

    boost::property_map<Graph, boost::vertex_index_t>::type
        indexmap = boost::get(boost::vertex_index, graph);

    Graph::vertex_descriptor prev_vertex = boost::vertex(one_graph_id, graph);
    Point3f location = graph[prev_vertex].location;

    int min_idx = -1;
    float min_distance = std::numeric_limits<float>::max();

    Graph::adjacency_iterator ai, aend;
    for (boost::tie(ai, aend) = boost::adjacent_vertices(prev_vertex, graph);
        ai != aend; ++ai) {
      Segment3f segment(location, graph[*ai].location);

      float distance = boost::geometry::distance(point, segment);
      if (distance < min_distance) {
        min_distance = distance;
        min_idx = indexmap[*ai];
      }
    }

    return min_idx;
  }

  float getNodeAngle(int u, int v, const Graph & graph) {
    Graph::vertex_descriptor vd_u = boost::vertex(u, graph);
    Graph::vertex_descriptor vd_v = boost::vertex(v, graph);
    if (graph[vd_v].location.get<2>() != graph[vd_u].location.get<2>()) {
      throw std::runtime_error("can't calculate angle for nodes not on the same height");
    }
    return atan2f(graph[vd_v].location.get<1>() - graph[vd_u].location.get<1>(),
        graph[vd_v].location.get<0>() - graph[vd_u].location.get<0>());
  }

  bool onSameFloor(int idx1, int idx2, const Graph& graph) {
    Point3f loc1 = getLocationFromGraphId(idx1, graph);
    Point3f loc2 = getLocationFromGraphId(idx2, graph);
    return loc1.get<2>() == loc2.get<2>();
  }

  float getAbsoluteAngleDifference(float angle1, float angle2) {
    while (angle2 > angle1 + M_PI) angle2 -= 2 * M_PI;
    while (angle2 <= angle1 - M_PI) angle2 += 2 * M_PI;
    return fabs (angle1 - angle2);
  }

  float getEuclideanDistance(int u, int v, const Graph& graph) {
    Graph::vertex_descriptor vd_u = boost::vertex(u, graph);
    Graph::vertex_descriptor vd_v = boost::vertex(v, graph);
    return boost::geometry::distance(graph[vd_v].location, graph[vd_u].location);
  }

  float getShortestPathWithDistance(int start_idx,
                                    int goal_idx,
                                    std::vector<int> &path_from_goal,
                                    const Graph &graph) {

    utexas_guidance::Graph graph_copy(graph);
    // Perform Dijakstra from start_idx
    std::vector<Graph::vertex_descriptor> p(boost::num_vertices(graph_copy));
    std::vector<float> d(boost::num_vertices(graph_copy));
    Graph::vertex_descriptor s = boost::vertex(start_idx, graph_copy);

    boost::property_map<Graph, boost::vertex_index_t>::type indexmap = boost::get(boost::vertex_index, graph_copy);
    boost::property_map< Graph, float Edge::* >::type weightmap = boost::get(&Edge::weight, graph_copy);

    boost::dijkstra_shortest_paths(graph_copy,
                                   s,
                                   &p[0],
                                   &d[0],
                                   weightmap,
                                   indexmap,
                                   std::less<float>(),
                                   boost::closed_plus<float>(),
                                   (std::numeric_limits<float>::max)(),
                                   0,
                                   boost::default_dijkstra_visitor());

    // Look up the parent chain from the goal vertex to the start vertex
    path_from_goal.clear();

    Graph::vertex_descriptor g = boost::vertex(goal_idx, graph_copy);
    while (indexmap[p[g]] != start_idx) {
      path_from_goal.push_back(indexmap[p[g]]);
      g = p[g];
    }
    path_from_goal.push_back(start_idx);

    return d[goal_idx];
  }

  float getShortestPathDistance(int start_idx, int goal_idx, const Graph &graph) {
    std::vector<int> temp_path;
    return getShortestPathWithDistance(start_idx, goal_idx, temp_path, graph);
  }

  void getAllShortestPaths(std::vector<std::vector<float> > &shortest_distances,
                           std::vector<std::vector<std::vector<int> > > &shortest_paths,
                           const Graph& graph) {

    int num_vertices = boost::num_vertices(graph);
    shortest_paths.resize(num_vertices);
    shortest_distances.resize(num_vertices);

    for (int idx = 0; idx < num_vertices; ++idx) {
      shortest_distances[idx].resize(num_vertices);
      shortest_paths[idx].resize(num_vertices);
      for (int j = 0; j < num_vertices; ++j) {
        if (j == idx) {
          shortest_distances[idx][j] = 0;
          shortest_paths[idx][j].clear();
        } else {
          shortest_distances[idx][j] =
            getShortestPathWithDistance(idx, j, shortest_paths[idx][j], graph);
          // Post-process the shortest path - add goal, remove start and reverse
          shortest_paths[idx][j].insert(shortest_paths[idx][j].begin(), j); // Add j
          shortest_paths[idx][j].pop_back(); // Remove idx
          std::reverse(shortest_paths[idx][j].begin(), shortest_paths[idx][j].end());
        }
      }
    }
  }

  void getAdjacentVertices(int graph_id, const Graph& graph, std::vector<int>& adjacent_vertices) {
    adjacent_vertices.clear();
    boost::property_map<Graph, boost::vertex_index_t>::type indexmap = boost::get(boost::vertex_index, graph);
    Graph::vertex_descriptor vertex = boost::vertex(graph_id, graph);
    Graph::adjacency_iterator ai, aend;
    for (boost::tie(ai, aend) = boost::adjacent_vertices(vertex, graph); ai != aend; ++ai) {
      adjacent_vertices.push_back(indexmap[*ai]);
    }
  }

  void getAdjacentVerticesOnSameFloor(int graph_id, const Graph& graph, std::vector<int>& adjacent_vertices) {
    adjacent_vertices.clear();
    boost::property_map<Graph, boost::vertex_index_t>::type indexmap = boost::get(boost::vertex_index, graph);
    Graph::vertex_descriptor vertex = boost::vertex(graph_id, graph);
    Graph::adjacency_iterator ai, aend;
    for (boost::tie(ai, aend) = boost::adjacent_vertices(vertex, graph); ai != aend; ++ai) {
      if (onSameFloor(graph_id, indexmap[*ai], graph)) {
        adjacent_vertices.push_back(indexmap[*ai]);
      }
    }
  }
  void getAllAdjacentVertices(std::vector<std::vector<int> >& adjacent_vertices_map,
                              const Graph& graph) {
    adjacent_vertices_map.resize(boost::num_vertices(graph));
    for (int graph_id = 0; graph_id < boost::num_vertices(graph); ++graph_id) {
      std::vector<int> adjacent_vertices;
      getAdjacentVertices(graph_id, graph, adjacent_vertices);
      adjacent_vertices_map[graph_id] = std::vector<int>(adjacent_vertices.begin(), adjacent_vertices.end());
    }
  }

  void getAllAdjacentVerticesOnSameFloor(std::vector<std::vector<int> >& adjacent_vertices_map,
                                         const Graph& graph) {
    adjacent_vertices_map.resize(boost::num_vertices(graph));
    for (int graph_id = 0; graph_id < boost::num_vertices(graph); ++graph_id) {
      std::vector<int> adjacent_vertices;
      getAdjacentVerticesOnSameFloor(graph_id, graph, adjacent_vertices);
      adjacent_vertices_map[graph_id] = std::vector<int>(adjacent_vertices.begin(), adjacent_vertices.end());
    }
  }

} /* utexas_guidance */
