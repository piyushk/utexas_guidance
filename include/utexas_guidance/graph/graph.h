#ifndef UTEXAS_GUIDANCE_GRAPH_H
#define UTEXAS_GUIDANCE_GRAPH_H

#include <boost/lexical_cast.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/labeled_graph.hpp>

#include <QGLViewer/qglviewer.h>

namespace utexas_guidance {

  typedef boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian> Point3f;
  typedef boost::geometry::model::segment<Point3f> Segment3f;

  struct Vertex {
    Point3f location;
  };

  struct Edge {
    float weight;
  };

  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge> Graph;

  void writeGraphToFile(const std::string &filename, const Graph& graph);

  void readGraphFromFile(const std::string &filename, Graph& graph);

  Point3f getLocationFromGraphId(int idx, const Graph& graph);

  int getClosestIdOnGraph(const Point3f &point, const Graph &graph, float threshold = 0.0f);

  int getClosestIdOnGraphFromEdge(const Point3f &point, const Graph &graph, int prev_graph_id);

  int getClosestEdgeOnGraphGivenId(const Point3f& point, const Graph &graph, int one_graph_id);

  float getNodeAngle(int u, int v, const Graph &graph);

  float getAbsoluteAngleDifference(float angle1, float angle2);

  bool onSameFloor(int idx1, int idx2, const Graph& graph);

  float getEuclideanDistance(int u, int v, const Graph &graph);

  float getShortestPathWithDistance(int start_idx,
                                    int goal_idx,
                                    std::vector<int> &path_from_goal,
                                    const Graph &graph);

  float getShortestPathDistance(int start_idx, int goal_idx, const Graph &graph);

  void getAllShortestPaths(std::vector<std::vector<float> > &shortest_distances,
                           std::vector<std::vector<std::vector<int> > > &shortest_paths,
                           const Graph& graph);

  void getAdjacentVertices(int v, const Graph& graph, std::vector<int>& adjacent_vertices);

  void getAdjacentVerticesOnSameFloor(int v, const Graph& graph, std::vector<int>& adjacent_vertices);

  void getAllAdjacentVertices(std::vector<std::vector<int> >& adjacent_vertices_map,
                              const Graph& graph);

  void getAllAdjacentVerticesOnSameFloor(std::vector<std::vector<int> >& adjacent_vertices_map,
                                         const Graph& graph);

  void draw(const Graph& graph,
            float linecolor_r = 0.5f,
            float linecolor_g = 0.5f,
            float linecolor_b = 0.5f,
            float vertexcolor_r = 1.0f,
            float vertexcolor_g = 0.0f,
            float vertexcolor_b = 0.0f,
            bool put_all_edges = true,
            std::vector<std::pair<int, int> > specific_edges = std::vector<std::pair<int, int> >());

} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_GRAPH_H */
