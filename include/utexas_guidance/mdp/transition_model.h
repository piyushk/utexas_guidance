#ifndef UTEXAS_GUIDANCE_TRANSITION_MODEL_H
#define UTEXAS_GUIDANCE_TRANSITION_MODEL_H

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// #include <utexas_guidance/common.h>
// #include <utexas_guidance/mrn/common.h>

#include <utexas_guidance/graph/graph.h>
#include <utexas_guidance/mdp/structures.h>

namespace utexas_guidance {

  class HumanDecisionModel {

    public: 

      typedef boost::shared_ptr<HumanDecisionModel> Ptr;
      typedef boost::shared_ptr<const HumanDecisionModel> ConstPtr;

      HumanDecisionModel(const utexas_guidance::Graph graph, float decision_variance_multiplier = 1.0f);
      virtual ~HumanDecisionModel();

      virtual int getNextNode(const State &state, RNG &rng);

    private:

      utexas_guidance::Graph graph_;
      std::map<int, std::vector<int> > adjacent_vertices_map_;
      float decision_variance_multiplier_;

  };

  class TaskGenerationModel {

    public:

      typedef boost::shared_ptr<TaskGenerationModel> Ptr;
      typedef boost::shared_ptr<TaskGenerationModel> ConstPtr;

      virtual ~TaskGenerationModel() = 0;
      virtual void generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng) = 0;

  };

  class FixedTaskGenerationModel : public TaskGenerationModel {

    public:

      typedef boost::shared_ptr<FixedTaskGenerationModel> Ptr;
      typedef boost::shared_ptr<FixedTaskGenerationModel> ConstPtr;

      FixedTaskGenerationModel(const std::string& task_file,
                               float task_utility);
      virtual ~TaskGenerationModel();

      virtual void generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng);

    private:

      std::vector<int> robot_home_base_;
      std::vector<std::vector<std::vector<int> > > goals_by_distance_;

      float task_utility_;
      bool home_base_only_;

  };

  class RandomTaskGenerationModel : public TaskGenerationModel {

    public:

      typedef boost::shared_ptr<RandomTaskGenerationModel> Ptr;
      typedef boost::shared_ptr<RandomTaskGenerationModel> ConstPtr;

      RandomTaskGenerationModel(const std::vector<int> robot_home_base,
                                const utexas_guidance::Graph &graph,
                                float task_utility,
                                bool home_base_only = false);
      virtual ~TaskGenerationModel();

      virtual void generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng);

    private:

      void cacheNewGoalsByDistance(const utexas_guidance::Graph &graph);

      std::vector<int> robot_home_base_;
      std::vector<std::vector<std::vector<int> > > goals_by_distance_;

      float task_utility_;
      bool home_base_only_;

  };

  class MotionModel {

    public:

      typedef boost::shared_ptr<MotionModel> Ptr;
      typedef boost::shared_ptr<const MotionModel> ConstPtr;

      MotionModel(const utexas_guidance::Graph graph, float avg_robot_speed, float avg_human_speed);

      virtual ~MotionModel();
      virtual bool move(State &state,
                        int next_node,
                        const TaskGenerationModel::Ptr &task_generation_model,
                        RNG &rng,
                        float &total_time);

      virtual float getHumanSpeed();
      virtual float getRobotSpeed();

    private:

      utexas_guidance::Graph graph_;
      std::vector<std::vector<std::vector<size_t> > > shortest_paths_;
      std::vector<std::vector<float> > shortest_distances_;

      float robot_speed_;
      float human_speed_;

  };

} /* bwi_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_TRANSITION_MODEL_H */
