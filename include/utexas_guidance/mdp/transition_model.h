#ifndef UTEXAS_GUIDANCE_TRANSITION_MODEL_H
#define UTEXAS_GUIDANCE_TRANSITION_MODEL_H

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// #include <utexas_guidance/common.h>
// #include <utexas_guidance/mrn/common.h>

#include <utexas_guidance/graph/graph.h>
#include <utexas_guidance/mdp/common.h>
#include <utexas_guidance/mdp/structures.h>

#include <utexas_planning/common/rng.h>

namespace utexas_guidance {

  class HumanDecisionModel {

    public:

      typedef boost::shared_ptr<HumanDecisionModel> Ptr;
      typedef boost::shared_ptr<const HumanDecisionModel> ConstPtr;

      HumanDecisionModel(const utexas_guidance::Graph& graph, float decision_variance_multiplier = 1.0f);
      virtual ~HumanDecisionModel();

      virtual int getNextNode(const RequestState& state, RNG &rng) const;

    private:

      utexas_guidance::Graph graph_;
      std::vector<std::vector<int> > adjacent_vertices_map_;
      std::vector<std::vector<int> > adjacent_vertices_on_same_floor_map_;
      float decision_variance_multiplier_;

  };

  class TaskGenerationModel {

    public:

      typedef boost::shared_ptr<TaskGenerationModel> Ptr;
      typedef boost::shared_ptr<TaskGenerationModel> ConstPtr;

      virtual ~TaskGenerationModel();
      virtual void generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng) const = 0;

  };

  class RandomTaskGenerationModel : public TaskGenerationModel {

    public:

      typedef boost::shared_ptr<RandomTaskGenerationModel> Ptr;
      typedef boost::shared_ptr<RandomTaskGenerationModel> ConstPtr;

      RandomTaskGenerationModel(const std::string& robot_home_base_file,
                                const utexas_guidance::Graph &graph,
                                float task_utility,
                                float task_time,
                                bool home_base_only = false);
      virtual ~RandomTaskGenerationModel();

      virtual void generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng) const;

    private:

      void cacheNewGoalsByDistance(const utexas_guidance::Graph& graph);

      std::vector<int> robot_home_base_;
      std::vector<std::vector<std::vector<int> > > goals_by_distance_;

      float task_utility_;
      float task_time_;
      bool home_base_only_;

  };

  class FixedTaskGenerationModel : public TaskGenerationModel {

    public:

      typedef boost::shared_ptr<FixedTaskGenerationModel> Ptr;
      typedef boost::shared_ptr<FixedTaskGenerationModel> ConstPtr;

      FixedTaskGenerationModel(const std::string& task_file,
                               float task_utility,
                               float task_time);
      virtual ~FixedTaskGenerationModel();

      virtual void generateNewTaskForRobot(int robot_id, RobotState &robot, RNG &rng) const;

    private:

      std::vector<std::vector<int> > tasks_;

      float task_utility_;
      float task_time_;

  };

  class MotionModel {

    public:

      typedef boost::shared_ptr<MotionModel> Ptr;
      typedef boost::shared_ptr<const MotionModel> ConstPtr;

      MotionModel(const utexas_guidance::Graph graph,
                  float avg_human_speed,
                  float avg_robot_speed,
                  float avg_elevator_human_speed,
                  float avg_elevator_robot_speed);

      virtual ~MotionModel();
      virtual void move(State &state,
                        const HumanDecisionModel::ConstPtr& human_decision_model,
                        const TaskGenerationModel::ConstPtr& task_generation_model,
                        RNG &rng,
                        float &total_time) const;

      virtual float getHumanSpeed();
      virtual float getRobotSpeed();
      virtual float getHumanSpeedInElevator();
      virtual float getRobotSpeedInElevator();

    private:

      utexas_guidance::Graph graph_;
      std::vector<std::vector<std::vector<int> > > shortest_paths_;
      std::vector<std::vector<float> > shortest_distances_;

      float human_speed_;
      float robot_speed_;
      float elevator_human_speed_;
      float elevator_robot_speed_;

  };

} /* bwi_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_TRANSITION_MODEL_H */
