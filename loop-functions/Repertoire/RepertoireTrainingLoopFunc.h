/**
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef REP_TRAIN_LOOP_FUNC
#define REP_TRAIN_LOOP_FUNC

#include <numeric>
#include <iostream>
#include <fstream>

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/plugins/simulator/entities/light_sensor_equipped_entity.h>
#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_light_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_ground_sensor.h>

#include "../../src/CoreLoopFunctions.h"
#include "./Objects/Box.h"


using namespace argos;

class RepertoireTrainingLoopFunc : public CoreLoopFunctions {
  public:

    RepertoireTrainingLoopFunc();
    RepertoireTrainingLoopFunc(const RepertoireTrainingLoopFunc& orig);
    virtual ~RepertoireTrainingLoopFunc();

    virtual void Destroy();
    virtual void Init(TConfigurationNode& t_tree);

    virtual argos::CColor GetFloorColor(const CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();
    virtual void Reset();

    Real GetObjectiveFunction();

    std::vector<Real> GetSDBC();

    CVector3 GetRandomPosition();

  private:

    Real DistanceRobotRobot(CEPuckEntity* pc_robot_1, CEPuckEntity* pc_robot_2);
    Real DistanceRobotWall(CEPuckEntity* pc_robot, CBoxEntity* pc_wall);

    /* The BoxBoundaries are not correct, they need to be computed.
       This function computes the X and Y values to be removed from the wall's position (its center)
       to obtain the two extremity points. */
    CVector2 ComputeDeltaWall(CBoxEntity* pc_wall);

    /* Left wheel = index 0
       Right wheel = index 1 */
    Real ComputeLinearVelocity(CEPuckEntity* pc_robot);
    Real ComputeAngularVelocity(CEPuckEntity* pc_robot);

    Real ComputeMeanValueFeature(UInt8 un_index_feature);
    Real ComputeStandardDeviationValueFeature(UInt8 un_index_feature);

    void AddObstacle(Box* pc_obstacle);

    // Environments elements
    Real m_fPatchRadius;
    CVector2 m_cPatchPosition;
    CColor m_cPatchColor;
    std::vector<CVector2> m_vecPossiblePatchesPositions;
    std::vector<CColor> m_vecPossiblePatchesColors;
    std::vector<Real> m_vecPossibleLightIntensities;
    std::vector<Box*> m_vecPossibleObstacles;


    // Quality metric
    Real m_fQualityMetric;
    UInt32 m_unExperimentLength;
    UInt32 m_unNumberCollisions;


    // Characterisation elements
    Real m_fBound = 3.0f;

    UInt8 m_unNumberFeatures;
    bool m_boolSmaplingFeatures; // If true, number of robots randomly selected and SDBC only contains mean values, no SDs.
    std::vector<std::vector<Real>> m_vecAllSDBC;
    std::vector<Real> m_vecFinalSDBC;

    std::vector<Real> m_vecFeatureScalingFactorMeans;
    std::vector<Real> m_vecFeatureScalingFactorStandardDevs;

};

#endif
