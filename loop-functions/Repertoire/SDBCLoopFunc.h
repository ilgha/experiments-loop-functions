/**
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef SDBC_LOOP_FUNC
#define SDBC_LOOP_FUNC

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/core/simulator/physics_engine/physics_model.h>


#include "../../src/CoreLoopFunctions.h"

using namespace argos;

class SDBCLoopFunc: public CoreLoopFunctions {
  public:
    SDBCLoopFunc();
    SDBCLoopFunc(const SDBCLoopFunc& orig);
    virtual ~SDBCLoopFunc();

    virtual void Destroy();
    virtual void Init(TConfigurationNode& t_tree);

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();
    virtual void Reset();

    Real GetObjectiveFunction();

    Real DistanceRobotRobot(CEPuckEntity* pc_robot_1, CEPuckEntity* pc_robot_2);
    Real DistanceRobotLight(CEPuckEntity* pc_robot, CLightEntity* pc_light);
    Real DistanceRobotPoint(CEPuckEntity* pc_robot, CVector2 c_point);
    Real DistanceRobotWall(CEPuckEntity* pc_robot, CBoxEntity* pc_wall);

    /* The BoxBoundaries are not correct, they need to be computed.
       This function computes the X and Y values to be removed from the wall's position (its center)
       to obtain the two extremity points. */
    CVector2 ComputeDeltaWall(CBoxEntity* pc_wall);

    /* Left wheel = index 0
       Right wheel = index 1 */
    Real ComputeLinearVelocity(CEPuckEntity* pc_robot);
    Real ComputeAngularVelocity(CEPuckEntity* pc_robot);

    CVector3 GetRandomPosition();

  private:
    Real m_fObjectiveFunction;

    UInt32 m_unExperimentLength;


    // Characterisation elements
    Real m_fArrayMeanRobotRobotDistance[];
    Real m_fArrayMeanRobotRobotMinDistance[];
    Real m_fArrayMeanRobotWallsDistance[];
    Real m_fArrayMeanRobotLightDistance[];
    Real m_fArrayMeanRobotLinearVelocity[];
    Real m_fArrayMeanRobotAngularVelocity[];
};

#endif
