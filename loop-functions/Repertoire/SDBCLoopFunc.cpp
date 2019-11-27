/**
  * @file <loop-functions/AggregationTwoSpotsLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "SDBCLoopFunc.h"

/****************************************/
/****************************************/

SDBCLoopFunc::SDBCLoopFunc() {
  m_fObjectiveFunction = 0;

}

/****************************************/
/****************************************/

SDBCLoopFunc::SDBCLoopFunc(const SDBCLoopFunc& orig) {}

/****************************************/
/****************************************/

void SDBCLoopFunc::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
    TConfigurationNode cParametersNode;
    try {
      cParametersNode = GetNode(t_tree, "params");
      GetNodeAttributeOrDefault(cParametersNode, "experiment_length", m_unExperimentLength, (UInt32) 1);
    } catch(std::exception e) {
    }
    m_fArrayMeanRobotRobotDistance[m_unNumberRobots * m_unExperimentLength] = {0};
    m_fArrayMeanRobotRobotMinDistance[m_unNumberRobots * m_unExperimentLength] = {0};
}

/****************************************/
/****************************************/


SDBCLoopFunc::~SDBCLoopFunc() {}

/****************************************/
/****************************************/

void SDBCLoopFunc::Destroy() {}

/****************************************/
/****************************************/

argos::CColor SDBCLoopFunc::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  CVector2 cSpot1(0.0f, 0.0f);
  Real d = (cSpot1 - vCurrentPoint).Length();
  if (d <= 0.03) {
    return CColor::BLACK;
  }

  return CColor::GRAY50;
}


/****************************************/
/****************************************/

void SDBCLoopFunc::Reset() {
  m_fObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void SDBCLoopFunc::PostStep() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CSpace::TMapPerType& tWallMap = GetSpace().GetEntitiesByType("box");
  CSpace::TMapPerType& tLightMap = GetSpace().GetEntitiesByType("light");

  Real fRobotRobotDistance[tEpuckMap.size() * tEpuckMap.size()] = {0};
  Real fRobotWallsDistance[tEpuckMap.size() * tWallMap.size()] = {0};
  Real fRobotLightDistance[tEpuckMap.size() * tLightMap.size()] = {0};

  UInt32 unRobotRobotDistanceCounter = 0;
  UInt32 unRobotWallsDistanceCounter = 0;
  UInt32 unRobotLightDistanceCounter = 0;

  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    LOG << "Robot " << pcEpuck->GetId() << std::endl;
    LOG << ComputeLinearVelocity(pcEpuck) << std::endl;
    LOG << ComputeAngularVelocity(pcEpuck) << std::endl;
    for (CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
      CLightEntity* pcLight = any_cast<CLightEntity*>(it->second);
      //LOG << "Distance " << pcEpuck->GetId() << " to " << pcLight->GetId() << " = " << DistanceRobotLight(pcEpuck, pcLight) << std::endl;
    }
    for (CSpace::TMapPerType::iterator it = tWallMap.begin(); it != tWallMap.end(); ++it) {
      CBoxEntity* pcWall = any_cast<CBoxEntity*>(it->second);
      //LOG << "Distance " << pcEpuck->GetId() << " to " << pcWall->GetId() << " = " << DistanceRobotWall(pcEpuck, pcWall) << std::endl;
    }
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
      CEPuckEntity* pcOtherEpuck = any_cast<CEPuckEntity*>(it->second);
      if (pcEpuck->GetId() != pcOtherEpuck->GetId()) {
        //LOG << "Distance " << pcEpuck->GetId() << " to " << pcOtherEpuck->GetId() << " = " << DistanceRobotRobot(pcEpuck, pcOtherEpuck) << std::endl;
      }
    }
  }
}

/****************************************/
/****************************************/

Real SDBCLoopFunc::DistanceRobotLight(CEPuckEntity* pc_robot, CLightEntity* pc_light) {
  return Distance(pc_robot->GetEmbodiedEntity().GetOriginAnchor().Position,  pc_light->GetPosition());
}

/****************************************/
/****************************************/

Real SDBCLoopFunc::DistanceRobotRobot(CEPuckEntity* pc_robot_1, CEPuckEntity* pc_robot_2) {
  return Distance(pc_robot_1->GetEmbodiedEntity().GetOriginAnchor().Position,  pc_robot_2->GetEmbodiedEntity().GetOriginAnchor().Position);
}

/****************************************/
/****************************************/

Real SDBCLoopFunc::DistanceRobotPoint(CEPuckEntity* pc_robot, CVector2 c_point) {
  CVector2 cRobotLocation(pc_robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                          pc_robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
  return Distance(cRobotLocation, c_point);
}

/****************************************/
/****************************************/

Real SDBCLoopFunc::DistanceRobotWall(CEPuckEntity* pc_robot, CBoxEntity* pc_wall) {
  CVector2 cRobotPosition = CVector2(pc_robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), pc_robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

  CVector2 cDelta = ComputeDeltaWall(pc_wall);
  CVector2 cMinCorner = CVector2(pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + cDelta.GetX(), pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetY() - cDelta.GetY());
  CVector2 cMaxCorner = CVector2(pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetX() - cDelta.GetX(),  pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + cDelta.GetY());

  // Following code taken from https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
  Real fLenghtSquare = SquareDistance(cMinCorner, cMaxCorner);
  if (fLenghtSquare == 0.0f) {
    LOG << "Distance = " << Distance(cMinCorner, cRobotPosition) << std::endl;
  }

  CVector2 cLineMinRobot = cRobotPosition - cMinCorner;
  CVector2 cLineMaxMin = cMaxCorner - cMinCorner;
  Real t = Max((Real) 0.0f, Min((Real) 1.0f, (cLineMinRobot.DotProduct(cLineMaxMin))/fLenghtSquare));

  CVector2 cProjectionOnWall = cMinCorner + t * cLineMaxMin;

  return Distance(cProjectionOnWall, cRobotPosition);
}

/****************************************/
/****************************************/

CVector2 SDBCLoopFunc::ComputeDeltaWall(CBoxEntity* pc_wall) {
  CVector2 cCenterWall(pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

  CRadians cZAngle, cYAngle, cXAngle;
  pc_wall->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
  Real fLength = pc_wall->GetSize().GetY();

  CVector2 cDelta(Sin(cZAngle) * fLength/2, Cos(cZAngle) * fLength/2);
  return cDelta;
}

/****************************************/
/****************************************/

Real SDBCLoopFunc::ComputeLinearVelocity(CEPuckEntity* pc_robot) {
  return (pc_robot->GetWheeledEntity().GetWheelVelocity(0) + pc_robot->GetWheeledEntity().GetWheelVelocity(1))/2;
}

/****************************************/
/****************************************/

Real SDBCLoopFunc::ComputeAngularVelocity(CEPuckEntity* pc_robot) {
  Real fInterwheelDistance = pc_robot->GetWheeledEntity().GetWheelPosition(0).GetY() * 2;
  return (pc_robot->GetWheeledEntity().GetWheelVelocity(1) - pc_robot->GetWheeledEntity().GetWheelVelocity(0))/ fInterwheelDistance;
}

/****************************************/
/****************************************/

void SDBCLoopFunc::PostExperiment() {

}

/****************************************/
/****************************************/

Real SDBCLoopFunc::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 SDBCLoopFunc::GetRandomPosition() {
  Real temp, a, b, fPosX, fPosY;
  bool bPlaced = false;
  do {
      a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
      b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
      // If b < a, swap them
      if (b < a) {
        temp = a;
        a = b;
        b = temp;
      }
      fPosX = b * m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a/b));
      fPosY = b * m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a/b));

      bPlaced = true;

  } while(!bPlaced);
  return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(SDBCLoopFunc, "sdbc_loop_func");
