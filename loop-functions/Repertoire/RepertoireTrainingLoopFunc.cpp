/**
  * @file <loop-functions/AggregationTwoSpotsLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "RepertoireTrainingLoopFunc.h"

/****************************************/
/****************************************/

RepertoireTrainingLoopFunc::RepertoireTrainingLoopFunc() {
  m_fQualityMetric = 0.0f;
  m_unExperimentLength = 0;
  m_unNumberCollisions = 0;

  m_fPatchRadius = 0.35;

  m_vecPossiblePatchesColors.push_back(CColor::WHITE);
  m_vecPossiblePatchesColors.push_back(CColor::GRAY50);
  m_vecPossiblePatchesColors.push_back(CColor::BLACK);

  m_vecPossiblePatchesPositions.push_back(CVector2(0,0));
  m_vecPossiblePatchesPositions.push_back(CVector2(0,-0.65));
  m_vecPossiblePatchesPositions.push_back(CVector2(0, 0.65));

  m_vecPossibleLightIntensities.push_back(0.0f);
  m_vecPossibleLightIntensities.push_back(5.0f);

}

/****************************************/
/****************************************/

RepertoireTrainingLoopFunc::RepertoireTrainingLoopFunc(const RepertoireTrainingLoopFunc& orig) {}

/****************************************/
/****************************************/

void RepertoireTrainingLoopFunc::Init(TConfigurationNode& t_tree) {
  CoreLoopFunctions::Init(t_tree);

  UInt8 unIndexColor = m_pcRng->Uniform(CRange<UInt32>(0, m_vecPossiblePatchesColors.size()));
  m_cPatchColor = m_vecPossiblePatchesColors[unIndexColor];

  UInt8 unIndexPosition = m_pcRng->Uniform(CRange<UInt32>(0, m_vecPossiblePatchesPositions.size()));
  m_cPatchPosition = m_vecPossiblePatchesPositions[unIndexPosition];

  CSpace::TMapPerType& tLightMap = GetSpace().GetEntitiesByType("light");
  UInt8 unIndexIntensity = m_pcRng->Uniform(CRange<UInt32>(0, m_vecPossibleLightIntensities.size()));
  for (CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
    CLightEntity* pcLight = any_cast<CLightEntity*>(it->second);
    pcLight->SetIntensity(m_vecPossibleLightIntensities[unIndexIntensity]);
  }
}

/****************************************/
/****************************************/

void RepertoireTrainingLoopFunc::PostStep() {
  m_unExperimentLength += 1;

  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CSpace::TMapPerType& tWallMap = GetSpace().GetEntitiesByType("box");

  // For each robot
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);

    // Compute number collisions
    if (pcEpuck->GetEmbodiedEntity().IsCollidingWithSomething()) {
      m_unNumberCollisions++;
    }

    // Compute linear velocity
    m_vecRobotLinearVelocity.push_back(ComputeLinearVelocity(pcEpuck));

    // Compute angular velocity
    m_vecRobotAngularVelocity.push_back(ComputeAngularVelocity(pcEpuck));

    // Compute distance to walls
    for (CSpace::TMapPerType::iterator it = tWallMap.begin(); it != tWallMap.end(); ++it) {
      CBoxEntity* pcWall = any_cast<CBoxEntity*>(it->second);
      m_vecRobotWallsDistances.push_back(DistanceRobotWall(pcEpuck, pcWall));
    }

    // Compute distance to every other robot and min distance
    Real fMinDistance = 100000.0f;
    Real fCurrentDistance = 0.0f;
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
      CEPuckEntity* pcOtherEpuck = any_cast<CEPuckEntity*>(it->second);
      if (pcEpuck->GetId() != pcOtherEpuck->GetId()) {
        fCurrentDistance = DistanceRobotRobot(pcEpuck, pcOtherEpuck);
        if (fCurrentDistance < fMinDistance) {
          fMinDistance = fCurrentDistance;
        }
        m_vecRobotRobotDistances.push_back(fCurrentDistance);
      }
    }
    m_vecRobotRobotMinDistances.push_back(fMinDistance);


    // Ambient light perceived
    CCI_EPuckLightSensor* pcLightSensor = pcEpuck->GetControllableEntity().GetController().GetSensor<CCI_EPuckLightSensor>("epuck_light");
    const CCI_EPuckLightSensor::TReadings& sLightReadings = pcLightSensor->GetReadings();
    for (UInt8 i = 0; i < sLightReadings.size(); i++) {
      m_vecLightValuesPerceived.push_back(sLightReadings[i].Value);
    }

    // Color ground perceived
    CCI_EPuckGroundSensor* pcGroundSensor = pcEpuck->GetControllableEntity().GetController().GetSensor<CCI_EPuckGroundSensor>("epuck_ground");
    const CCI_EPuckGroundSensor::SReadings& sGroundReadings = pcGroundSensor->GetReadings();
    m_vecGroundValuesPerceived.push_back(sGroundReadings.Left);
    m_vecGroundValuesPerceived.push_back(sGroundReadings.Center);
    m_vecGroundValuesPerceived.push_back(sGroundReadings.Right);

  }

}

/****************************************/
/****************************************/

RepertoireTrainingLoopFunc::~RepertoireTrainingLoopFunc() {}

/****************************************/
/****************************************/

void RepertoireTrainingLoopFunc::Destroy() {}

/****************************************/
/****************************************/

argos::CColor RepertoireTrainingLoopFunc::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  Real fDistancePointCenter = (vCurrentPoint - m_cPatchPosition).Length();
  if (fDistancePointCenter < m_fPatchRadius) {
    return m_cPatchColor;
  }

  return CColor::GRAY50;
}

/****************************************/
/****************************************/

void RepertoireTrainingLoopFunc::Reset() {
  // Quality metric
  m_fQualityMetric = 0.0f;
  m_unExperimentLength = 0;
  m_unNumberCollisions = 0;

  // SDBC
  m_vecSDBC.clear();
  m_vecRobotLinearVelocity.clear();
  m_vecRobotAngularVelocity.clear();
  m_vecRobotRobotDistances.clear();
  m_vecRobotRobotMinDistances.clear();
  m_vecRobotWallsDistances.clear();
  m_vecLightValuesPerceived.clear();
  m_vecGroundValuesPerceived.clear();

  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void RepertoireTrainingLoopFunc::PostExperiment() {
  m_fQualityMetric = 1 - m_unNumberCollisions / (Real)(m_unExperimentLength * m_unNumberRobots);

  m_vecSDBC.push_back(ComputeMeanValue(m_vecRobotLinearVelocity));
  m_vecSDBC.push_back(ComputeStandardDeviationValue(m_vecRobotLinearVelocity));

  m_vecSDBC.push_back(ComputeMeanValue(m_vecRobotAngularVelocity));
  m_vecSDBC.push_back(ComputeStandardDeviationValue(m_vecRobotAngularVelocity));

  m_vecSDBC.push_back(ComputeMeanValue(m_vecRobotWallsDistances));
  m_vecSDBC.push_back(ComputeStandardDeviationValue(m_vecRobotWallsDistances));

  m_vecSDBC.push_back(ComputeMeanValue(m_vecRobotRobotDistances));
  m_vecSDBC.push_back(ComputeStandardDeviationValue(m_vecRobotRobotDistances));

  m_vecSDBC.push_back(ComputeMeanValue(m_vecRobotRobotMinDistances));
  m_vecSDBC.push_back(ComputeStandardDeviationValue(m_vecRobotRobotMinDistances));

  m_vecSDBC.push_back(ComputeMeanValue(m_vecLightValuesPerceived));
  m_vecSDBC.push_back(ComputeStandardDeviationValue(m_vecLightValuesPerceived));

  m_vecSDBC.push_back(ComputeMeanValue(m_vecGroundValuesPerceived));
  m_vecSDBC.push_back(ComputeStandardDeviationValue(m_vecGroundValuesPerceived));

  // Test
  // std::vector<Real> vecTest;
  // vecTest.push_back(1);
  // vecTest.push_back(2);
  // vecTest.push_back(3);
  // vecTest.push_back(4);
  // vecTest.push_back(5);
  // vecTest.push_back(6);
  // vecTest.push_back(7);
  // vecTest.push_back(8);
  // vecTest.push_back(9);
  // vecTest.push_back(10);
  // LOG << "Test " << ComputeMeanValue(vecTest) << " " << ComputeStandardDeviationValue(vecTest) << std::endl;

}

/****************************************/
/****************************************/

Real RepertoireTrainingLoopFunc::GetObjectiveFunction() {
  return m_fQualityMetric;
}

/****************************************/
/****************************************/

Real RepertoireTrainingLoopFunc::DistanceRobotRobot(CEPuckEntity* pc_robot_1, CEPuckEntity* pc_robot_2) {
  return Distance(pc_robot_1->GetEmbodiedEntity().GetOriginAnchor().Position,  pc_robot_2->GetEmbodiedEntity().GetOriginAnchor().Position);
}

/****************************************/
/****************************************/

Real RepertoireTrainingLoopFunc::DistanceRobotWall(CEPuckEntity* pc_robot, CBoxEntity* pc_wall) {
  CVector2 cRobotPosition = CVector2(pc_robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), pc_robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

  CVector2 cDelta = ComputeDeltaWall(pc_wall);
  CVector2 cMinCorner = CVector2(pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + cDelta.GetX(), pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetY() - cDelta.GetY());
  CVector2 cMaxCorner = CVector2(pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetX() - cDelta.GetX(),  pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + cDelta.GetY());

  // Following code taken from https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
  Real fLenghtSquare = SquareDistance(cMinCorner, cMaxCorner);

  CVector2 cLineMinRobot = cRobotPosition - cMinCorner;
  CVector2 cLineMaxMin = cMaxCorner - cMinCorner;
  Real t = Max((Real) 0.0f, Min((Real) 1.0f, (cLineMinRobot.DotProduct(cLineMaxMin))/fLenghtSquare));

  CVector2 cProjectionOnWall = cMinCorner + t * cLineMaxMin;

  return Distance(cProjectionOnWall, cRobotPosition);
}

/****************************************/
/****************************************/

CVector2 RepertoireTrainingLoopFunc::ComputeDeltaWall(CBoxEntity* pc_wall) {
  CVector2 cCenterWall(pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), pc_wall->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

  CRadians cZAngle, cYAngle, cXAngle;
  pc_wall->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
  Real fLength = pc_wall->GetSize().GetY();

  CVector2 cDelta(Sin(cZAngle) * fLength/2, Cos(cZAngle) * fLength/2);
  return cDelta;
}

/****************************************/
/****************************************/

Real RepertoireTrainingLoopFunc::ComputeLinearVelocity(CEPuckEntity* pc_robot) {
  return (pc_robot->GetWheeledEntity().GetWheelVelocity(0) + pc_robot->GetWheeledEntity().GetWheelVelocity(1))/2;
}

/****************************************/
/****************************************/

Real RepertoireTrainingLoopFunc::ComputeAngularVelocity(CEPuckEntity* pc_robot) {
  Real fInterwheelDistance = pc_robot->GetWheeledEntity().GetWheelPosition(0).GetY() * 2;
  return (pc_robot->GetWheeledEntity().GetWheelVelocity(1) - pc_robot->GetWheeledEntity().GetWheelVelocity(0))/ fInterwheelDistance;
}

/****************************************/
/****************************************/

Real RepertoireTrainingLoopFunc::ComputeMeanValue(std::vector<Real> vector_values) {
  if (vector_values.size() > 0) {
    return std::accumulate(vector_values.begin(), vector_values.end(), 0.0) / vector_values.size();
  } else {
    THROW_ARGOSEXCEPTION("Empty vector!");
  }
}

/****************************************/
/****************************************/

Real RepertoireTrainingLoopFunc::ComputeStandardDeviationValue(std::vector<Real> vector_values) {
  Real fStandardDeviation = 0.0f;
  if (vector_values.size() > 0) {
    Real fMean = std::accumulate(vector_values.begin(), vector_values.end(), 0.0) / vector_values.size();
    for (size_t i = 0; i < vector_values.size(); i++) {
      fStandardDeviation += std::pow(vector_values[i] - fMean, 2);
    }
    return std::sqrt(fStandardDeviation / vector_values.size()) ;
  } else {
    THROW_ARGOSEXCEPTION("Empty vector!");
  };
}

/****************************************/
/****************************************/

CVector3 RepertoireTrainingLoopFunc::GetRandomPosition() {
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

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(RepertoireTrainingLoopFunc, "rep_train_loop_func");
