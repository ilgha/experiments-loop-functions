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
  m_vecPossiblePatchesPositions.push_back(CVector2(0,-0.70));
  m_vecPossiblePatchesPositions.push_back(CVector2(0, 0.70));

  m_vecPossibleLightIntensities.push_back(0.0f);
  m_vecPossibleLightIntensities.push_back(5.0f);


  m_vecPossibleObstacles.push_back(new Box(argos::CVector2(0, 0.35), 90, 0.70, 0));   // Box = position center; orientation; length; index
  m_vecPossibleObstacles.push_back(new Box(argos::CVector2(0, -0.35), 90, 0.70, 0));   // Box = position center; orientation; length; index
  m_vecPossibleObstacles.push_back(new Box(argos::CVector2(0.35, 0), 0, 0.70, 0));   // Box = position center; orientation; length; index
  m_vecPossibleObstacles.push_back(new Box(argos::CVector2(-0.35, 0), 0, 0.70, 0));   // Box = position center; orientation; length; index

}

/****************************************/
/****************************************/

RepertoireTrainingLoopFunc::RepertoireTrainingLoopFunc(const RepertoireTrainingLoopFunc& orig) {}

/****************************************/
/****************************************/

void RepertoireTrainingLoopFunc::Init(TConfigurationNode& t_tree) {
  CoreLoopFunctions::Init(t_tree);

  TConfigurationNode cParametersNode;
  try {
    cParametersNode = GetNode(t_tree, "params");
    GetNodeAttributeOrDefault(cParametersNode, "samplingFeatures", m_boolSmaplingFeatures, false);
  } catch(std::exception e) {
    LOGERR << "Problem with Attributes in node params" << std::endl;
  }

  if (m_boolSmaplingFeatures) {
    UInt8 unNbRobotsToRemove = m_pcRng->Uniform(CRange<UInt32>(0, 10));
    for(UInt8 i = 1; i <= unNbRobotsToRemove; i++) {
      std::ostringstream id;
      id << "epuck" << i;
      RemoveEntity(id.str().c_str());
    }
  }

  m_unNumberFeatures = 7;   // Mean linear speed, angular speed, distance to walls/obstacles, distance to robots, distance to closest robot, ambient light perceived, ground color perceived.

  // Initialization scaling factors
  //m_vecFeatureScalingFactorMeans = {0.0, 0.0, 1.259605869, 1.084184883, 0.268730835, 0.00797736053, 0.497768835};
  //m_vecFeatureScalingFactorStandardDevs = {12.0, 4.5283, 0.02633527, 0.07769956, 0.0317541, 0.008598, 0.0441245};
  m_vecFeatureScalingFactorMeans = {0.0, 0.0, 0.9654774, 0.6546410150000002, 0.23583227699999973, 0.006369859000000004, 0.38180308800000307};
  m_vecFeatureScalingFactorStandardDevs = {12.0, 4.5283, 0.17962875427902958, 0.2532494908214206, 0.03819563474317805, 0.00720179463562769, 0.07595536343119043};

  // Floor patches
  UInt8 unIndexColor = m_pcRng->Uniform(CRange<UInt32>(0, m_vecPossiblePatchesColors.size()));
  m_cPatchColor = m_vecPossiblePatchesColors[unIndexColor];

  UInt8 unIndexPosition = m_pcRng->Uniform(CRange<UInt32>(0, m_vecPossiblePatchesPositions.size()));
  m_cPatchPosition = m_vecPossiblePatchesPositions[unIndexPosition];


  // Light
  CSpace::TMapPerType& tLightMap = GetSpace().GetEntitiesByType("light");
  UInt8 unIndexIntensity = m_pcRng->Uniform(CRange<UInt32>(0, m_vecPossibleLightIntensities.size()));
  for (CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
    CLightEntity* pcLight = any_cast<CLightEntity*>(it->second);
    pcLight->SetIntensity(m_vecPossibleLightIntensities[unIndexIntensity]);
  }


  // Obstacles
  UInt8 unNumberObstacles = m_pcRng->Uniform(CRange<UInt32>(0,2));      // Max 1 obstacle for now on...
  if (unNumberObstacles > 0) {
    UInt8 unIndexObstacle = m_pcRng->Uniform(CRange<UInt32>(0, m_vecPossibleObstacles.size()));
    AddObstacle(m_vecPossibleObstacles.at(unIndexObstacle));
  }
}

/****************************************/
/****************************************/

void RepertoireTrainingLoopFunc::AddObstacle(Box* pc_obstacle) {
  CBoxEntity* pcBox;
  pcBox = new CBoxEntity("ostacle_" + ToString(pc_obstacle->GetIndex()),                                                  // Name
                          CVector3(pc_obstacle->GetPosition().GetX(), pc_obstacle->GetPosition().GetY(), 0),              // Position
                          CQuaternion().FromEulerAngles(argos::ToRadians(argos::CDegrees(pc_obstacle->GetOrientation())), // Orientation
                                                        CRadians::ZERO,
                                                        CRadians::ZERO),
                          false,                                                                                        // Movable
                          CVector3(pc_obstacle->GetWidth(), pc_obstacle->GetLength(), pc_obstacle->GetHeight()));       // Size

  AddEntity(*pcBox);
}

/****************************************/
/****************************************/

void RepertoireTrainingLoopFunc::PostStep() {
  m_unExperimentLength += 1;

  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CSpace::TMapPerType& tWallMap = GetSpace().GetEntitiesByType("box");

  std::vector<Real> vecCurrentSDBC{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  // For each robot
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);

    // Compute number collisions
    if (pcEpuck->GetEmbodiedEntity().IsCollidingWithSomething()) {
      m_unNumberCollisions++;
    }

    // Compute linear velocity
    vecCurrentSDBC.at(0) += ComputeLinearVelocity(pcEpuck);

    // Compute angular velocity
    vecCurrentSDBC.at(1) += ComputeAngularVelocity(pcEpuck);

    // Compute mean distance to walls or obstacles
    Real fDistanceToWalls = 0;
    UInt32 unNumberWalls = 0;
    for (CSpace::TMapPerType::iterator it = tWallMap.begin(); it != tWallMap.end(); ++it) {
      CBoxEntity* pcWall = any_cast<CBoxEntity*>(it->second);
      fDistanceToWalls += DistanceRobotWall(pcEpuck, pcWall);
      /* std::cout << DistanceRobotWall(pcEpuck, pcWall) << std::endl; */
      if ( DistanceRobotWall(pcEpuck, pcWall) < 0.095 ) {
          m_unNumberCollisions++;
      }
      unNumberWalls += 1;
    }
    vecCurrentSDBC.at(2) += fDistanceToWalls / unNumberWalls;

    // Compute distance to every other robot and min distance
    Real fMinDistance = 100000.0f;
    Real fCurrentDistanceRobot = 0.0f;
    Real fTotalDistanceRobot = 0.0f;
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
      CEPuckEntity* pcOtherEpuck = any_cast<CEPuckEntity*>(it->second);
      if (pcEpuck->GetId() != pcOtherEpuck->GetId()) {
        fCurrentDistanceRobot = DistanceRobotRobot(pcEpuck, pcOtherEpuck);

        /* std::cout << "rob: " << DistanceRobotRobot(pcEpuck, pcOtherEpuck)<< std::endl; */
        if ( DistanceRobotRobot(pcEpuck, pcOtherEpuck) < 0.095 ) {
          m_unNumberCollisions++;
        }

        if (fCurrentDistanceRobot < fMinDistance) {
          fMinDistance = fCurrentDistanceRobot;
        }
        fTotalDistanceRobot += fCurrentDistanceRobot;
      }
    }
    vecCurrentSDBC.at(3) += fTotalDistanceRobot / (m_unNumberRobots - 1);
    vecCurrentSDBC.at(4) += fMinDistance;


    // Ambient light perceived
    Real fAmbientLightPerceived = 0.0f;
    CCI_EPuckLightSensor* pcLightSensor = pcEpuck->GetControllableEntity().GetController().GetSensor<CCI_EPuckLightSensor>("epuck_light");
    const CCI_EPuckLightSensor::TReadings& sLightReadings = pcLightSensor->GetReadings();
    for (UInt8 i = 0; i < sLightReadings.size(); i++) {
      fAmbientLightPerceived = sLightReadings[i].Value;
    }
    vecCurrentSDBC.at(5) += fAmbientLightPerceived / sLightReadings.size();

    // Color ground perceived
    Real fGroundColorPerceived = 0.0f;
    CCI_EPuckGroundSensor* pcGroundSensor = pcEpuck->GetControllableEntity().GetController().GetSensor<CCI_EPuckGroundSensor>("epuck_ground");
    const CCI_EPuckGroundSensor::SReadings& sGroundReadings = pcGroundSensor->GetReadings();
    fGroundColorPerceived += sGroundReadings.Left;
    fGroundColorPerceived += sGroundReadings.Center;
    fGroundColorPerceived += sGroundReadings.Right;
    vecCurrentSDBC.at(6) += fGroundColorPerceived / 3;
  }

  for (UInt8 i = 0; i < vecCurrentSDBC.size(); i++) {
    vecCurrentSDBC.at(i) /= m_unNumberRobots;
  }

  /*
  for (UInt8 i = 0; i < vecCurrentSDBC.size(); i++) {
    LOG << vecCurrentSDBC.at(i) << " ";
  }
  LOG << std::endl;  */

  m_vecAllSDBC.push_back(vecCurrentSDBC);
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
  m_vecAllSDBC.clear();
  m_vecFinalSDBC.clear();

  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void RepertoireTrainingLoopFunc::PostExperiment() {
  m_fQualityMetric = 1 - m_unNumberCollisions / (Real)(m_unExperimentLength * m_unNumberRobots);

  if (m_boolSmaplingFeatures) {
    for (UInt8 i = 0; i < m_unNumberFeatures; i++) { // For all features
      m_vecFinalSDBC.push_back(ComputeMeanValueFeature(i));
    }
  } else {
    for (UInt8 i = 0; i < m_unNumberFeatures; i++) { // For all features
      // LOG << ComputeMeanValueFeature(i) << " " <<  m_vecFeatureScalingFactorMeans.at(i) << " " << m_vecFeatureScalingFactorStandardDevs.at(i) << " " << (ComputeMeanValueFeature(i) - m_vecFeatureScalingFactorMeans.at(i)) / m_vecFeatureScalingFactorStandardDevs.at(i) << std::endl;
      Real fScaledMean = Max(-m_fBound, Min(m_fBound, (ComputeMeanValueFeature(i) - m_vecFeatureScalingFactorMeans.at(i)) / m_vecFeatureScalingFactorStandardDevs.at(i)));
      m_vecFinalSDBC.push_back(fScaledMean);
      Real fScaledSD = Max(-m_fBound, Min(m_fBound, (ComputeStandardDeviationValueFeature(i) / m_vecFeatureScalingFactorStandardDevs.at(i) * 2 - 1)));
      m_vecFinalSDBC.push_back(fScaledSD);
    }
  }



  LOG << "SDBC ";
  for (UInt32 i = 0; i < m_vecFinalSDBC.size(); i++) {
     LOG << m_vecFinalSDBC[i] << " ";
  }
  LOG << std::endl;

  LOG << "Fitness " << m_fQualityMetric <<  std::endl; //" (" << m_unNumberCollisions << ")" << std::endl;

}

/****************************************/
/****************************************/

Real RepertoireTrainingLoopFunc::GetObjectiveFunction() {
  return m_fQualityMetric;
}

/****************************************/
/****************************************/

std::vector<Real> RepertoireTrainingLoopFunc::GetSDBC() {
  return m_vecFinalSDBC;
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

Real RepertoireTrainingLoopFunc::ComputeMeanValueFeature(UInt8 un_index_feature) {
  if (m_vecAllSDBC.size() > 0) {
    Real fSumFeatures = 0.0f;
    for (UInt32 i = 0; i < m_vecAllSDBC.size(); i++) {
       fSumFeatures += m_vecAllSDBC[i][un_index_feature];
    }
    return fSumFeatures / m_vecAllSDBC.size();
  } else {
    THROW_ARGOSEXCEPTION("Empty vector!");
  }
}

/****************************************/
/****************************************/

Real RepertoireTrainingLoopFunc::ComputeStandardDeviationValueFeature(UInt8 un_index_feature) {
  Real fStandardDeviation = 0.0f;
  if (m_vecAllSDBC.size() > 0) {
    Real fSumFeatures = 0.0f;
    for (UInt32 i = 0; i < m_vecAllSDBC.size(); i++) {
       fSumFeatures += m_vecAllSDBC[i][un_index_feature];
    }
    Real fMeanFeature = fSumFeatures / m_vecAllSDBC.size();
    for (size_t i = 0; i < m_vecAllSDBC.size(); i++) {
      fStandardDeviation += std::pow(m_vecAllSDBC[i][un_index_feature] - fMeanFeature, 2);
    }
    return std::sqrt(fStandardDeviation / m_vecAllSDBC.size()) ;
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
