/*
 * @file <src/CoreLoopFunctions.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 * @author Ken Hasselmann - <khasselm@ulb.ac.be>
 *
 * @package experiments-loop-functions
 *
 * @license MIT License
 */


#include "CoreLoopFunctions.h"

/****************************************/
/****************************************/

void CoreLoopFunctions::Init(argos::TConfigurationNode& t_tree) {
  m_pcRng = CRandom::CreateRNG("argos");
  TConfigurationNode cParametersNode;
  try {
    cParametersNode = GetNode(t_tree, "params");
    GetNodeAttributeOrDefault(cParametersNode, "number_robots", m_unNumberRobots, (UInt32) 1);
    GetNodeAttributeOrDefault(cParametersNode, "dist_radius", m_fDistributionRadius, (Real) 0);
    GetNodeAttributeOrDefault(cParametersNode, "is_robots_black", m_bBlack, false);
    GetNodeAttributeOrDefault(cParametersNode, "is_random", m_bIsRandom, true);
    //GetNodeAttributeOrDefault(cParametersNode, "black_patch", m_bBlack, false);
    GetNodeAttributeOrDefault(cParametersNode, "is_random_and_black_activated", m_bRandomAndBlackActivated, false);
  } catch(std::exception e) {
    LOGERR << e.what() << std::endl;
  }

  m_cChooseColor = CColor::BLACK;
  fRandomIndex = 0;

  if(m_bRandomAndBlackActivated == true) {
        SelectColorOrder();
  }
  MoveRobots();

}

/****************************************/
/****************************************/

void CoreLoopFunctions::Reset() {
  if(m_bRandomAndBlackActivated == true) {
        SelectColorOrder();
  }
  MoveRobots();
}

/****************************************/
/****************************************/

CoreLoopFunctions::~CoreLoopFunctions() {}

/****************************************/
/****************************************/

//This function is decrepated: the robots should be instanciated in the .argos file
void CoreLoopFunctions::PositionRobots() {
  CEPuckEntity* pcEpuck;
  bool bPlaced = false;
  UInt32 unTrials;
  for(UInt32 i = 1; i < m_unNumberRobots + 1; ++i) {
    std::ostringstream id;
    id << "epuck" << i;
    pcEpuck = new CEPuckEntity(id.str().c_str(),
                               "decrepated",
                               CVector3(0,0,0),
                               CQuaternion().FromEulerAngles(CRadians::ZERO,CRadians::ZERO,CRadians::ZERO));
    AddEntity(*pcEpuck);
    // Choose position at random
    unTrials = 0;
    do {
       ++unTrials;
       CVector3 cEpuckPosition = GetRandomPosition();
       bPlaced = MoveEntity((*pcEpuck).GetEmbodiedEntity(),
                            cEpuckPosition,
                            CQuaternion().FromEulerAngles(m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO,CRadians::TWO_PI)),
                            CRadians::ZERO,CRadians::ZERO),false);
    } while(!bPlaced && unTrials < 100);
    if(!bPlaced) {
       THROW_ARGOSEXCEPTION("Can't place robot");
    }
  }
}

/****************************************/
/****************************************/

void CoreLoopFunctions::MoveRobots() {
  CEPuckEntity* pcEpuck;
  bool bPlaced = false;
  UInt32 unTrials;
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    pcEpuck = any_cast<CEPuckEntity*>(it->second);
    // Choose position at random
    unTrials = 0;
    do {
       ++unTrials;
       CVector3 cEpuckPosition = GetRandomPosition();
       bPlaced = MoveEntity(pcEpuck->GetEmbodiedEntity(),
                            cEpuckPosition,
                            CQuaternion().FromEulerAngles(m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO,CRadians::TWO_PI)),
                            CRadians::ZERO,CRadians::ZERO),false);
    } while(!bPlaced && unTrials < 100);
    if(!bPlaced) {
       THROW_ARGOSEXCEPTION("Can't place robot");
    }
  }
}

/****************************************/
/****************************************/

void CoreLoopFunctions::RemoveRobots() {
  for(UInt32 i = 1; i < m_unNumberRobots + 1; ++i) {
    std::ostringstream id;
    id << "epuck" << i;
    RemoveEntity(id.str().c_str());
  }
}

/****************************************/
/****************************************/

void CoreLoopFunctions::SelectColorOrder() {

    if (m_bIsRandom == true) {
        fRandomIndex = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    }
    else {
        if(m_bBlack == true) {
        fRandomIndex = 0.0f;
        }
        else {
        fRandomIndex = 1.0f;
        }
    }


    if (fRandomIndex > 0.5) {
        m_cChooseColor = CColor::WHITE;
    }
    else {
        m_cChooseColor = CColor::BLACK;
    }

}
