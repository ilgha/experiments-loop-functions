#include "ExplorationLoopFunc.h"


/****************************************/
// Version 2
/****************************************/

ExplorationLoopFunction::ExplorationLoopFunction() {
  m_unGranularity = 20;   // The 'granularity' of the grid.
  m_cRegex = std::regex("epuck_?([0-9]+)_?([0-9]+)?");
}

/****************************************/
/****************************************/

ExplorationLoopFunction::~ExplorationLoopFunction() {
  delete m_p3DGrid;
}


/****************************************/
/****************************************/

void ExplorationLoopFunction::Init(argos::TConfigurationNode& t_tree){
  CoreLoopFunctions::Init(t_tree);

  m_fScore = 0.0f;

  InitGrid();
  RegisterPositions();
}

/****************************************/
/****************************************/

void ExplorationLoopFunction::InitGrid(){
  m_cSizeArena.Set(GetSpace().GetArenaSize().GetX(), GetSpace().GetArenaSize().GetY());
  m_unCellsInRaws = (UInt32)(m_cSizeArena.GetX()*m_unGranularity);
  m_unCellsInColumns = (UInt32)(m_cSizeArena.GetY()*m_unGranularity);
  m_p3DGrid = new UInt16**[m_unNumberRobots];
  // m_unCellsInRaws = 10;
  // m_unCellsInColumns = 10;
  for(UInt16 i =0; i < m_unNumberRobots; i++){
    m_p3DGrid[i] = new UInt16*[m_unCellsInRaws];
    for(UInt32 j =0; j < m_unCellsInRaws; j++){
      m_p3DGrid[i][j] = new UInt16[m_unCellsInColumns];
      for(UInt32 k = 0; k < m_unCellsInColumns;k++){
        m_p3DGrid[i][j][k] = 0;
      }
    }
  }

  // for(UInt16 i =0; i < m_unNumberRobots; i++){
  //   std::cout << "Robot " << i << std::endl;
  //   for(int j =0; j < m_unCellsInRaws; j++){
  //     for(int k = 0; k < m_unCellsInColumns; k++){
  //       std::cout << m_p3DGrid[i][j][k] << " ";
  //     }
  //     std::cout << std::endl;
  //   }
  // }

}

/****************************************/
/****************************************/

ExplorationLoopFunction::ExplorationLoopFunction(const ExplorationLoopFunction& orig) {}

/****************************************/
/****************************************/

void ExplorationLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void ExplorationLoopFunction::Reset() {
  m_fScore = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void ExplorationLoopFunction::PostStep() {
  RegisterPositions();
}

/****************************************/
/****************************************/

Real ExplorationLoopFunction::ComputeStepObjectiveValue() {
  Real fScore = 0;
  UInt32 unIndividualScore = 0;
  for(UInt16 i = 0; i < m_unNumberRobots; i++){
    unIndividualScore = 0;
    for(UInt32 j = 0; j < m_unCellsInRaws; j++){
      for(UInt32 k = 0; k < m_unCellsInColumns; k++){
        if (m_p3DGrid[i][j][k] == 1) {
          fScore++;
          unIndividualScore++;
        }
      }
    }
  }

  return fScore;
}

/****************************************/
/****************************************/

void ExplorationLoopFunction::RegisterPositions() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  UInt32 unIdRobot = 0;
  CVector2 cEpuckPosition(0,0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                        pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    std::string strRobotId = pcEpuck->GetId();
    std::smatch cMatch;
    bool bMatchFound = std::regex_match(strRobotId, cMatch , m_cRegex);
    if (bMatchFound) {
      unIdRobot = std::stoi(cMatch[1].str());
    }

    UInt32 x = (unsigned int)((cEpuckPosition.GetX() + m_cSizeArena.GetX()/2.0) * m_unGranularity);
    UInt32 y = (unsigned int)((cEpuckPosition.GetY() + m_cSizeArena.GetY()/2.0) * m_unGranularity);
    m_p3DGrid[unIdRobot][x][y] = 1;
  }
}

/****************************************/
/****************************************/

void ExplorationLoopFunction::PostExperiment() {
  m_fScore = ComputeStepObjectiveValue();
  LOG << "Objective function result = " << m_fScore << std::endl;
}

/****************************************/
/****************************************/

Real ExplorationLoopFunction::GetObjectiveFunction() {
  return m_fScore;
}

/****************************************/
/****************************************/

CVector3 ExplorationLoopFunction::GetRandomPosition() {
  Real a;
  Real b;
  Real temp;

  a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  // If b < a, swap them
  if (b < a) {
    temp = a;
    a = b;
    b = temp;
  }
  Real fPosX = b * m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a/b));
  Real fPosY = b * m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a/b));
  return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(ExplorationLoopFunction, "exploration_loop_functions");
