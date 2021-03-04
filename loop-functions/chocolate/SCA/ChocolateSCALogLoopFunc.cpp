/**
  * @file <loop-functions/example/ChocolateSCALogLoopFunc.cpp>
  * Shelter woth constrained access and logging of the objective function
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "ChocolateSCALogLoopFunc.h"

/****************************************/
/****************************************/

ChocolateSCALogLoopFunction::ChocolateSCALogLoopFunction() {
  m_fSpotRadius = 0.3;
  m_cCoordBlackSpot = CVector2(0.8,-0.1);
  m_cCoordWhiteSpot = CVector2(-0.8,-0.1);
  m_fObjectiveFunction = 0;
  m_fWidthShelter = 0.6;
  m_fHeightShelter = 0.15;
  m_cPositionShelter = CVector2(0,0);
  m_unTimeStep = 0;
}

/****************************************/
/****************************************/

ChocolateSCALogLoopFunction::ChocolateSCALogLoopFunction(const ChocolateSCALogLoopFunction& orig) 
{
  m_unTimeStep = orig.m_unTimeStep;
}

/****************************************/
/****************************************/

ChocolateSCALogLoopFunction::~ChocolateSCALogLoopFunction() {}

/****************************************/

/****************************************/
void ChocolateSCALogLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void ChocolateSCALogLoopFunction::Reset() {
  m_fObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void ChocolateSCALogLoopFunction::Init(TConfigurationNode& t_tree) {
  CoreLoopFunctions::Init(t_tree);
  m_unTimeStep = 0;
  CQuaternion angleWall;

  // Center
  angleWall.FromEulerAngles(CRadians::PI_OVER_TWO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxCenter = new CBoxEntity("wall_center",
      CVector3(0, m_cPositionShelter.GetY() - (m_fHeightShelter/2)- (0.05/2), 0.0),
      angleWall,
      false,
      CVector3(0.05, m_fWidthShelter+0.05, 0.2));
  AddEntity(*m_pcBoxCenter);


  // Left
  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxLeft = new CBoxEntity("wall_left",
      CVector3(-m_fWidthShelter/2-0.05/4, m_cPositionShelter.GetY()-0.05/2, 0.0),
      angleWall,
      false,
      CVector3(0.05, m_fHeightShelter+0.05, 0.2));
  AddEntity(*m_pcBoxLeft);

  // Right
  m_pcBoxRight = new CBoxEntity("wall_right",
      CVector3(m_fWidthShelter/2+0.05/4, m_cPositionShelter.GetY()-0.05/2, 0.0),
      angleWall,
      false,
      CVector3(0.05, m_fHeightShelter+0.05, 0.2));
  AddEntity(*m_pcBoxRight);
}

/****************************************/
/****************************************/

argos::CColor ChocolateSCALogLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordBlackSpot - vCurrentPoint).Length();
  if (d <= m_fSpotRadius) {
    return CColor::BLACK;
  }

  d = (m_cCoordWhiteSpot - vCurrentPoint).Length();
  if (d <= m_fSpotRadius) {
    return CColor::BLACK;
  }

  if (IsInShelter(vCurrentPoint)) {
    return CColor::WHITE;
  }

  return CColor::GRAY50;
}

/****************************************/
/****************************************/

bool ChocolateSCALogLoopFunction::IsInShelter(CVector2& c_position) {
  Real fMaximalXCoord = m_fWidthShelter / 2;
  Real fMaximalYCoord = (m_fHeightShelter / 2) + m_cPositionShelter.GetY();
  if (c_position.GetX() > -fMaximalXCoord && c_position.GetX() < fMaximalXCoord) {
    if (c_position.GetY() > -fMaximalYCoord && c_position.GetY() < fMaximalYCoord) {
      return true;
    }
  }
  return false;
}

/****************************************/
/****************************************/

void ChocolateSCALogLoopFunction::PostStep() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    if (IsInShelter(cEpuckPosition)) {
      m_fObjectiveFunction += 1;
    }
  }
  std::cout << "--t " << m_unTimeStep << " --o " << m_fObjectiveFunction << std::endl;
  m_unTimeStep++;
}

/****************************************/
/****************************************/

Real ChocolateSCALogLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 ChocolateSCALogLoopFunction::GetRandomPosition() {
  Real temp;
  Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  Real  b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
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

REGISTER_LOOP_FUNCTIONS(ChocolateSCALogLoopFunction, "chocolate_sca_log_loop_functions");
