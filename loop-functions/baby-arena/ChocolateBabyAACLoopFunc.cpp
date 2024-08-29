/**
  * @file <loop-functions/example/ChocolateAACLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "ChocolateBabyAACLoopFunc.h"

/****************************************/
/****************************************/

ChocolateBabyAACLoopFunction::ChocolateBabyAACLoopFunction() {
  m_fRadius = 0.09;
  m_cCoordBlackSpot = CVector2(0,0.16);
  m_cCoordWhiteSpot = CVector2(0,-0.16);
  m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

ChocolateBabyAACLoopFunction::ChocolateBabyAACLoopFunction(const ChocolateBabyAACLoopFunction& orig) {}

/****************************************/
/****************************************/

ChocolateBabyAACLoopFunction::~ChocolateBabyAACLoopFunction() {}

/****************************************/
/****************************************/

void ChocolateBabyAACLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void ChocolateBabyAACLoopFunction::Reset() {
  m_fObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

argos::CColor ChocolateBabyAACLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordBlackSpot - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  d = (m_cCoordWhiteSpot - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::WHITE;
  }


  return CColor::GRAY50;
}

/****************************************/
/****************************************/

void ChocolateBabyAACLoopFunction::PostStep() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    Real fDistanceSpot = (m_cCoordBlackSpot - cEpuckPosition).Length();
    if (fDistanceSpot <= m_fRadius) {
      m_fObjectiveFunction += 1;
    }
  }
}

/****************************************/
/****************************************/

Real ChocolateBabyAACLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 ChocolateBabyAACLoopFunction::GetRandomPosition() {
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

/****************************************/
/****************************************/

void ChocolateBabyAACLoopFunction::PostExperiment()
{
    LOG << m_fObjectiveFunction << std::endl;
}

REGISTER_LOOP_FUNCTIONS(ChocolateBabyAACLoopFunction, "chocolate_baby_aac_loop_functions");
