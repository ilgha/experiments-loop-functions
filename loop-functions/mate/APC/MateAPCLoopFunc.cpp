/**
 * Any-Point Closseness loop function
 *
 * @file <loop-functions/mate/APC/MateAPCLoopFunc.cpp>
 *
 * @author Fernando Mendiburu - <fmendibu@ulb.ac.be>
 *
 * @package experiments-loop-functions
 *
 * @license MIT License
 */

#include "MateAPCLoopFunc.h"

/****************************************/
/****************************************/

MateAPCLoopFunction::MateAPCLoopFunction() {

  m_fSideSquare = 1.0;

  m_cCoordSquareSpot = CVector2(0.6, 0);

  m_unNumberPoints = 1000;

  m_fObjectiveFunction = 0;

}

/****************************************/
/****************************************/

MateAPCLoopFunction::MateAPCLoopFunction(const MateAPCLoopFunction& orig) {}

/****************************************/
/****************************************/

MateAPCLoopFunction::~MateAPCLoopFunction() {}

/****************************************/
/****************************************/

void MateAPCLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void MateAPCLoopFunction::Reset() {
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

argos::CColor MateAPCLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 cCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  if (IsOnSquareArea(cCurrentPoint)){
      return CColor::BLACK;
  } else{
      return CColor::GRAY50;
  }
}

/****************************************/
/****************************************/

void MateAPCLoopFunction::PostExperiment() {
  m_fObjectiveFunction = ComputeObjectiveFunction();
  LOG << "Score: " << GetObjectiveFunction() << std::endl;
}



/****************************************/
/****************************************/

Real MateAPCLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

Real MateAPCLoopFunction::ComputeObjectiveFunction() {
    CVector2 cRandomPoint;
    Real dA=0, dP=0;
    CSpace::TMapPerType mEpucks = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    Real fDistanceToRandomPoint = 0;

    // White square area
    for(UInt32 i = 0; i < m_unNumberPoints; i++){

        Real fMinDistanceOnSquare = 0.67;  // Correspond to worst case, only one robot in the corner of the square

        cRandomPoint = RandomPointOnSquareArea();

        for (CSpace::TMapPerType::iterator it = mEpucks.begin(); it != mEpucks.end(); ++it) {
            CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*> ((*it).second);
            cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                               pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
            if(IsOnSquareArea(cEpuckPosition)){
                fDistanceToRandomPoint = (cRandomPoint - cEpuckPosition).Length();
                if(fDistanceToRandomPoint < fMinDistanceOnSquare){
                    fMinDistanceOnSquare = fDistanceToRandomPoint;
                }
            }
        }

        dA += fMinDistanceOnSquare;
    }
    dA /= m_unNumberPoints;




    Real performance = 100*100*dA*dA;   // in cm2

    return performance;
}

/****************************************/
/****************************************/

CVector2 MateAPCLoopFunction::RandomPointOnSquareArea(){
    return CVector2(m_pcRng->Uniform(CRange<Real>(m_cCoordSquareSpot.GetX() - m_fSideSquare/2.0f, m_cCoordSquareSpot.GetX() + m_fSideSquare/2.0f)),
                    m_pcRng->Uniform(CRange<Real>(m_cCoordSquareSpot.GetY() - m_fSideSquare/2.0f, m_cCoordSquareSpot.GetY() + m_fSideSquare/2.0f)));
}

/****************************************/
/****************************************/

bool MateAPCLoopFunction::IsOnSquareArea(const CVector2& c_point){
    CRange<Real> cRangeSquareX(m_cCoordSquareSpot.GetX() - m_fSideSquare/2.0f, m_cCoordSquareSpot.GetX() + m_fSideSquare/2.0f);
    CRange<Real> cRangeSquareY(m_cCoordSquareSpot.GetY() - m_fSideSquare/2.0f, m_cCoordSquareSpot.GetY() + m_fSideSquare/2.0f);

    if (cRangeSquareX.WithinMinBoundIncludedMaxBoundIncluded(c_point.GetX()) &&
            cRangeSquareY.WithinMinBoundIncludedMaxBoundIncluded(c_point.GetY())) {
        return true;
    }
    return false;
}


/****************************************/
/****************************************/

CVector3 MateAPCLoopFunction::GetRandomPosition() {

    CVector3 cPosition;


    do {
    cPosition = CVector3(m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,-0.6)),
                         m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,m_fDistributionRadius)),
                         0);
    } while(cPosition.Length()>=m_fDistributionRadius);


    return cPosition;
}


REGISTER_LOOP_FUNCTIONS(MateAPCLoopFunction, "mate_apc_loop_functions");
