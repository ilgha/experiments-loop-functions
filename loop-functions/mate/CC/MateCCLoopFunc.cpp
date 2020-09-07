/**
 * Conditional Coverage loop function
 *
 * @file <loop-functions/mate/CC/MateCCLoopFunc.cpp>
 *
 * @author Fernando Mendiburu - <fmendibu@ulb.ac.be>
 *
 * @package experiments-loop-functions
 *
 * @license MIT License
 */

#include "MateCCLoopFunc.h"

/****************************************/
/****************************************/

MateCCLoopFunction::MateCCLoopFunction() {

    m_fSensingRange = 0.15;
    m_fCommunicationDistance = 0.15;

  m_fSideSquare = 1.0;

  m_cCoordSquareSpot = CVector2(0.6, 0);

  m_unNumberPoints = 10000;

  m_fObjectiveFunction = 0;

  m_fAreaLimit = -0.6;
}

/****************************************/
/****************************************/

MateCCLoopFunction::MateCCLoopFunction(const MateCCLoopFunction& orig) {}

/****************************************/
/****************************************/

MateCCLoopFunction::~MateCCLoopFunction() {}

/****************************************/
/****************************************/

void MateCCLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void MateCCLoopFunction::Reset() {
    m_fObjectiveFunction = 0;
  CoreLoopFunctions::Reset();

}

/****************************************/
/****************************************/

void MateCCLoopFunction::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);

}

/****************************************/
/****************************************/

argos::CColor MateCCLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 cCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());


  if (IsOnArea(cCurrentPoint)){
      return CColor::BLACK;
  }

  if (IsOnArea2(cCurrentPoint)){
      return CColor::WHITE;
  }


  return CColor::GRAY50;


}

/****************************************/
/****************************************/

void MateCCLoopFunction::PostStep() {

    m_fObjectiveFunction += ComputeObjectiveFunction();

}

void MateCCLoopFunction::PostExperiment() {
    //m_fObjectiveFunction = ComputeObjectiveFunction();
    LOG << "Score: " << GetObjectiveFunction() << std::endl;
}

/****************************************/
/****************************************/

Real MateCCLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

Real MateCCLoopFunction::ComputeObjectiveFunction() {


    Real performance = 0;
    Real performance2 = 0;
    Real fvalue = 0;

    /* Push all agents in a vector */
    std::vector<CAgent> agents = PickAgents();

    /* Compute the coverage-ratio of the biggest group in the spot*/

    performance  = ComputeCoverageRatioOnArea(agents);
    performance2 = ComputeCoverageRatioOnArea2(agents);

    if(m_cChooseColor == CColor::BLACK) {
        fvalue = performance2;
    } else {
        fvalue = performance;
    }

    return fvalue;


}


/****************************************/
/****************************************/

Real MateCCLoopFunction::ComputeCoverageRatioOnArea(std::vector<CAgent>& agents) {


    std::vector<CVector2> vGroup;

    Real performance = 0;
    UInt32 unNumPoints = 0;


    for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){
       if (IsOnArea(ag->cPosition)) {
          vGroup.push_back(ag->cPosition);
       }
    }


    /* If there exists robots in the area */
    if(!vGroup.empty()) {


        /* Monte-Carlo sampling to estimate the ratio of the bounding box that is covered by the sensing range */
        Real avg = 0;
        for (size_t i = 0; i < m_unNumberPoints; i++) {

           CVector2 rndPoint = RandomPointOnArea();

           if(IsInsideArena(rndPoint)) {

               unNumPoints++;
               for (std::vector<CVector2>::iterator pos = vGroup.begin(); pos != vGroup.end(); ++pos) {
                  if ((rndPoint- *pos).Length() <= m_fSensingRange) {
                     avg++;
                     break;
                  }
               }
           }
        }

        /* Compute the coverage-ratio */
        performance= avg / unNumPoints;

    }

    return performance;
}


Real MateCCLoopFunction::ComputeCoverageRatioOnArea2(std::vector<CAgent>& agents) {


    std::vector<CVector2> vGroup;

    Real performance = 0;
    UInt32 unNumPoints = 0;


    for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){
       if (IsOnArea2(ag->cPosition)) {
          vGroup.push_back(ag->cPosition);
       }
    }


    /* If there exists robots in the area */
    if(!vGroup.empty()) {


        /* Monte-Carlo sampling to estimate the ratio of the bounding box that is covered by the sensing range */
        Real avg = 0;
        for (size_t i = 0; i < m_unNumberPoints; i++) {

           CVector2 rndPoint = RandomPointOnArea2();

           if(IsInsideArena(rndPoint)) {

               unNumPoints++;
               for (std::vector<CVector2>::iterator pos = vGroup.begin(); pos != vGroup.end(); ++pos) {
                  if ((rndPoint- *pos).Length() <= m_fSensingRange) {
                     avg++;
                     break;
                  }
               }
           }
        }

        /* Compute the coverage-ratio */
        performance= avg / unNumPoints;

    }

    return performance;
}

/****************************************/
/****************************************/

void MateCCLoopFunction::AddNeighs(std::vector<CAgent>& agents, std::vector<CAgent>::iterator ag) {
    for (std::vector<CAgent>::iterator neigh = agents.begin(); neigh != agents.end(); ++neigh) {
        if (neigh->unClusterID != 0)
         continue;
        if ( Distance(ag->cPosition, neigh->cPosition) < m_fCommunicationDistance ) {
            neigh->unClusterID = ag->unClusterID;
            AddNeighs(agents, neigh);
        }
    }
}

std::vector<MateCCLoopFunction::CAgent> MateCCLoopFunction::PickAgents() {

    std::vector<CAgent> agents;
    CSpace::TMapPerType m_tEPuckEntityMap = GetSpace().GetEntitiesByType("epuck");

    CVector2 cEpuckPosition;

    /* create a vector with the agents positions (using the objects CAgents) */
    for (CSpace::TMapPerType::iterator itEPuckEntity = m_tEPuckEntityMap.begin(); itEPuckEntity != m_tEPuckEntityMap.end(); itEPuckEntity++) {
       CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*> (itEPuckEntity->second);
       cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                          pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

       agents.push_back(CAgent(cEpuckPosition,atoi(pcEpuck->GetId().substr(5, 6).c_str())));
    }

    return agents;
}

/****************************************/
/****************************************/

UInt32 MateCCLoopFunction::DetermineBiggestGroup(std::vector<CAgent>& agents) {

    /* Cluster the agents in groups */
    UInt32 maxUsedID = 0;
    for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){
       if (ag->unClusterID != 0)
          continue;
       ag->unClusterID = ++maxUsedID;
       AddNeighs(agents, ag);
    }

    /* Determine the biggest group */
    size_t maxGroupSize = 0;
    UInt32 biggestGroupID = 0;
    for (UInt32 i = 1; i <= maxUsedID; i++){
       size_t size = 0;
       for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){

          if (ag->unClusterID == i) {
             size++;
          }
       }

       if (maxGroupSize < size){
          maxGroupSize = size;
          biggestGroupID = i;
       }
    }

    return biggestGroupID;
}

/****************************************/
/****************************************/

CVector2 MateCCLoopFunction::RandomPointOnArea(){
    return CVector2(m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius, m_fDistributionRadius)),
                    m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius, m_fAreaLimit)));
}

CVector2 MateCCLoopFunction::RandomPointOnArea2(){
    return CVector2(m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius, m_fDistributionRadius)),
                    m_pcRng->Uniform(CRange<Real>(-m_fAreaLimit, m_fDistributionRadius)));
}

/****************************************/
/****************************************/

bool MateCCLoopFunction::IsOnArea(const CVector2& c_position) {

  if(c_position.GetY() < m_fAreaLimit) {
      return true;
  }

  return false;
}

bool MateCCLoopFunction::IsOnArea2(const CVector2& c_position) {

  if(c_position.GetY() >= -m_fAreaLimit) {
      return true;
  }

  return false;
}

/****************************************/
/****************************************/

bool MateCCLoopFunction::IsInsideArena(const CVector2& c_position) {

  if(c_position.Length() < m_fDistributionRadius) {
      return true;
  }

  return false;
}

/****************************************/
/****************************************/

CVector3 MateCCLoopFunction::GetRandomPosition() {

    CVector3 cPosition;

    if(m_cChooseColor == CColor::WHITE) {

        do {
        cPosition = CVector3(m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,m_fDistributionRadius)),
                             m_pcRng->Uniform(CRange<Real>(0.6,m_fDistributionRadius)),
                             0);
        } while(cPosition.Length()>=m_fDistributionRadius);
    } else {

        do {
        cPosition = CVector3(m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,m_fDistributionRadius)),
                             m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,-0.6)),
                             0);
        } while(cPosition.Length()>=m_fDistributionRadius);
    }

    return cPosition;
}


REGISTER_LOOP_FUNCTIONS(MateCCLoopFunction, "mate_cc_loop_functions");
