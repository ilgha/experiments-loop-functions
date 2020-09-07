/**
 * Networked Coverage loop function
 *
 * @file <loop-functions/mate/NC/MateNCLoopFunc.h>
 *
 * @author Fernando Mendiburu - <fmendibu@ulb.ac.be>
 *
 * @package experiments-loop-functions
 *
 * @license MIT License
 */

#include "MateNCLoopFunc.h"

/****************************************/
/****************************************/

MateNCLoopFunction::MateNCLoopFunction() {
  m_fSensingRange = 0.15;
  m_fCommunicationDistance = 0.30;
  m_unNumberPoints = 10000;
  m_cArenaCenter = CVector2(0,0);
  m_fObjectiveFunction = 0;

  m_fAreaLimit = -0.6;


}

/****************************************/
/****************************************/

MateNCLoopFunction::MateNCLoopFunction(const MateNCLoopFunction& orig) {}

/****************************************/
/****************************************/

MateNCLoopFunction::~MateNCLoopFunction() {}

/****************************************/
/****************************************/

void MateNCLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void MateNCLoopFunction::Reset() {
  /* Reset variables */
  m_fObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

argos::CColor MateNCLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
    CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

    if (IsOnArea(vCurrentPoint)) {
      return CColor::BLACK;
    }

    return CColor::GRAY50;
}

/****************************************/
/****************************************/

void MateNCLoopFunction::PostStep() {
    m_fObjectiveFunction += ComputeObjectiveFunction();
    //LOG << "Score: " << ComputeObjectiveFunction() << std::endl;
}

void MateNCLoopFunction::PostExperiment() {

    LOG << "Score: " << GetObjectiveFunction() << std::endl;
}

/****************************************/
/****************************************/

bool MateNCLoopFunction::IsOnArea(CVector2& c_position) {

  if(c_position.GetY() < m_fAreaLimit) {
      return true;
  }

  return false;
}


bool MateNCLoopFunction::IsInsideArena(CVector2& c_position) {

  if(c_position.Length() < m_fDistributionRadius) {
      return true;
  }

  return false;
}

/****************************************/
/****************************************/

Real MateNCLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

void MateNCLoopFunction::AddNeighs(std::vector<CAgent> &agents, std::vector<CAgent>::iterator ag) {
    for (std::vector<CAgent>::iterator neigh = agents.begin(); neigh != agents.end(); ++neigh) {
        if (neigh->unClusterID != 0)
         continue;
        if ( Distance(ag->cPosition, neigh->cPosition) < m_fCommunicationDistance ) {
            neigh->unClusterID = ag->unClusterID;
            AddNeighs(agents, neigh);
        }
    }
}

/****************************************/
/****************************************/

std::vector<MateNCLoopFunction::CAgent> MateNCLoopFunction::PickAgentsInBlackSpot() {

    std::vector<CAgent> agents;
    CSpace::TMapPerType m_tEPuckEntityMap = GetSpace().GetEntitiesByType("epuck");

    CVector2 cEpuckPosition;

    /* create a vector with the agents positions (using the objects CAgents) */
    for (CSpace::TMapPerType::iterator itEPuckEntity = m_tEPuckEntityMap.begin(); itEPuckEntity != m_tEPuckEntityMap.end(); itEPuckEntity++) {
       CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*> (itEPuckEntity->second);
       cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                          pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

       if(IsOnArea(cEpuckPosition)) {
           agents.push_back(CAgent(cEpuckPosition,atoi(pcEpuck->GetId().substr(5, 6).c_str())));
       }
    }

    return agents;
}

/****************************************/
/****************************************/

UInt32 MateNCLoopFunction::DetermineBiggestGroup(std::vector<CAgent>& agents) {

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

Real MateNCLoopFunction::ComputeCoverageRatio(std::vector<CAgent>& agents, UInt32 GroupID) {


    std::vector<CVector2> vGroup;

    Real performance = 0;
    UInt32 unNumPoints = 0;


    for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){
       if ((ag->unClusterID == GroupID) && IsOnArea(ag->cPosition)) {
          vGroup.push_back(ag->cPosition);
       }
    }


    /* If there exists robots in the area contained in the biggest group */
    if(!vGroup.empty()) {


        /* Monte-Carlo sampling to estimate the ratio of the bounding box that is covered by the sensing range */
        Real avg = 0;
        for (size_t i = 0; i < m_unNumberPoints; i++) {

           CVector2 rndPoint = RandomPointOnNest();

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

Real MateNCLoopFunction::ComputeObjectiveFunction() {

  Real performance = 0;

  /* Push all agents in a vector */
  std::vector<CAgent> agents = PickAgentsInBlackSpot();

  /* Determine the biggest group ID*/
  UInt32 unBiggestGroupID = DetermineBiggestGroup(agents);

  /* Compute the coverage-ratio of the biggest group in the spot*/
  performance = ComputeCoverageRatio(agents,unBiggestGroupID);

  //PrintAgents(agents);

  return performance;
}

/****************************************/
/****************************************/

void MateNCLoopFunction::PrintAgents(std::vector<CAgent>& agents) {

    std::vector<CAgent>::iterator it;

    for (it = agents.begin(); it != agents.end(); it++) {

        std::cout << "IdRobot: " << it->unRobotID << " IdCluster: " << it->unClusterID << " pos: " << it->cPosition << "\n";

    }

     std::cout << "\n";
}

std::vector<MateNCLoopFunction::CAgent> MateNCLoopFunction::PickAgentsOfSameID(std::vector<CAgent>& agents, UInt32 un_GroupID) {

    std::vector<CAgent> agentes;

    for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){
        if(ag->unClusterID == un_GroupID) {
            agentes.push_back(*ag);
        }


    }

    return agentes;
}


CVector2 MateNCLoopFunction::GetRandomPoint() {

    CVector3 cPosition;

    cPosition.FromSphericalCoords(
                m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,-0.0)),     // distance from origin
                CRadians::PI_OVER_TWO,                                                             // angle with Z axis
                m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO, CRadians::PI))                                         // rotation around Z
                );

    return CVector2(cPosition.GetX(), cPosition.GetY());
}


CVector2 MateNCLoopFunction::RandomPointOnNest(){
    return CVector2(m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius, m_fDistributionRadius)),
                    m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius, m_fAreaLimit)));
}


CVector3 MateNCLoopFunction::GetRandomPosition() {

    CVector3 cPosition;


    do {
    cPosition = CVector3(m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,m_fDistributionRadius)),
                         m_pcRng->Uniform(CRange<Real>(0.6,m_fDistributionRadius)),
                         0);
    } while(cPosition.Length()>=m_fDistributionRadius);


    return cPosition;
}

REGISTER_LOOP_FUNCTIONS(MateNCLoopFunction, "mate_nc_loop_functions");
