/**
  * @file <loop-functions/AggregationTwoSpotsLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "QualityMetricCollision.h"

/****************************************/
/****************************************/

QualityMetricCollision::QualityMetricCollision() {
  m_unExperimentLenght = 0;
  m_unNumberCollisions = 0;
  m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

QualityMetricCollision::QualityMetricCollision(const QualityMetricCollision& orig) {}

/****************************************/
/****************************************/

void QualityMetricCollision::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
    TConfigurationNode cParametersNode;
    try {
      cParametersNode = GetNode(t_tree, "params");
    } catch(std::exception e) {
    }
}

/****************************************/
/****************************************/


QualityMetricCollision::~QualityMetricCollision() {}

/****************************************/
/****************************************/

void QualityMetricCollision::Destroy() {}

/****************************************/
/****************************************/

argos::CColor QualityMetricCollision::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  return CColor::GRAY50;
}


/****************************************/
/****************************************/

void QualityMetricCollision::Reset() {
  m_fObjectiveFunction = 0;
  m_unNumberCollisions = 0;
  m_unExperimentLenght = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void QualityMetricCollision::PostStep() {
  m_unExperimentLenght++;
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    if (pcEpuck->GetEmbodiedEntity().IsCollidingWithSomething()) {
      m_unNumberCollisions++;
    }
  }
}

/****************************************/
/****************************************/

void QualityMetricCollision::PostExperiment() {
  m_fObjectiveFunction = 1 - m_unNumberCollisions / (Real)(m_unExperimentLenght * m_unNumberRobots);
  LOG << "Score = " << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real QualityMetricCollision::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 QualityMetricCollision::GetRandomPosition() {
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

REGISTER_LOOP_FUNCTIONS(QualityMetricCollision, "collision_loop_func");
