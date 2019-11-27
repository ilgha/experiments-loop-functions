/**
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef QUALITY_METRIC_COLLISION
#define QUALITY_METRIC_COLLISION

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

#include "../../src/CoreLoopFunctions.h"

using namespace argos;

class QualityMetricCollision: public CoreLoopFunctions {
  public:
    QualityMetricCollision();
    QualityMetricCollision(const QualityMetricCollision& orig);
    virtual ~QualityMetricCollision();

    virtual void Destroy();
    virtual void Init(TConfigurationNode& t_tree);

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();
    virtual void Reset();

    Real GetObjectiveFunction();

    CVector3 GetRandomPosition();

  private:
    UInt32 m_unNumberCollisions;
    UInt32 m_unExperimentLenght;
    Real m_fObjectiveFunction;
};

#endif
