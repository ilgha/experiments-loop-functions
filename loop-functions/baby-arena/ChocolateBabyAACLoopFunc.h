/*
 * Aggregation with Ambient Clues
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 */

#ifndef CHOCOLATE_BABY_AAC_LOOP_FUNC_H
#define CHOCOLATE_BABY_AAC_LOOP_FUNC_H

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class ChocolateBabyAACLoopFunction : public CoreLoopFunctions {

   public:
      ChocolateBabyAACLoopFunction();
      ChocolateBabyAACLoopFunction(const ChocolateBabyAACLoopFunction& orig);
      virtual ~ChocolateBabyAACLoopFunction();

      virtual void Destroy();
      virtual void Reset();
      virtual void PostStep();
      virtual void PostExperiment();

      Real GetObjectiveFunction();

      virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

      virtual CVector3 GetRandomPosition();

    private:
      Real m_fRadius;
      CVector2 m_cCoordBlackSpot;
      CVector2 m_cCoordWhiteSpot;

      Real m_fObjectiveFunction;
};

#endif
