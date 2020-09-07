/**
 * Any-Point Closseness loop function
 *
 * @file <loop-functions/mate/APC/MateAPCLoopFunc.h>
 *
 * @author Fernando Mendiburu - <fmendibu@ulb.ac.be>
 *
 * @package experiments-loop-functions
 *
 * @license MIT License
 */

 #ifndef MATE_APC_LOOP_FUNC_H
 #define MATE_APC_LOOP_FUNC_H

 #include "../../../src/CoreLoopFunctions.h"
 #include <argos3/core/simulator/space/space.h>
 #include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class MateAPCLoopFunction : public CoreLoopFunctions {

   public:
      MateAPCLoopFunction();
      MateAPCLoopFunction(const MateAPCLoopFunction& orig);
      virtual ~MateAPCLoopFunction();

      virtual void Destroy();
      virtual void Reset();
      virtual void PostExperiment();


      Real GetObjectiveFunction();

      virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

      virtual CVector3 GetRandomPosition();

    private:
      Real ComputeObjectiveFunction();
      CVector2 RandomPointOnSquareArea();
      bool IsOnSquareArea(const CVector2& c_point);

      Real m_fRadiusRobot;
      Real m_fSideSquare;
      CVector2 m_cCoordSquareSpot;
      UInt32 m_unNumberPoints;
      Real m_fObjectiveFunction;

};

#endif
