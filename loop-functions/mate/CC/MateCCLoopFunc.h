/**
 * Conditional Coverage loop function
 *
 * @file <loop-functions/mate/CC/MateCCLoopFunc.h>
 *
 * @author Fernando Mendiburu - <fmendibu@ulb.ac.be>
 *
 * @package experiments-loop-functions
 *
 * @license MIT License
 */

 #ifndef MATE_CC_LOOP_FUNC_H
 #define MATE_CC_LOOP_FUNC_H

 #include "../../../src/CoreLoopFunctions.h"
 #include <argos3/core/simulator/space/space.h>
 #include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class MateCCLoopFunction : public CoreLoopFunctions {

   public:
      MateCCLoopFunction();
      MateCCLoopFunction(const MateCCLoopFunction& orig);
      virtual ~MateCCLoopFunction();

      virtual void Destroy();
      virtual void Reset();
      virtual void PostExperiment();
      virtual void PostStep();
      virtual void Init(TConfigurationNode& t_tree);

      Real GetObjectiveFunction();

      virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

      virtual CVector3 GetRandomPosition();

    private:

      struct CAgent {
        CVector2 cPosition;
        UInt32 unRobotID;
        UInt32 unClusterID;
        CAgent(CVector2 c_position, UInt32 un_RobotID) {
          cPosition = c_position;
          unRobotID = un_RobotID;
          unClusterID = 0;
        }
        CAgent(CVector2 c_position, UInt32 un_RobotID, UInt32 un_GroupID) {
          cPosition = c_position;
          unRobotID = un_RobotID;
          unClusterID = un_GroupID;
        }

      };

      Real ComputeObjectiveFunction();
      Real ComputeCoverageRatioOnArea(std::vector<CAgent>& agents);
      Real ComputeCoverageRatioOnArea2(std::vector<CAgent>& agents);
      CVector2 RandomPointOnArea();
      CVector2 RandomPointOnArea2();
      bool IsOnArea(const CVector2& c_position);
      bool IsOnArea2(const CVector2& c_position);
      bool IsInsideArena(const CVector2& c_position);

      std::vector<MateCCLoopFunction::CAgent> PickAgents();
      UInt32 DetermineBiggestGroup(std::vector<CAgent>& agents);

      void AddNeighs(std::vector<CAgent>& agents, std::vector<CAgent>::iterator ag);

      //Real ComputeCoverageRatio(std::vector<CAgent>& agents, UInt32 GroupID);

      //void SelectColorOrder();

      Real m_fRadiusRobot;
      Real m_fSideSquare;
      CVector2 m_cCoordSquareSpot;
      UInt32 m_unNumberPoints;
      Real m_fObjectiveFunction;
      Real m_fAreaLimit;
      Real m_fSensingRange;
      Real m_fCommunicationDistance;

};

#endif
