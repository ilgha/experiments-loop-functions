#ifndef BABY_GRID_EXPLORATION_LOOP_FUNC
#define BABY_GRID_EXPLORATION_LOOP_FUNC

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class BabyGridExplorationLoopFunction : public CoreLoopFunctions
{
public:
  BabyGridExplorationLoopFunction();
  BabyGridExplorationLoopFunction(const BabyGridExplorationLoopFunction &orig);
  virtual ~BabyGridExplorationLoopFunction();

  virtual void Destroy();

  virtual argos::CColor GetFloorColor(const argos::CVector2 &c_position_on_plane);
  virtual void PostExperiment();
  virtual void PostStep();
  virtual void Init(TConfigurationNode &t_tree);
  virtual void Reset();
  virtual CVector3 GetRandomPosition();

  Real GetObjectiveFunction();

private:
  std::vector<std::vector<int>> m_grid;

  Real m_arenaSize;
  UInt32 m_gridSize;
  Real m_fObjectiveFunction;
};

#endif
