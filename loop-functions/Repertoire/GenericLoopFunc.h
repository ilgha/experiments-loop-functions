/**
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef GENERIC_LOOP_FUNC
#define GENERIC_LOOP_FUNC

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/core/simulator/physics_engine/physics_model.h>


#include "../../src/CoreLoopFunctions.h"

using namespace argos;

class GenericLoopFunc: public CoreLoopFunctions {
  public:

    struct CPatch {
      std::string Type;
      std::string Color;
      Real CenterX;
      Real CenterY;
      Real Size;   // Radius if Type == 'circ'; SideLength if Type == 'rect'
    };

    GenericLoopFunc();
    GenericLoopFunc(const GenericLoopFunc& orig);
    virtual ~GenericLoopFunc();

    virtual void Destroy();
    virtual void Init(TConfigurationNode& t_tree);

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();
    virtual void Reset();

    Real GetObjectiveFunction();

    CVector3 GetRandomPosition();

  private:
    std::vector<std::string> SplitDescriptionString(std::string& str_string_to_split);
    void HandlePatch(std::vector<std::string>& vec_patch_config, UInt32 un_index_patch);
    bool IsPointOnPatch(CVector2& c_point, CPatch& s_patch);
    void InitializeMission();
    CColor GetColorFromString(std::string str_color);



    Real m_fObjectiveFunction;

    std::string m_strMissionDescription;
    std::vector<std::string> m_vecMissionDescription;

    std::string m_strMissionType;
    std::string m_strFloorColor;
    UInt32 m_unExperimentLength;
    UInt32 m_unNumberPatches;
    std::vector<CPatch> m_vecPatches;

    /* Foraging */
    std::string m_strColorNest;
    std::string m_strColorFoodSource;

};

#endif
