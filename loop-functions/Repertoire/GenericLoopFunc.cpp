/**
  * @file <loop-functions/AggregationTwoSpotsLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "GenericLoopFunc.h"

/****************************************/
/****************************************/

GenericLoopFunc::GenericLoopFunc() {
  m_fObjectiveFunction = 0;
  m_strMissionDescription = "Undefined";
}

/****************************************/
/****************************************/

GenericLoopFunc::GenericLoopFunc(const GenericLoopFunc& orig) {}

/****************************************/
/****************************************/

void GenericLoopFunc::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
    TConfigurationNode cParametersNode;
    try {
      cParametersNode = GetNode(t_tree, "params");
      GetNodeAttribute(cParametersNode, "mission_description", m_strMissionDescription);
    } catch(std::exception e) {
    }
    m_vecMissionDescription = SplitDescriptionString(m_strMissionDescription);

    std::vector<std::string>::iterator it;

    try {
      it = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), "--m");
      m_strMissionType = (*(it+1)).c_str();
      it = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), "--np");
      m_unNumberPatches = atoi((*(it+1)).c_str());
      std::vector<std::string>::iterator vecFirstPatch;
			std::vector<std::string>::iterator vecSecondPatch;
      for (UInt32 i = 0; i < m_unNumberPatches; i++) {
        std::ostringstream oss;
        oss << "--tp" << i;
        vecFirstPatch = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), oss.str());
        if (i+1 < m_unNumberPatches) {
          std::ostringstream oss;
          oss << "--tp" << i+1;
          vecSecondPatch = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), oss.str());
        } else {
          vecSecondPatch = m_vecMissionDescription.end();
        }
        std::vector<std::string> vecPatchConfig(vecFirstPatch, vecSecondPatch);
        HandlePatch(vecPatchConfig);
      }
    } catch (std::exception e) {
      THROW_ARGOSEXCEPTION("Error while parsing mission description.");
    }
}

/****************************************/
/****************************************/

void GenericLoopFunc::HandlePatch(std::vector<std::string>& vec_patch_config) {
  std::cout << "PAAAATCH" << std::endl;
  for (std::vector<std::string>::iterator it = vec_patch_config.begin(); it!=vec_patch_config.end(); ++it) {
     std::cout << *it << " ";
  }
  std::cout << std::endl;
}

/****************************************/
/****************************************/

void GenericLoopFunc::PostStep() {

}

/****************************************/
/****************************************/


GenericLoopFunc::~GenericLoopFunc() {}

/****************************************/
/****************************************/

void GenericLoopFunc::Destroy() {}

/****************************************/
/****************************************/

argos::CColor GenericLoopFunc::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  return CColor::GRAY50;
}

/****************************************/
/****************************************/

std::vector<std::string> GenericLoopFunc::SplitDescriptionString(std::string& str_string_to_split) {
  std::stringstream cStringStream(str_string_to_split);
  std::istream_iterator<std::string> cStringBegin(cStringStream);
  std::istream_iterator<std::string> cStringEnd;
  std::vector<std::string> vStrings(cStringBegin, cStringEnd);
  // for (std::vector<std::string>::iterator it = vStrings.begin(); it!=vStrings.end(); ++it) {
  //   std::cout << *it << std::endl;
  // }
  return vStrings;
}


/****************************************/
/****************************************/

void GenericLoopFunc::Reset() {
  m_fObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void GenericLoopFunc::PostExperiment() {

}

/****************************************/
/****************************************/

Real GenericLoopFunc::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 GenericLoopFunc::GetRandomPosition() {
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

REGISTER_LOOP_FUNCTIONS(GenericLoopFunc, "generic_loop_func");
