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
      InitializeMission();
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
        HandlePatch(vecPatchConfig, i);
      }
    } catch (std::exception e) {
      LOGERR << "Error while parsing mission description: " << e.what() << std::endl;
    }
}

/****************************************/
/****************************************/

void GenericLoopFunc::InitializeMission() {
  std::vector<std::string>::iterator it;

  if (m_strMissionType == "for") {
    try {
      it = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), "--cnf");
      m_strColorNest = (*(it+1)).c_str();
      it = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), "--cfsf");
      m_strColorFoodSource = (*(it+1)).c_str();
    } catch (std::exception e) {
      LOGERR << "Error initializing mission: " << e.what() << std::endl;
    }
  } else {
    LOGERR << "Error: Unknown mission type: " << m_strMissionType << std::endl;
  }
}

/****************************************/
/****************************************/

void GenericLoopFunc::HandlePatch(std::vector<std::string>& vec_patch_config, UInt32 un_index_patch) {
  CPatch currentPatch;
  std::vector<std::string>::iterator it;

  try {
    std::ostringstream oss;
    // Extracting type
    oss << "--tp" << un_index_patch;
    it = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), oss.str());
    currentPatch.Type = (*(it+1)).c_str();
    // Extracting size
    oss.str("");
    oss << "--sp" << un_index_patch;
    it = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), oss.str());
    currentPatch.Size = strtod((*(it+1)).c_str(), NULL);
    // Extracting coordinate X of center
    oss.str("");
    oss << "--cxp" << un_index_patch;
    it = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), oss.str());
    currentPatch.CenterX = strtod((*(it+1)).c_str(), NULL);
    // Extracting coordinate Y of center
    oss.str("");
    oss << "--cyp" << un_index_patch;
    it = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), oss.str());
    currentPatch.CenterY = strtod((*(it+1)).c_str(), NULL);
    // Extracting color
    oss.str("");
    oss << "--cp" << un_index_patch;
    it = std::find(m_vecMissionDescription.begin(), m_vecMissionDescription.end(), oss.str());
    currentPatch.Color = (*(it+1)).c_str();
  } catch (std::exception e) {
    THROW_ARGOSEXCEPTION("Error while parsing patch description.");
  }
  m_vecPatches.push_back(currentPatch);
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
  std::vector<CPatch>::iterator it;

  for (it = m_vecPatches.begin(); it != m_vecPatches.end(); ++it) {
    if (IsPointOnPatch(vCurrentPoint, (*it))) {
      return GetColorFromString((*it).Color);
    }
  }

  return CColor::GRAY50;
}

/****************************************/
/****************************************/

bool GenericLoopFunc::IsPointOnPatch(CVector2& c_point, CPatch& s_patch) {
  CVector2 centerPatch(s_patch.CenterX, s_patch.CenterY);
  if (s_patch.Type == "rect") {
    if ((c_point.GetX() <= centerPatch.GetX() + s_patch.Size/2) and (c_point.GetX() >= centerPatch.GetX() - s_patch.Size/2) and (c_point.GetY() <= centerPatch.GetY() + s_patch.Size/2) and (c_point.GetY() >= centerPatch.GetY() - s_patch.Size/2)){
      return true;
    } else {
      return false;
    }
  } else if (s_patch.Type == "circ") {
    Real distancePointCenter = (c_point - centerPatch).Length();
    if (distancePointCenter < s_patch.Size) {
      return true;
    } else {
      return false;
    }
  } else {
    THROW_ARGOSEXCEPTION("Unknown patch type : " + s_patch.Type)
  }
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

/****************************************/
/****************************************/

CColor GenericLoopFunc::GetColorFromString(std::string str_color) {
  CColor returnColor;
  if (str_color == "white") {
    returnColor = CColor::WHITE;
  } else if (str_color == "black") {
    returnColor = CColor::BLACK;
  } else if (str_color == "gray") {
    returnColor = CColor::GRAY50;
  }
  return returnColor;
}

REGISTER_LOOP_FUNCTIONS(GenericLoopFunc, "generic_loop_func");
