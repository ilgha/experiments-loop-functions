#include "epuck_planning_user_functions.h"

using namespace argos;

/****************************************/
/****************************************/

CEPuckPlanningQTOpenGLUserFunctions::CEPuckPlanningQTOpenGLUserFunctions() {
    RegisterUserFunction<CEPuckPlanningQTOpenGLUserFunctions,CEPuckEntity>(&CEPuckPlanningQTOpenGLUserFunctions::Draw);
    RegisterUserFunction<CEPuckPlanningQTOpenGLUserFunctions,CBoxEntity>(&CEPuckPlanningQTOpenGLUserFunctions::Draw);
    m_pcDrawRobotIDFont            = QFont("Times", 15,0,false);
    m_pcDrawWallIDFont            = QFont("Times", 10,0,false);
}

/****************************************/
/****************************************/

CEPuckPlanningQTOpenGLUserFunctions::~CEPuckPlanningQTOpenGLUserFunctions() {
}

/****************************************/
/****************************************/

void CEPuckPlanningQTOpenGLUserFunctions::Draw(CEPuckEntity& c_entity) {
  //GetQTOpenGLWidget().qglColor(QColor(0,0,255));
  //GetQTOpenGLWidget().renderText(-0.01, -0.01, 0.14, QString(c_entity.GetId().substr(5,6).c_str()), m_pcDrawIDFont);
  DrawText(CVector3(-0.01, -0.01, 0.30), c_entity.GetId().substr(5,6).c_str(), CColor::RED, m_pcDrawRobotIDFont);
}

/****************************************/
/****************************************/

void CEPuckPlanningQTOpenGLUserFunctions::Draw(CBoxEntity& c_entity) {
  DrawText(CVector3(-0.01, -0.01, 0.14), c_entity.GetId().substr(5,6).c_str(), CColor::BLUE, m_pcDrawWallIDFont);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CEPuckPlanningQTOpenGLUserFunctions, "epuck_planning_qtopengl_user_functions")
