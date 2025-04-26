#include "LoggerLoopFunctions.h"
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_main_window.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

using namespace argos;

void CLoggerLoopFunctions::PostStep() {
  struct stat info;
  if (stat("./frames", &info) != 0) {
    mkdir("./frames", 0777);
  }

  static int frame_id = 0;
  std::stringstream filename_s;

  filename_s << "frames/frame_" << std::setw(5) << std::setfill('0')
             << frame_id++ << ".png";
  std::string filename = filename_s.str();
  std::cout << "Saving frame to " << filename << std::endl;
  // Get the visualization object
  argos::CVisualization &cVisualization = GetSimulator().GetVisualization();
  // Ensure the visualization is of the expected Qt OpenGL type
  auto *pcQtRender = dynamic_cast<argos::CQTOpenGLRender *>(&cVisualization);
  if (pcQtRender) {
    // Access the main window
    argos::CQTOpenGLMainWindow &pcMainWindow = pcQtRender->GetMainWindow();

    // Capture the current window as a pixmap
    QPixmap pixmap = pcMainWindow.grab();

    // Save the pixmap to the specified file
    if (!pixmap.save(QString::fromStdString(filename))) {
      std::cerr << "Failed to save the visualization snapshot to " << filename
                << std::endl;
    } else {
      std::cout << "Visualization snapshot saved to " << filename << std::endl;
    }
  } else {
    std::cerr << "Qt OpenGL Visualization is not available." << std::endl;
  }
  // CSpace &cSpace = GetSpace();
  // for (auto &cEntity : cSpace.GetEntitiesByType("foot-bot")) {
  //   CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(cEntity.second);
  //   const CVector3 &cPosition =
  //       cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position;
  //   std::cout << cFootBot.GetId() << " Position: " << cPosition << std::endl;
  // }
}

REGISTER_LOOP_FUNCTIONS(CLoggerLoopFunctions, "logger_loop_functions")