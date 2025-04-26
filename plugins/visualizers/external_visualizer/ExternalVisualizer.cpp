#include "ExternalVisualizer.h"
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_main_window.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>
#include <thread>

void ExternalVisualizer::Screenshot(argos::CSimulator &simulator) {}

void ExternalVisualizer::Init(argos::TConfigurationNode &t_tree) {}

void ExternalVisualizer::Reset() {}

void ExternalVisualizer::Destroy() {}

void ExternalVisualizer::Execute() {
  while (true) {
    auto &cSimulator = argos::CSimulator::GetInstance();
    auto &cSpace = cSimulator.GetSpace();
    auto &tFootBots = cSpace.GetEntitiesByType("foot-bot");

    for (const auto &it : tFootBots) {
      argos::CFootBotEntity &pcFootBot =
          *argos::any_cast<argos::CFootBotEntity *>(it.second);
      const argos::CVector3 &cPos =
          pcFootBot.GetEmbodiedEntity().GetOriginAnchor().Position;
      argos::LOG << pcFootBot.GetId() << ": (" << cPos.GetX() << ", "
                 << cPos.GetY() << ", " << cPos.GetZ() << ")" << std::endl;
    }

    argos::CSimulator::GetInstance().UpdateSpace();

    ExternalVisualizer::Screenshot(cSimulator);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    break;
  }
}

REGISTER_VISUALIZATION(ExternalVisualizer, "external_visualizer", "Kevin Zheng",
                       "0.0.1", "External Visualizer", "External Visualizer",
                       "Experimental")
