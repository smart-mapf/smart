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
#include <fmt/core.h>
#include <thread>

using namespace argos;
using namespace fmt;
using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

void ExternalVisualizer::Screenshot(CSimulator &simulator) {}

void ExternalVisualizer::Init(TConfigurationNode &t_tree) {}

void ExternalVisualizer::Reset() {}

void ExternalVisualizer::Destroy() {}

void ExternalVisualizer::Execute() {

  while (true) {
    auto &simulator = CSimulator::GetInstance();
    auto &space = simulator.GetSpace();
    auto &bots = space.GetEntitiesByType("foot-bot");
    for (const auto &it : bots) {
      auto &bot = *any_cast<CFootBotEntity *>(it.second);
      auto &anchor = bot.GetEmbodiedEntity().GetOriginAnchor();
      auto &p = anchor.Position;
      LOG << format("{}: ({:.2f}, {:.2f}, {:.2f})", bot.GetId(), p.GetX(),
                    p.GetY(), p.GetZ())
          << endl;
    }

    CSimulator::GetInstance().UpdateSpace();

    ExternalVisualizer::Screenshot(simulator);

    sleep_for(milliseconds(1000 / 60));
  }
}

REGISTER_VISUALIZATION(ExternalVisualizer, "external_visualizer", "Kevin Zheng",
                       "0.0.1", "External Visualizer", "External Visualizer",
                       "Experimental")
