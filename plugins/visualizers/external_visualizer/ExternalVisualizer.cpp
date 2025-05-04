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

#include "log.h"

using namespace argos;
using namespace fmt;
using namespace std;
using namespace std::chrono;

void ExternalVisualizer::Capture() {
  auto &instance = CSimulator::GetInstance();
  auto &space = instance.GetSpace();
  auto &bots = space.GetEntitiesByType("foot-bot");
  item(
    obj(
      val("clock", space.GetSimulationClock());
      key("agents",      
        list(
          for (const auto &it : bots) {
            auto &bot = *any_cast<CFootBotEntity *>(it.second);
            auto &anchor = bot.GetEmbodiedEntity().GetOriginAnchor();
            auto &p = anchor.Position;
            obj( 
              val("id", bot.GetId());
              val("x", p.GetX(), "{:.2f}");
              val("y", p.GetY(), "{:.2f}");
              val("z", p.GetZ(), "{:.2f}");
              val("rx", p.GetXAngle().GetValue(), "{:.2f}");
              val("ry", p.GetYAngle().GetValue(), "{:.2f}");
              val("rz", p.GetZAngle().GetValue(), "{:.2f}");
            );
          }
        );
      );
    )
  )}

void ExternalVisualizer::Init(TConfigurationNode &t_tree) {}

void ExternalVisualizer::Reset() {}

void ExternalVisualizer::Destroy() {}

#define DEFINITION 2

void ExternalVisualizer::Execute() {
  cout << "---" << endl;
  ExternalVisualizer::Capture();
  while (true) {
    auto &instance = CSimulator::GetInstance();
    auto t = instance.GetSpace().GetSimulationClock();
    if (t % DEFINITION == 0) {
      ExternalVisualizer::Capture();
    }
    instance.UpdateSpace();
  };
  cout << "---" << endl;
}

REGISTER_VISUALIZATION(ExternalVisualizer, "external_visualizer", "Kevin Zheng",
                       "0.0.1", "External Visualizer", "External Visualizer",
                       "Experimental")
