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
#include <array>
#include <fmt/core.h>

#include "log.h"

using namespace argos;
using namespace fmt;
using namespace std;
using namespace std::chrono;

void ExternalVisualizer::Capture() {
  auto &space = CSimulator::GetInstance().GetSpace();

  std::vector<std::pair<std::string, CFootBotEntity*>> bots;
  for (const auto &it : space.GetEntitiesByType("foot-bot")) {
    auto &bot = *any_cast<CFootBotEntity *>(it.second);
    auto index = bot.GetConfigurationNode()->GetAttributeOrDefault("index", "default");
    bots.emplace_back(index, &bot);
  }

  std::sort(bots.begin(), bots.end(), [](const auto &a, const auto &b) { return stoi(a.first) < stoi(b.first); });

  item(
    val("type", "tick"); 
    val("clock", space.GetSimulationClock());
    key("agents", 
      list(
        for (const auto &it : bots) {
            auto &bot = *any_cast<CFootBotEntity *>(it.second);
            auto &anchor = bot.GetEmbodiedEntity().GetOriginAnchor();
            auto &rotation = anchor.Orientation;
            CRadians cZAngle;
            CRadians cYAngle;
            CRadians cXAngle;
            rotation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
            obj(
              val("id", bot.GetConfigurationNode()->GetAttributeOrDefault("index","default"));
              val("x", anchor.Position.GetX(), "{:.2f}");
              val("y", anchor.Position.GetY(), "{:.2f}"); 
              val("z", anchor.Position.GetZ(), "{:.2f}");
              val("rx", cXAngle.GetValue(), "{:.2f}");
              val("ry", cYAngle.GetValue(), "{:.2f}");
              val("rz", cZAngle.GetValue(), "{:.2f}");
            );
           }
        );
      );
    )
}

void ExternalVisualizer::Init(TConfigurationNode &t_tree) {}

void ExternalVisualizer::Reset() {}

void ExternalVisualizer::Destroy() {}

#define DEFINITION 1

void ExternalVisualizer::Execute() {
  auto &instance = CSimulator::GetInstance();
  while (!instance.IsExperimentFinished()) {
    auto t = instance.GetSpace().GetSimulationClock();
    if (t % DEFINITION == 0) {
      ExternalVisualizer::Capture();
    }
    instance.UpdateSpace();
  };
}

REGISTER_VISUALIZATION(
  ExternalVisualizer, 
  "external_visualizer", 
  "Kevin Zheng",     
  "0.0.1", 
  "External Visualizer", 
  "External Visualizer",
  "Experimental"
)
