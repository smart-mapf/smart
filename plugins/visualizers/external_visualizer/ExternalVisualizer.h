#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/visualization/visualization.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

class ExternalVisualizer : public argos::CVisualization {
public:
  virtual void Init(argos::TConfigurationNode &t_tree) override;
  virtual void Reset() override;
  virtual void Destroy() override;
  virtual void Execute() override;
  virtual void Capture();
};