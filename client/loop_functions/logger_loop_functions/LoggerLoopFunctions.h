#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/visualization/visualization.h>

class CLoggerLoopFunctions : public argos::CLoopFunctions {
public:
  virtual void PostStep() override;
};