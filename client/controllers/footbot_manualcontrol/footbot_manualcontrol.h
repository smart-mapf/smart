#ifndef FOOTBOT_MANUALCONTROL_H
#define FOOTBOT_MANUALCONTROL_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CFootBotManualControl : public CCI_Controller {

public:

   struct SWheelTurningParams {
      enum ETurningMechanism
      {
         NO_TURN = 0,
         SOFT_TURN,
         HARD_TURN
      } TurningMechanism;
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

public:

   CFootBotManualControl();

   virtual ~CFootBotManualControl() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();

   virtual void Reset() {}

   virtual void Destroy() {}

   void Select();

   void Deselect();

   void SetControlVector(const CVector2& c_control);

protected:

   void SetWheelSpeedsFromVector(const CVector2& c_heading);

private:

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_LEDsActuator* m_pcLEDs;

   SWheelTurningParams m_sWheelTurningParams;

   bool m_bSelected;

   CVector2 m_cControl;
};

#endif
