#ifndef FOOTBOT_NN_CONTROLLER
#define FOOTBOT_NN_CONTROLLER

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include "nn/perceptron.h"

using namespace argos;

class CFootBotNNController : public CCI_Controller {

public:

   CFootBotNNController();
   virtual ~CFootBotNNController();

   void Init(TConfigurationNode& t_node);
   void ControlStep();
   void Reset();
   void Destroy();

   inline CPerceptron& GetPerceptron() {
      return m_cPerceptron;
   }

private:

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_FootBotProximitySensor* m_pcProximity;
   CCI_FootBotLightSensor* m_pcLight;
   CPerceptron m_cPerceptron;
   Real m_fLeftSpeed, m_fRightSpeed;

};

#endif
