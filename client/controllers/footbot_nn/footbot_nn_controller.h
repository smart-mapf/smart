#ifndef FOOTBOT_NN_CONTROLLER
#define FOOTBOT_NN_CONTROLLER

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the perceptron */
#include "nn/perceptron.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 * In this case, we also inherit from the CPerceptron class. We use
 * virtual inheritance so that matching methods in the CCI_Controller
 * and CPerceptron don't get messed up.
 */
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

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* The perceptron neural network */
   CPerceptron m_cPerceptron;
   /* Wheel speeds */
   Real m_fLeftSpeed, m_fRightSpeed;

};

#endif
