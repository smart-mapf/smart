#include "footbot_nn_controller.h"

/****************************************/
/****************************************/

static CRange<Real> NN_OUTPUT_RANGE(0.0f, 1.0f);
static CRange<Real> WHEEL_ACTUATION_RANGE(-5.0f, 5.0f);

/****************************************/
/****************************************/

CFootBotNNController::CFootBotNNController() {
}

/****************************************/
/****************************************/

CFootBotNNController::~CFootBotNNController() {
}

/****************************************/
/****************************************/

void CFootBotNNController::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    */
   try {
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
      m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex);
   }

   /* Initialize the perceptron */
   try {
      m_cPerceptron.Init(t_node);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the perceptron network", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotNNController::ControlStep() {
   /* Get sensory data */
   const CCI_FootBotProximitySensor::TReadings& tProx = m_pcProximity->GetReadings();
   const CCI_FootBotLightSensor::TReadings& tLight = m_pcLight->GetReadings();
   /* Fill NN inputs from sensory data */
   for(size_t i = 0; i < tProx.size(); ++i) {
      m_cPerceptron.SetInput(i, tProx[i].Value);
   }
   for(size_t i = 0; i < tLight.size(); ++i) {
      m_cPerceptron.SetInput(tProx.size()+i, tLight[i].Value);
   }
   /* Compute NN outputs */
   m_cPerceptron.ComputeOutputs();
   /*
    * Apply NN outputs to actuation
    * The NN outputs are in the range [0,1]
    * To allow for backtracking, we remap this range
    * into [-5:5] linearly.
    */
   NN_OUTPUT_RANGE.MapValueIntoRange(
      m_fLeftSpeed,               // value to write
      m_cPerceptron.GetOutput(0), // value to read
      WHEEL_ACTUATION_RANGE       // target range (here [-5:5])
      );
   NN_OUTPUT_RANGE.MapValueIntoRange(
      m_fRightSpeed,              // value to write
      m_cPerceptron.GetOutput(1), // value to read
      WHEEL_ACTUATION_RANGE       // target range (here [-5:5])
      );
   m_pcWheels->SetLinearVelocity(
      m_fLeftSpeed,
      m_fRightSpeed);
}

/****************************************/
/****************************************/

void CFootBotNNController::Reset() {
   m_cPerceptron.Reset();
}

/****************************************/
/****************************************/

void CFootBotNNController::Destroy() {
   m_cPerceptron.Destroy();
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotNNController, "footbot_nn_controller")
