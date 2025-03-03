#include "footbot_nn_controller.h"

static CRange<Real> NN_OUTPUT_RANGE(0.0f, 1.0f);
static CRange<Real> WHEEL_ACTUATION_RANGE(-5.0f, 5.0f);

CFootBotNNController::CFootBotNNController() {
}

CFootBotNNController::~CFootBotNNController() {
}

void CFootBotNNController::Init(TConfigurationNode& t_node) {
   try {
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
      m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex);
   }

   try {
      m_cPerceptron.Init(t_node);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the perceptron network", ex);
   }
}

void CFootBotNNController::ControlStep() {
   const CCI_FootBotProximitySensor::TReadings& tProx = m_pcProximity->GetReadings();
   const CCI_FootBotLightSensor::TReadings& tLight = m_pcLight->GetReadings();
   for(size_t i = 0; i < tProx.size(); ++i) {
      m_cPerceptron.SetInput(i, tProx[i].Value);
   }
   for(size_t i = 0; i < tLight.size(); ++i) {
      m_cPerceptron.SetInput(tProx.size()+i, tLight[i].Value);
   }
   m_cPerceptron.ComputeOutputs();
   NN_OUTPUT_RANGE.MapValueIntoRange(
      m_fLeftSpeed,
      m_cPerceptron.GetOutput(0),
      WHEEL_ACTUATION_RANGE
      );
   NN_OUTPUT_RANGE.MapValueIntoRange(
      m_fRightSpeed,
      m_cPerceptron.GetOutput(1),
      WHEEL_ACTUATION_RANGE
      );
   m_pcWheels->SetLinearVelocity(
      m_fLeftSpeed,
      m_fRightSpeed);
}

void CFootBotNNController::Reset() {
   m_cPerceptron.Reset();
}

void CFootBotNNController::Destroy() {
   m_cPerceptron.Destroy();
}

REGISTER_CONTROLLER(CFootBotNNController, "footbot_nn_controller")
