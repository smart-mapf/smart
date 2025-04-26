#include "mpga_phototaxis_loop_functions.h"

CMPGAPhototaxisLoopFunctions::CMPGAPhototaxisLoopFunctions() :
   m_vecInitSetup(5),
   m_pcFootBot(NULL),
   m_pcController(NULL),
   m_pfControllerParams(new Real[GENOME_SIZE]),
   m_pcRNG(NULL) {}

CMPGAPhototaxisLoopFunctions::~CMPGAPhototaxisLoopFunctions() {
   delete[] m_pfControllerParams;
}

void CMPGAPhototaxisLoopFunctions::Init(TConfigurationNode& t_node) {
   m_pcRNG = CRandom::CreateRNG("argos");

   m_pcFootBot = new CFootBotEntity(
      "fb",
      "fnn"
      );
   AddEntity(*m_pcFootBot);
   m_pcController = &dynamic_cast<CFootBotNNController&>(m_pcFootBot->GetControllableEntity().GetController());

   CRadians cOrient;
   for(size_t i = 0; i < 5; ++i) {
      m_vecInitSetup[i].Position.FromSphericalCoords(
         4.5f,
         CRadians::PI_OVER_TWO,
         static_cast<Real>(i+1) * CRadians::PI / 12.0f
         );
      cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
      m_vecInitSetup[i].Orientation.FromEulerAngles(
         cOrient,
         CRadians::ZERO,
         CRadians::ZERO
         );
   }

   try {
      UInt32 unTrial;
      GetNodeAttribute(t_node, "trial", unTrial);
      SetTrial(unTrial);
      Reset();
   }
   catch(CARGoSException& ex) {}
}

void CMPGAPhototaxisLoopFunctions::Reset() {
   if(!MoveEntity(
         m_pcFootBot->GetEmbodiedEntity(),
         m_vecInitSetup[GetTrial()].Position,
         m_vecInitSetup[GetTrial()].Orientation,
         false
         )) {
      LOGERR << "Can't move robot in <"
             << m_vecInitSetup[GetTrial()].Position
             << ">, <"
             << m_vecInitSetup[GetTrial()].Orientation
             << ">"
             << std::endl;
   }
}

void CMPGAPhototaxisLoopFunctions::ConfigureFromGenome(const Real* pf_genome) {
   for(size_t i = 0; i < GENOME_SIZE; ++i) {
      m_pfControllerParams[i] = pf_genome[i];
   }
   m_pcController->GetPerceptron().SetOnlineParameters(GENOME_SIZE, m_pfControllerParams);
}

Real CMPGAPhototaxisLoopFunctions::Score() {
   return m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position.Length();
}

REGISTER_LOOP_FUNCTIONS(CMPGAPhototaxisLoopFunctions, "mpga_phototaxis_loop_functions")
