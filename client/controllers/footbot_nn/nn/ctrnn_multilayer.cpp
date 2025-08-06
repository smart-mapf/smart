#include "ctrnn_multilayer.h"

#include <cmath>
#include <fstream>
#include <argos3/core/utility/string_utilities.h>

/****************************************/
/****************************************/

CCtrnnMultilayer::CCtrnnMultilayer() :
   m_pfInputToHiddenWeights(NULL),
   m_pfHiddenBiases(NULL),
   m_pfHiddenTaus(NULL),
   m_pfHiddenDeltaStates(NULL),
   m_pfHiddenStates(NULL),
   m_pfHiddenToHiddenWeights(NULL),
   m_pfHiddenToOutputWeights(NULL),
   m_pfOutputBiases(NULL),
   m_pfOutputTaus(NULL),
   m_unNumberOfHiddenNodes(0),
   m_fTimeStep(0.1f),
   m_cWeightsBounds(-4.0f, 4.0f),
   m_cBiasesBounds(-4.0f, 4.0f),
   m_cTausBounds(-1.0f, 3.0f) {}

/****************************************/
/****************************************/

CCtrnnMultilayer::~CCtrnnMultilayer() {
   if(m_pfInputToHiddenWeights)  delete[] m_pfInputToHiddenWeights;
   if(m_pfHiddenToHiddenWeights) delete[] m_pfHiddenToHiddenWeights;
   if(m_pfHiddenBiases)          delete[] m_pfHiddenBiases;
   if(m_pfHiddenToOutputWeights) delete[] m_pfHiddenToOutputWeights;
   if(m_pfOutputBiases)          delete[] m_pfOutputBiases;
   if(m_pfHiddenTaus)            delete[] m_pfHiddenTaus;
   if(m_pfHiddenDeltaStates)     delete[] m_pfHiddenDeltaStates;
   if(m_pfHiddenStates)          delete[] m_pfHiddenStates;
}

/****************************************/
/****************************************/

void CCtrnnMultilayer::Init( TConfigurationNode& t_node ) {
   ////////////////////////////////////////////////////////////////////////////////
   // First perform common initialisation from base class
   ////////////////////////////////////////////////////////////////////////////////
   CNeuralNetwork::Init( t_node );

   ////////////////////////////////////////////////////////////////////////////////
   // Load number of hidden nodes
   ////////////////////////////////////////////////////////////////////////////////
   try{
      GetNodeAttribute(t_node, "num_hidden", m_unNumberOfHiddenNodes);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Missing required tag <num_hidden> for CTRNN multi-layer initialisation", ex);
   }

   ////////////////////////////////////////////////////////////////////////////////
   // integration step
   ////////////////////////////////////////////////////////////////////////////////
   GetNodeAttribute(t_node, "integration_step", m_fTimeStep);

   ////////////////////////////////////////////////////////////////////////////////
   // Load upper and lower bounds for weigths, biases and taus
   ////////////////////////////////////////////////////////////////////////////////
   GetNodeAttribute(t_node, "weight_range", m_cWeightsBounds);
   GetNodeAttribute(t_node, "bias_range",  m_cBiasesBounds);
   GetNodeAttribute(t_node, "tau_range",    m_cTausBounds);

   ////////////////////////////////////////////////////////////////////////////////
   // check and load parameters from file
   ////////////////////////////////////////////////////////////////////////////////
   if(m_strParameterFile != "") {
      try {
         LoadNetworkParameters(m_strParameterFile);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("cannot load parameters from file.", ex);
      }
   }
}

/****************************************/
/****************************************/

void CCtrnnMultilayer::Reset() {
   for(size_t i = 0; i < m_unNumberOfHiddenNodes; ++i) {
      m_pfHiddenDeltaStates[i] = 0.0f;
      m_pfHiddenStates[i]      = 0.0f;
   }
}

/****************************************/
/****************************************/

void CCtrnnMultilayer::Destroy() {
   m_unNumberOfHiddenNodes = 0;

   if( m_pfInputToHiddenWeights )  delete[] m_pfInputToHiddenWeights;
   m_pfInputToHiddenWeights = NULL;

   if( m_pfHiddenToHiddenWeights ) delete[] m_pfHiddenToHiddenWeights;
   m_pfHiddenToHiddenWeights = NULL;

   if( m_pfHiddenBiases )          delete[] m_pfHiddenBiases;
   m_pfHiddenBiases = NULL;

   if( m_pfHiddenToOutputWeights ) delete[] m_pfHiddenToOutputWeights;
   m_pfHiddenToOutputWeights = NULL;

   if( m_pfOutputBiases )          delete[] m_pfOutputBiases;
   m_pfOutputBiases = NULL;

   if( m_pfHiddenTaus )            delete[] m_pfHiddenTaus;
   m_pfHiddenTaus = NULL;

   if( m_pfHiddenDeltaStates )     delete[] m_pfHiddenDeltaStates;
   m_pfHiddenDeltaStates = NULL;

   if( m_pfHiddenStates )          delete[] m_pfHiddenStates;
   m_pfHiddenStates = NULL;
}


/****************************************/
/****************************************/

void CCtrnnMultilayer::LoadNetworkParameters(const std::string& str_filename ) {
   std::ifstream cIn;
   if( !cIn ) {
      THROW_ARGOSEXCEPTION("Cannot open parameter file '" << str_filename << "' for reading");
   }

   // first parameter is the number of real-valued weights
   UInt32 un_length = 0;
   if( !(cIn >> un_length) ) {
      THROW_ARGOSEXCEPTION("Cannot read data from file '" << str_filename << "'");
   }


   // create weights vector and load it from file
   Real* pfGenes = new Real[un_length];
   for( UInt32 i = 0; i < un_length; i++ ) {
      if( !(cIn >> pfGenes[i]) ) {
         delete[] pfGenes;
         THROW_ARGOSEXCEPTION("Cannot read data from file '" << str_filename << "'");
      }
   }

   // load parameters in the appropriate structures
   LoadNetworkParameters(un_length, pfGenes);
   delete[] pfGenes;
}


/****************************************/
/****************************************/

void CCtrnnMultilayer::LoadNetworkParameters( const UInt32 un_num_params, const Real* params ) {
   // check consistency between paramter file and xml declaration
   UInt32 un_num_parameters =
      m_unNumberOfHiddenNodes * (m_unNumberOfInputs + 1)  +
      m_unNumberOfHiddenNodes * m_unNumberOfHiddenNodes   +
      m_unNumberOfOutputs * (m_unNumberOfHiddenNodes + 1) +
      m_unNumberOfHiddenNodes;

   if(un_num_params != un_num_parameters) {
      THROW_ARGOSEXCEPTION("Number of parameter mismatch: '"
                           << "passed "
                           << un_num_params
                           << " parameters, while "
                           << un_num_parameters
                           << " were expected from the xml configuration file");
   }

   UInt32 unChromosomePosition = 0;

   if( m_pfInputToHiddenWeights == NULL ) m_pfInputToHiddenWeights = new Real[m_unNumberOfInputs * m_unNumberOfHiddenNodes];
   for( UInt32 i = 0; i < m_unNumberOfInputs * m_unNumberOfHiddenNodes; i++ ) {
      m_pfInputToHiddenWeights[i] = params[unChromosomePosition++]*(m_cWeightsBounds.GetMax() - m_cWeightsBounds.GetMin() ) + m_cWeightsBounds.GetMin();
   }

   if( m_pfHiddenToHiddenWeights == NULL ) m_pfHiddenToHiddenWeights = new Real[m_unNumberOfHiddenNodes * m_unNumberOfHiddenNodes];
   for( UInt32 i = 0; i < m_unNumberOfHiddenNodes * m_unNumberOfHiddenNodes; i++ ) {
      m_pfHiddenToHiddenWeights[i] = params[unChromosomePosition++]*(m_cWeightsBounds.GetMax() - m_cWeightsBounds.GetMin() ) + m_cWeightsBounds.GetMin();
   }

   if( m_pfHiddenBiases == NULL ) m_pfHiddenBiases = new Real[m_unNumberOfHiddenNodes];
   for( UInt32 i = 0; i < m_unNumberOfHiddenNodes; i++ ) {
      m_pfHiddenBiases[i] = params[unChromosomePosition++]*(m_cBiasesBounds.GetMax() - m_cBiasesBounds.GetMin() ) + m_cBiasesBounds.GetMin();
   }

   if( m_pfHiddenTaus == NULL ) m_pfHiddenTaus = new Real[m_unNumberOfHiddenNodes];
   for( UInt32 i = 0; i < m_unNumberOfHiddenNodes; i++ ) {
      m_pfHiddenTaus[i] = pow(10, (m_cTausBounds.GetMin() + ((m_cTausBounds.GetMax()-m_cTausBounds.GetMin()) * (params[unChromosomePosition++]))));
   }

   if( m_pfHiddenToOutputWeights == NULL ) m_pfHiddenToOutputWeights = new Real[m_unNumberOfHiddenNodes * m_unNumberOfOutputs];
   for( UInt32 i = 0; i < m_unNumberOfHiddenNodes * m_unNumberOfOutputs; i++ ) {
      m_pfHiddenToOutputWeights[i] = params[unChromosomePosition++]*(m_cWeightsBounds.GetMax() - m_cWeightsBounds.GetMin() ) + m_cWeightsBounds.GetMin();
   }

   if( m_pfOutputBiases == NULL ) m_pfOutputBiases = new Real[m_unNumberOfOutputs];
   for( UInt32 i = 0; i < m_unNumberOfOutputs; i++ ) {
      m_pfOutputBiases[i] = params[unChromosomePosition++]*(m_cBiasesBounds.GetMax() - m_cBiasesBounds.GetMin() ) + m_cBiasesBounds.GetMin();
   }

   if( m_pfHiddenDeltaStates == NULL) m_pfHiddenDeltaStates  = new Real[m_unNumberOfHiddenNodes];
   if( m_pfHiddenStates == NULL ) m_pfHiddenStates = new Real[m_unNumberOfHiddenNodes];
   for( UInt32 i = 0; i < m_unNumberOfHiddenNodes; i++ ) {
      m_pfHiddenDeltaStates[i] = 0.0f;
      m_pfHiddenStates[i]      = 0.0f;
   }
}



/****************************************/
/****************************************/

void CCtrnnMultilayer::ComputeOutputs( void ) {
   // Update delta state of hidden layer from inputs:
   for( UInt32 i = 0; i < m_unNumberOfHiddenNodes; i++ ) {
      m_pfHiddenDeltaStates[i] = -m_pfHiddenStates[i];

      for( UInt32 j = 0; j < m_unNumberOfInputs; j++ ) {
         // weight * sigmoid(state)
         m_pfHiddenDeltaStates[i] += m_pfInputToHiddenWeights[i * m_unNumberOfInputs + j] * m_pfInputs[j] ;
      }

      // Update delta state from hidden layer, self-recurrent connections:
      for( UInt32 j = 0; j < m_unNumberOfHiddenNodes; j++ ) {
         Real z = (Real(1.0)/(exp(-( m_pfHiddenStates[j] + m_pfHiddenBiases[j])) + 1.0));
         m_pfHiddenDeltaStates[i] += m_pfHiddenToHiddenWeights[i * m_unNumberOfHiddenNodes + j] * z;
      }
   }


   // once all delta state are computed, get the new activation for the hidden unit
   for( UInt32  i = 0; i < m_unNumberOfHiddenNodes; i++ ) {
      m_pfHiddenStates[i] += m_pfHiddenDeltaStates[i] * m_fTimeStep/m_pfHiddenTaus[i];
   }

   // Update the outputs layer::
   for( UInt32 i = 0; i < m_unNumberOfOutputs; i++ ) {

      // Initialise to 0
      m_pfOutputs[i] = 0.0f;

      // Sum over all the hidden nodes
      for( UInt32 j = 0; j < m_unNumberOfHiddenNodes; j++ ) {
         Real z = (Real(1.0)/( exp(-( m_pfHiddenStates[j] + m_pfHiddenBiases[j])) + 1.0 ));
         m_pfOutputs[i] += m_pfHiddenToOutputWeights[i * m_unNumberOfHiddenNodes + j] * z;
      }

      // Compute the activation function immediately, since this is
      // what we return and since the output layer is not recurrent:
      m_pfOutputs[i] = (Real(1.0)/( exp(-( m_pfOutputs[i] + m_pfOutputBiases[i])) + 1.0 ));
   }
}

