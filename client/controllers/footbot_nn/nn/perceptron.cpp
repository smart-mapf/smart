#include "perceptron.h"

#include <fstream>
#include <cmath>

CPerceptron::CPerceptron() :
   m_unNumberOfWeights(0),
   m_pfWeights(NULL) {}

CPerceptron::~CPerceptron() {
   if(m_pfWeights) delete[] m_pfWeights;
}

void CPerceptron::Init(TConfigurationNode& t_tree) {
   CNeuralNetwork::Init(t_tree);

   if( m_strParameterFile != "" ) {
      try{
         LoadNetworkParameters(m_strParameterFile);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("cannot load parameters from file.", ex);
      }
   }
}

void CPerceptron::Destroy() {
   if( m_pfWeights ) delete[] m_pfWeights;
   m_pfWeights = NULL;
   m_unNumberOfWeights = 0;
}

void CPerceptron::LoadNetworkParameters(const std::string& str_filename) {
   std::ifstream cIn(str_filename.c_str(), std::ios::in);
   if( !cIn ) {
      THROW_ARGOSEXCEPTION("Cannot open parameter file '" << str_filename << "' for reading");
   }

   UInt32 un_length = 0;
   if( !(cIn >> un_length) ) {
      THROW_ARGOSEXCEPTION("Cannot read data from file '" << str_filename << "'");
   }

   m_unNumberOfWeights = (m_unNumberOfInputs + 1) * m_unNumberOfOutputs;
   if( un_length != m_unNumberOfWeights ) {
      THROW_ARGOSEXCEPTION("Number of parameter mismatch: '"
                           << str_filename
                           << "' contains "
                           << un_length
                           << " parameters, while "
                           << m_unNumberOfWeights
                           << " were expected from the XML configuration file");
   }

   if(m_pfWeights == NULL) m_pfWeights = new Real[m_unNumberOfWeights];
   for(size_t i = 0; i < m_unNumberOfWeights; ++i) {
      if( !(cIn >> m_pfWeights[i] ) ) {
         THROW_ARGOSEXCEPTION("Cannot read data from file '" << str_filename << "'");
      }
   }
}

void CPerceptron::LoadNetworkParameters(const UInt32 un_num_params,
                                        const Real* pf_params) {
   m_unNumberOfWeights = (m_unNumberOfInputs + 1) * m_unNumberOfOutputs;
   if(un_num_params != m_unNumberOfWeights) {
      THROW_ARGOSEXCEPTION("Number of parameter mismatch: '"
                           << "passed "
                           << un_num_params
                           << " parameters, while "
                           << m_unNumberOfWeights
                           << " were expected from the XML configuration file");
   }

   if(m_pfWeights == NULL) m_pfWeights = new Real[m_unNumberOfWeights];
   for(size_t i = 0; i < m_unNumberOfWeights; ++i ) {
      m_pfWeights[i] = pf_params[i];
   }
}

void CPerceptron::ComputeOutputs() {
   for(size_t i = 0; i < m_unNumberOfOutputs; ++i) {
      m_pfOutputs[i] = m_pfWeights[i * (m_unNumberOfInputs + 1)];

      for(size_t j = 0; j < m_unNumberOfInputs; ++j) {
         size_t ji = i * (m_unNumberOfInputs + 1) + (j + 1);
         m_pfOutputs[i] += m_pfWeights[ji] * m_pfInputs[j];
      }

      m_pfOutputs[i] = 1.0f / ( 1.0f + ::exp( -m_pfOutputs[i]) );
   }
}
