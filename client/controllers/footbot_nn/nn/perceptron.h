#ifndef PERCEPTRON_H
#define PERCEPTRON_H

#include "neural_network.h"

class CPerceptron : public CNeuralNetwork {

public:

   CPerceptron();
   virtual ~CPerceptron();
  
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Destroy();

   virtual void LoadNetworkParameters(const std::string& str_filename );
   virtual void LoadNetworkParameters(const UInt32 un_num_params,
                                      const Real* pf_params );
   virtual void ComputeOutputs();  

private:

   UInt32   m_unNumberOfWeights;
   Real*    m_pfWeights;
  
};

#endif
