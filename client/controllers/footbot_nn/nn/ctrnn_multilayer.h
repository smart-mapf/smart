#ifndef CTRNN_MULTILAYER_H
#define CTRNN_MULTILAYER_H

#include "neural_network.h"
#include <argos3/core/utility/math/range.h>

class CCtrnnMultilayer : public CNeuralNetwork {

public:

   CCtrnnMultilayer();
   virtual ~CCtrnnMultilayer();

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();

   virtual void LoadNetworkParameters(const std::string& str_filename);
   virtual void LoadNetworkParameters(const UInt32 un_num_params,
                                      const Real* pf_params );
   virtual void ComputeOutputs();


   inline  UInt32      GetNumberOfHiddenNodes() { return m_unNumberOfHiddenNodes; }
   inline  const Real* GetHiddenStates()        { return m_pfHiddenStates;        }
   inline  const Real* GetHiddenTaus()          { return m_pfHiddenTaus;          }
   inline  const Real* GetHiddenBias()          { return m_pfHiddenBiases;        }
   inline  const Real* GetOutputBias()          { return m_pfOutputBiases;        }

protected:

   Real* m_pfInputToHiddenWeights;

   Real* m_pfHiddenBiases;
   Real* m_pfHiddenTaus;
   Real* m_pfHiddenDeltaStates;
   Real* m_pfHiddenStates;

   Real* m_pfHiddenToHiddenWeights;
   Real* m_pfHiddenToOutputWeights;

   Real* m_pfOutputBiases;
   Real* m_pfOutputTaus;

   UInt32 m_unNumberOfHiddenNodes;
   Real m_fTimeStep;

   CRange<Real> m_cWeightsBounds;
   CRange<Real> m_cBiasesBounds;
   CRange<Real> m_cTausBounds;

};

#endif
