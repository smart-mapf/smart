#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include <argos3/core/control_interface/ci_controller.h>

using namespace argos;

class CNeuralNetwork {

public:

   CNeuralNetwork();
   virtual ~CNeuralNetwork();

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void Destroy();

   virtual void LoadNetworkParameters(const std::string& str_filename) = 0;
   virtual void LoadNetworkParameters(const UInt32 un_num_params,
                                      const Real* pf_params) = 0;
   virtual void ComputeOutputs() = 0;

   inline  UInt32 GetNumberOfInputs() {
      return m_unNumberOfInputs;
   }
   inline  void SetNumberOfInputs(UInt32 un_inputs) {
      m_unNumberOfInputs = un_inputs;
   }

   inline const Real* GetInputs() {
      return m_pfInputs;
   }
   void SetInput(UInt32 un_input_num,
                 Real f_input_value) {
      m_pfInputs[un_input_num] = f_input_value;
   }

   void SetInputRange(UInt32 un_input_start,
                      UInt32 un_num_values,
                      Real* pf_input_values ) {
      ::memcpy(m_pfInputs + un_input_start,
               pf_input_values,
               un_num_values);
   }

   inline  UInt32 GetNumberOfOutputs() {
      return m_unNumberOfOutputs;
   }
   inline  void SetNumberOfOutputs(UInt32 un_outputs) {
      m_unNumberOfOutputs = un_outputs;
   }

   inline  const Real* GetOutputs() {
      return m_pfOutputs;
   }
   inline  Real GetOutput(UInt32 un_num_output) {
      return m_pfOutputs[un_num_output];
   }

   virtual void SetOnlineParameters(const UInt32 un_num_params,
                                    const Real* pf_params);

protected:

   UInt32 m_unNumberOfInputs;
   UInt32 m_unNumberOfOutputs;

   Real* m_pfInputs;
   Real* m_pfOutputs;

   std::string m_strParameterFile;

};

#endif
