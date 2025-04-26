#ifndef MPGA_LOOP_FUNCTIONS_H
#define MPGA_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

using namespace argos;


class CMPGALoopFunctions : public CLoopFunctions {
   
public:

   CMPGALoopFunctions();

   /**
    * Class destructor.
    */
   virtual ~CMPGALoopFunctions() {}

   /**
    * Configures the trial using the passed genome.
    * @param pf_genome The genome.
    */
   virtual void ConfigureFromGenome(const Real* pf_genome) = 0;

   /**
    * Returns the current trial.
    */
   virtual UInt32 GetTrial() const;

   /**
    * Sets the current trial.
    * @param un_trial The trial number.
    */
   virtual void SetTrial(UInt32 un_trial);

   /**
    * Returns the score associated to the current trial.
    */
   virtual Real Score() = 0;

private:

   UInt32 m_unTrial;

};

#endif
