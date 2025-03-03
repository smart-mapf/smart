#include "mpga_loop_functions.h"

CMPGALoopFunctions::CMPGALoopFunctions() :
   m_unTrial(0) {}

UInt32 CMPGALoopFunctions::GetTrial() const {
   return m_unTrial;
}

void CMPGALoopFunctions::SetTrial(UInt32 un_trial) {
   m_unTrial = un_trial;
}
