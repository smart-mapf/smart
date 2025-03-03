#include <iostream>
#include <fstream>
#include <loop_functions/mpga_loop_functions/mpga.h>
#include <loop_functions/mpga_loop_functions/mpga_phototaxis_loop_functions.h>

void FlushIndividual(const CMPGA::SIndividual& s_ind,
                UInt32 un_generation) {
   std::ostringstream cOSS;
   cOSS << "best_" << un_generation << ".dat";
   std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
   cOFS << GENOME_SIZE;
   for(UInt32 i = 0; i < GENOME_SIZE; ++i) {
     cOFS << " " << s_ind.Genome[i];
   }
   cOFS << std::endl;
}

Real ScoreAggregator(const std::vector<Real>& vec_scores) {
   Real fScore = vec_scores[0];
   for(size_t i = 1; i < vec_scores.size(); ++i) {
     fScore = Max(fScore, vec_scores[i]);
   }
   return fScore;
}

int main() {
   CMPGA cGA(CRange<Real>(-10.0,10.0),
          GENOME_SIZE,
          5,
          0.05,
          5,
          100,
          false,
          "experiments/mpga.argos",
          &ScoreAggregator,
          12345
     );
   cGA.Evaluate();
   argos::LOG << "Generation #" << cGA.GetGeneration() << "...";
   argos::LOG << " scores:";
   for(UInt32 i = 0; i < cGA.GetPopulation().size(); ++i) {
     argos::LOG << " " << cGA.GetPopulation()[i]->Score;
   }
   LOG << std::endl;
   LOG.Flush();
   while(!cGA.Done()) {
     cGA.NextGen();
     cGA.Evaluate();
     argos::LOG << "Generation #" << cGA.GetGeneration() << "...";
     argos::LOG << " scores:";
     for(UInt32 i = 0; i < cGA.GetPopulation().size(); ++i) {
       argos::LOG << " " << cGA.GetPopulation()[i]->Score;
     }
     if(cGA.GetGeneration() % 5 == 0) {
       argos::LOG << " [Flushing genome... ";
       FlushIndividual(*cGA.GetPopulation()[0],
                   cGA.GetGeneration());
       argos::LOG << "done.]";
     }
     LOG << std::endl;
     LOG.Flush();
   }
   return 0;
}
