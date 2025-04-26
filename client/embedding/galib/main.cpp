#include <ga/ga.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>
#include <loop_functions/galib_phototaxis_loop_functions/galib_phototaxis_loop_functions.h>

float LaunchARGoS(GAGenome& c_genome) {
   GARealGenome& cRealGenome = dynamic_cast<GARealGenome&>(c_genome);
   static argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   static CGALibPhototaxisLoopFunctions& cLoopFunctions = dynamic_cast<CGALibPhototaxisLoopFunctions&>(cSimulator.GetLoopFunctions());
   Real fDistance = 0.0f;
   for(size_t i = 0; i < 5; ++i) {
      cLoopFunctions.SetTrial(i);
      cSimulator.Reset();
      cLoopFunctions.ConfigureFromGenome(cRealGenome);
      cSimulator.Execute();
      fDistance = Max(fDistance, cLoopFunctions.Performance());
   }
   return fDistance;
}

void FlushBest(const GARealGenome& c_genome, size_t un_generation) {
   std::ostringstream cOSS;
   cOSS << "best_" << un_generation << ".dat";
   std::ofstream cOFS(cOSS.str().c_str(), std::ios::out | std::ios::trunc);
   cOFS << GENOME_SIZE
        << " "
        << c_genome
        << std::endl;
}

int main(int argc, char** argv) {
   GAAlleleSet<float> cAlleleSet(-10.0f, 10.0f);
   GARealGenome cGenome(GENOME_SIZE, cAlleleSet, LaunchARGoS);
   GASimpleGA cGA(cGenome);
   cGA.minimize();
   cGA.populationSize(5);
   cGA.nGenerations(500);
   cGA.pMutation(0.05f);
   cGA.pCrossover(0.15f);
   cGA.scoreFilename("evolution.dat");
   cGA.flushFrequency(1);

   argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   cSimulator.SetExperimentFileName("experiments/galib.argos");
   cSimulator.LoadExperiment();

   cGA.initialize(12345);
   do {
      argos::LOG << "Generation #" << cGA.generation() << "...";
      cGA.step();
      argos::LOG << "done.";
      if(cGA.generation() % cGA.flushFrequency() == 0) {
         argos::LOG << "   Flushing...";
         cGA.flushScores();
         FlushBest(dynamic_cast<const GARealGenome&>(cGA.statistics().bestIndividual()), cGA.generation());
         argos::LOG << "done.";
      }
      LOG << std::endl;
      LOG.Flush();
   }
   while(! cGA.done());

   cSimulator.Destroy();

   return 0;
}
