#include "mpga.h"
#include <cstdio>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <argos3/core/simulator/simulator.h>
#include "mpga_loop_functions.h"

static const std::string SHARED_MEMORY_FILE = "/MPGA_SHARED_MEMORY_" + ToString(getpid());

bool SortHighToLow(const CMPGA::SIndividual* pc_a,
               const CMPGA::SIndividual* pc_b) {
   return pc_a->Score > pc_b->Score;
}

bool SortLowToHigh(const CMPGA::SIndividual* pc_a,
               const CMPGA::SIndividual* pc_b) {
   return pc_b->Score > pc_a->Score;
}

CMPGA::CMPGA(const CRange<Real>& c_allele_range,
          UInt32 un_genome_size,
          UInt32 un_pop_size,
          Real f_mutation_prob,
          UInt32 un_num_trials,
          UInt32 un_generations,
          bool b_maximize,
          const std::string& str_argosconf,
          TScoreAggregator t_score_aggregator,
          UInt32 un_random_seed) :
   m_unCurrentGeneration(0),
   m_cAlleleRange(c_allele_range),
   m_unGenomeSize(un_genome_size),
   m_unPopSize(un_pop_size),
   m_fMutationProb(f_mutation_prob),
   m_unNumTrials(un_num_trials),
   m_unGenerations(un_generations),
   m_strARGoSConf(str_argosconf),
   m_tScoreAggregator(t_score_aggregator),
   MasterPID(::getpid()),
   m_cIndComparator(b_maximize ? SortHighToLow : SortLowToHigh) {
   m_pcSharedMem = new CSharedMem(un_genome_size,
                          un_pop_size);
   for(UInt32 i = 0; i < m_unPopSize; ++i) {
     SlavePIDs.push_back(::fork());
     if(SlavePIDs.back() == 0) {
       LaunchARGoS(i);
     }
   }
   CRandom::CreateCategory("ga", un_random_seed);
   m_pcRNG = CRandom::CreateRNG("ga");
   SIndividual* psInd;
   for(size_t p = 0; p < m_unPopSize; ++p) {
     psInd = new SIndividual;
     psInd->Score = -1.0;
     for(size_t g = 0; g < m_unGenomeSize; ++g) {
       psInd->Genome.push_back(m_pcRNG->Uniform(m_cAlleleRange));
     }
     m_tPopulation.push_back(psInd);
   }
   ::sleep(3);
}

CMPGA::~CMPGA() {
   for(UInt32 i = 0; i < m_unPopSize; ++i) {
     ::kill(SlavePIDs[i], SIGTERM);
   }
   while(!m_tPopulation.empty()) {
     delete m_tPopulation.back();
     m_tPopulation.pop_back();
   }
   CRandom::RemoveCategory("ga");
   Cleanup();
}

const CMPGA::TPopulation& CMPGA::GetPopulation() const {
   return m_tPopulation;
}

UInt32 CMPGA::GetGeneration() const {
   return m_unCurrentGeneration;
}

void CMPGA::Cleanup() {
   delete m_pcSharedMem;
}

void CMPGA::Evaluate() {
   for(UInt32 i = 0; i < m_unPopSize; ++i) {
     m_pcSharedMem->SetGenome(i, &(m_tPopulation[i]->Genome[0]));
     ::kill(SlavePIDs[i], SIGCONT);
   }
   UInt32 unTrialsLeft = m_unPopSize;
   int nSlaveInfo;
   pid_t tSlavePID;
   while(unTrialsLeft > 0) {
     tSlavePID = ::waitpid(-1, &nSlaveInfo, WUNTRACED);
     if(!WIFSTOPPED(nSlaveInfo)) {
       LOGERR << "[FATAL] Slave process with PID " << tSlavePID << " exited, can't continue. Check file ARGoS_LOGERR_" << tSlavePID << " for more information." << std::endl;
       LOG.Flush();
       LOGERR.Flush();
       Cleanup();
       ::exit(1);
     }
     --unTrialsLeft;
   }
   for(UInt32 i = 0; i < m_unPopSize; ++i) {
     m_tPopulation[i]->Score = m_pcSharedMem->GetScore(i);
   }
   std::sort(m_tPopulation.begin(),
          m_tPopulation.end(),
          m_cIndComparator);
}

void CMPGA::NextGen() {
   ++m_unCurrentGeneration;
   Selection();
   Crossover();
   Mutation();
}

bool CMPGA::Done() const {
   return m_unCurrentGeneration >= m_unGenerations;
}

static CMPGA* GA_INSTANCE;

void SlaveHandleSIGTERM(int) {
   argos::CSimulator::GetInstance().Destroy();
   argos::LOG.Flush();
   argos::LOGERR.Flush();
   GA_INSTANCE->Cleanup();
}

void CMPGA::LaunchARGoS(UInt32 un_slave_id) {
   GA_INSTANCE = this;
   ::signal(SIGTERM, SlaveHandleSIGTERM);
   argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   try {
     cSimulator.SetExperimentFileName(m_strARGoSConf);
     cSimulator.LoadExperiment();
     LOG.Flush();
     LOGERR.Flush();
   }
   catch(CARGoSException& ex) {
     LOGERR << ex.what() << std::endl;
     ::raise(SIGTERM);
   }
   CMPGALoopFunctions& cLoopFunctions = dynamic_cast<CMPGALoopFunctions&>(cSimulator.GetLoopFunctions());
   std::vector<Real> vecScores(m_unNumTrials, 0.0);
   while(1) {
     ::raise(SIGTSTP);
     cLoopFunctions.ConfigureFromGenome(m_pcSharedMem->GetGenome(un_slave_id));
     for(size_t i = 0; i < m_unNumTrials; ++i) {
       cLoopFunctions.SetTrial(i);
       cSimulator.Reset();
       cSimulator.Execute();
       vecScores[i] = cLoopFunctions.Score();
       LOG.Flush();
       LOGERR.Flush();
     }
     ;
     m_pcSharedMem->SetScore(un_slave_id, m_tScoreAggregator(vecScores));
   }
}

void CMPGA::Selection() {
   while(m_tPopulation.size() > 2) {
     delete m_tPopulation.back();
     m_tPopulation.pop_back();
   }
}

void CMPGA::Crossover() {
   SIndividual* psParent1 = m_tPopulation[0];
   SIndividual* psParent2 = m_tPopulation[1];
   UInt32 unCut;
   SIndividual* psInd;
   for(UInt32 i = 2; i < m_unPopSize; ++i) {
     unCut = m_pcRNG->Uniform(CRange<UInt32>(1, m_unGenomeSize-1));
     psInd = new SIndividual;
     for(UInt32 j = 0; j < unCut; ++j) {
       psInd->Genome.push_back(psParent1->Genome[j]);
     }
     for(UInt32 j = unCut; j < m_unGenomeSize; ++j) {
       psInd->Genome.push_back(psParent2->Genome[j]);
     }
     m_tPopulation.push_back(psInd);
   }
}

void CMPGA::Mutation() {
   for(UInt32 i = 2; i < m_unPopSize; ++i) {
     for(UInt32 a = 0; a < m_unGenomeSize; ++a) {
       if(m_pcRNG->Bernoulli(m_fMutationProb))
         m_tPopulation[i]->Genome[a] = m_pcRNG->Uniform(m_cAlleleRange);
     }
   }
}

CMPGA::CSharedMem::CSharedMem(UInt32 un_genome_size,
                       UInt32 un_pop_size) :
   m_unGenomeSize(un_genome_size),
   m_unPopSize(un_pop_size) {
   m_nSharedMemFD = ::shm_open(SHARED_MEMORY_FILE.c_str(),
                        O_RDWR | O_CREAT,
                        S_IRUSR | S_IWUSR);
   if(m_nSharedMemFD < 0) {
     ::perror(SHARED_MEMORY_FILE.c_str());
     exit(1);
   }
   size_t unShareMemSize = m_unPopSize * (m_unGenomeSize+1) * sizeof(Real);
   ::ftruncate(m_nSharedMemFD, unShareMemSize);
   m_pfSharedMem = reinterpret_cast<Real*>(
     ::mmap(NULL,
          unShareMemSize,
          PROT_READ | PROT_WRITE,
          MAP_SHARED,
          m_nSharedMemFD,
          0));
   if(m_pfSharedMem == MAP_FAILED) {
     ::perror("shared memory");
     exit(1);
   }
}

CMPGA::CSharedMem::~CSharedMem() {
   munmap(m_pfSharedMem, m_unPopSize * (m_unGenomeSize+1) * sizeof(Real));
   close(m_nSharedMemFD);
   shm_unlink(SHARED_MEMORY_FILE.c_str());
}

Real* CMPGA::CSharedMem::GetGenome(UInt32 un_individual) {
   return m_pfSharedMem + un_individual * (m_unGenomeSize+1);
}

void CMPGA::CSharedMem::SetGenome(UInt32 un_individual,
                          const Real* pf_genome) {
   ::memcpy(m_pfSharedMem + un_individual * (m_unGenomeSize+1),
         pf_genome,
         m_unGenomeSize * sizeof(Real));
}

Real CMPGA::CSharedMem::GetScore(UInt32 un_individual) {
   return m_pfSharedMem[un_individual * (m_unGenomeSize+1) + m_unGenomeSize];
}

void CMPGA::CSharedMem::SetScore(UInt32 un_individual,
                         Real f_score) {
   m_pfSharedMem[un_individual * (m_unGenomeSize+1) + m_unGenomeSize] = f_score;
}
