#ifndef MPGA_H
#define MPGA_H

#include <vector>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;
class CMPGA {
   
public:

   /**
    * Information about an individual.
    */
   struct SIndividual {
      std::vector<Real> Genome;
      Real Score;
   };

   /** A population is a list of parameter sets */
   typedef std::vector<SIndividual*> TPopulation;

   /** Score aggregator function pointer */
   typedef Real (*TScoreAggregator)(const std::vector<Real>&);

public:

   /**
    * Class constructor.
    * @param c_allele_range The range of each allele of the genome.
    * @param un_genome_size The size of the genome of an individual.
    * @param un_pop_size The size of the population.
    * @param f_mutation_prob The mutation probability.
    * @param un_num_trials The number of trials per individual.
    * @param un_generations The max number of generations.
    * @param b_maximize true to maximize, false to minimize.
    * @param str_argosconf Path of the .argos file to use.
    * @param t_score_aggregator The function used to aggregate the trial scores.
    * @param un_random_seed The random seed.
    */
   CMPGA(const CRange<Real>& c_allele_range,
         UInt32 un_genome_size,
         UInt32 un_pop_size,
         Real f_mutation_prob,
         UInt32 un_num_trials,
         UInt32 un_generations,
         bool b_maximize,
         const std::string& str_argosconf,
         TScoreAggregator t_score_aggregator,
         UInt32 un_random_seed);

   /**
    * Class destructor.
    */
   ~CMPGA();

   /**
    * Returns the current population.
    */
   const TPopulation& GetPopulation() const;

   /**
    * Returns the current generation.
    */
   UInt32 GetGeneration() const;

   /**
    * Tidies up memory and close files.
    * Used internally, don't call it from user code.
    */
   virtual void Cleanup();

   /**
    * Runs the trials to evaluate the current population.
    */
   virtual void Evaluate();

   /**
    * Executes the next generation.
    */
   virtual void NextGen();

   /**
    * Returns true if the evolution is finished, false otherwise.
    */
   virtual bool Done() const;

private:

   /** Executes the slave process that manages ARGoS */
   virtual void LaunchARGoS(UInt32 un_slave_id);

   /**
    * Discards all the individuals apart from the top two.
    */
   virtual void Selection();

   /**
    * Performs crossover among the best individuals.
    */
   virtual void Crossover();

   /**
    * Performs mutation 
    */
   virtual void Mutation();

private:

   /** Shared memory manager for data exchange between master and slaves */
   class CSharedMem {
      
   public:

      /**
       * Class constructor.
       * @param un_genome_size The size of the genome of an individual.
       * @param un_pop_size The size of the population.
       */
      CSharedMem(UInt32 un_genome_size,
                 UInt32 un_pop_size);

      /**
       * Class destructor.
       */
      ~CSharedMem();

      /**
       * Returns the genome of an individual.
       * @param un_individual The individual.
       */
      Real* GetGenome(UInt32 un_individual);

      /**
       * Sets the genome of an individual.
       * @param un_individual The individual.
       * @param pf_genome The genome.
       */
      void SetGenome(UInt32 un_individual,
                     const Real* pf_genome);
      
      /**
       * Returns the score of an individual.
       * @param un_individual The individual.
       */
      Real GetScore(UInt32 un_individual);
      
      /**
       * Sets the score of an individual.
       * @param un_individual The individual.
       * @param f_score The score.
       */
      void SetScore(UInt32 un_individual,
                    Real f_score);

   private:
      
      /** Genome size */
      UInt32 m_unGenomeSize;
      
      /** Population size */
      UInt32 m_unPopSize;

      /** File descriptor for shared memory area */
      int m_nSharedMemFD;

      /** Pointer to the shared memory area */
      Real* m_pfSharedMem;

   };
   
protected:

   /** Current population */
   TPopulation m_tPopulation;

   /** Current generation */
   UInt32 m_unCurrentGeneration;

   /** The range of each allele in the genome */
   CRange<Real> m_cAlleleRange;

   /** Genome size */
   UInt32 m_unGenomeSize;

   /** Population size */
   UInt32 m_unPopSize;

   /** Mutation probability */
   Real m_fMutationProb;

   /** Number of trials per individual */
   UInt32 m_unNumTrials;

   /** Number of generations */
   UInt32 m_unGenerations;

   /** Path to the .argos file to use */
   std::string m_strARGoSConf;

   /** The function used to aggregate the scores */
   TScoreAggregator m_tScoreAggregator;

   /** PID of the master process */
   pid_t MasterPID;

   /** PIDs of the slave processes */
   std::vector<pid_t> SlavePIDs;

   /** The shared memory manager */
   CSharedMem* m_pcSharedMem;

   /** Random number generator */
   CRandom::CRNG* m_pcRNG;

   /** Comparison function to sort the population */
   bool (*m_cIndComparator)(const SIndividual*,
                            const SIndividual*);
};

#endif
