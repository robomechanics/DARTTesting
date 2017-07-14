#include "cmaes.h"
#include <iostream>

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "MinitaurWorldNode.hpp"
#include "MinitaurEventHandler.hpp"
#include "MinitaurWidget.hpp"

#include "MyWindow.hpp"
#include "computeCost.h"


#include <stdlib.h>
#include "cmaes.h"

/**
 * The objective (fitness) function to be minized. "cigtab" function.
 */
double fitfun(double const *x, int N)
{
  double cost = computeCost(x, N);
  std::cout << "CoT: " << cost << std::endl;

  return cost;
}

/**
 * The optimization loop.
 */
int main(int, char**) {
  CMAES<double> evo;
  double *arFunvals, *const*pop, *xfinal;

  // Initialize everything
  const int dim = 22;
  double xstart[dim];
  for(int i=0; i<dim; i++) xstart[i] = 0.5;
  double stddev[dim];
  for(int i=0; i<dim; i++) stddev[i] = 0.3;
  Parameters<double> parameters;
  // TODO Adjust parameters here
  parameters.init(dim, xstart, stddev);
  arFunvals = evo.init(parameters);

  std::cout << evo.sayHello() << std::endl;

  // Iterate until stop criterion holds
  while(!evo.testForTermination())
  {
    // Generate lambda new search points, sample population
    pop = evo.samplePopulation(); // Do not change content of pop

    /* Here you may resample each solution point pop[i] until it
       becomes feasible, e.g. for box constraints (variable
       boundaries). function is_feasible(...) needs to be
       user-defined.  
       Assumptions: the feasible domain is convex, the optimum is
       not on (or very close to) the domain boundary, initialX is
       feasible and initialStandardDeviations are sufficiently small
       to prevent quasi-infinite looping.
    */
    /* for (i = 0; i < evo.get(CMAES<double>::PopSize); ++i)
         while (!is_feasible(pop[i]))
           evo.reSampleSingle(i);
    */

    // evaluate the new search points using fitfun from above
    for (int i = 0; i < evo.get(CMAES<double>::Lambda); ++i)
      arFunvals[i] = fitfun(pop[i], (int) evo.get(CMAES<double>::Dimension));

    // update the search distribution used for sampleDistribution()
    evo.updateDistribution(arFunvals);
  }
  std::cout << "Stop:" << std::endl << evo.getStopMessage();
  evo.writeToFile(CMAES<double>::WKResume, "resumeevo1.dat"); // write resumable state of CMA-ES

  // get best estimator for the optimum, xmean
  xfinal = evo.getNew(CMAES<double>::XMean); // "XBestEver" might be used as well

  // do something with final solution and finally release memory
  delete[] xfinal;

  return 0;
}

// using namespace libcmaes;

// FitFunc minitaurCoT = [](const double *x, const int N)
// {
//   double cost = computeCost(x, N);
//   std::cout << "CoT: " << cost << std::endl;

//   return cost;
// };

// int main(int argc, char *argv[])
// {
  
//   int dim = 4; // problem dimensions.
//   std::vector<double> x0(dim,10.0);
//   double sigma = 0.1;
//   //int lambda = 100; // offsprings at each generation.
//   CMAParameters<> cmaparams(x0,sigma);
//   //cmaparams.set_algo(BIPOP_CMAES);
//   CMASolutions cmasols = cmaes<>(minitaurCoT,cmaparams);
//   std::cout << "best solution: " << cmasols << std::endl;
//   std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
//   return cmasols.run_status();  

//   return 0;
// }