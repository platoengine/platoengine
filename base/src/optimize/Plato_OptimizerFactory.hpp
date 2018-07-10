/*
 * Plato_Optimizer.hpp
 *
 *  Created on: April 19, 2017
 *
 */

#ifndef SRC_OPTIMIZERFACTORY_HPP_

#define SRC_OPTIMIZERFACTORY_HPP_

#include <mpi.h>

#include "Plato_OptimizerInterface.hpp"
#include "Plato_DiagnosticsInterface.hpp"
#include "Plato_OptimalityCriteriaInterface.hpp"
#include "Plato_MethodMovingAsymptotesInterface.hpp"
#include "Plato_KelleySachsBoundConstrainedInterface.hpp"
#include "Plato_KelleySachsAugmentedLagrangianInterface.hpp"
#include "Plato_GloballyConvergentMethodMovingAsymptotesInterface.hpp"

#ifdef ENABLE_ROL
#include "Plato_RapidOptimizationLibraryInterface.hpp"
#endif

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimizerFactory
{
public:
    OptimizerFactory()
    {
    }
    ~OptimizerFactory()
    {
    }

    /**********************************************************************/
    Plato::OptimizerInterface<ScalarType, OrdinalType>* create(Plato::Interface* aInterface, MPI_Comm aLocalComm)
    /**********************************************************************/
    {
     Plato::OptimizerInterface<ScalarType, OrdinalType>* tOptimizer = nullptr;
     try {
       auto tInputData = aInterface->getInputData();

       if( tInputData.size<Plato::InputData>("Optimizer") > 1 )
       {
         Plato::ParsingException tParsingException("Plato::OptimizerFactory: multiple 'Optimizer' definitions");
         aInterface->registerException(tParsingException);
       }
       if( tInputData.size<Plato::InputData>("Optimizer") == 0 )
       {
         Plato::ParsingException tParsingException("Plato::OptimizerFactory: missing 'Optimizer' definitions");
         aInterface->registerException(tParsingException);
       }
       auto tOptNode = tInputData.get<Plato::InputData>("Optimizer");
       std::string tOptPackage = Plato::Get::String(tOptNode, "Package");
       if( tOptPackage == "OC" )
       {
         try {
           tOptimizer = new Plato::OptimalityCriteriaInterface<ScalarType, OrdinalType>(aInterface, aLocalComm);
         } catch(...){aInterface->Catch();}
       }
       else if( tOptPackage == "GCMMA" )
       {
         try {
           tOptimizer = new Plato::GloballyConvergentMethodMovingAsymptotesInterface<ScalarType, OrdinalType>(aInterface, aLocalComm);
         } catch(...){aInterface->Catch();}
       }
       else if( tOptPackage == "MMA" )
       {
         try {
           tOptimizer = new Plato::MethodMovingAsymptotesInterface<ScalarType, OrdinalType>(aInterface, aLocalComm);
         } catch(...){aInterface->Catch();}
       }
       else if( tOptPackage == "KSUC" )
       {
         try {
           Plato::optimizer::algorithm_t tType = Plato::optimizer::algorithm_t::KELLEY_SACHS_UNCONSTRAINED;
           tOptimizer = new Plato::KelleySachsBoundConstrainedInterface<ScalarType, OrdinalType>(aInterface, aLocalComm, tType);
         } catch(...){aInterface->Catch();}
       }
       else if( tOptPackage == "KSBC" )
       {
         try {
           Plato::optimizer::algorithm_t tType = Plato::optimizer::algorithm_t::KELLEY_SACHS_BOUND_CONSTRAINED;
           tOptimizer = new Plato::KelleySachsBoundConstrainedInterface<ScalarType, OrdinalType>(aInterface, aLocalComm, tType);
         } catch(...){aInterface->Catch();}
       }
       else if( tOptPackage == "KSAL" )
       {
         try {
           tOptimizer = new Plato::KelleySachsAugmentedLagrangianInterface<ScalarType, OrdinalType>(aInterface, aLocalComm);
         } catch(...){aInterface->Catch();}
       }
#ifdef ENABLE_ROL
       else if( tOptPackage == "ROL" )
       {
         try {
           tOptimizer = new Plato::RapidOptimizationLibraryInterface<ScalarType, OrdinalType>(aInterface, aLocalComm);
         } catch(...){aInterface->Catch();}
       }
#endif
       else if( tOptPackage == "DerivativeChecker" )
       {
         try {
           tOptimizer = new Plato::DiagnosticsInterface<ScalarType, OrdinalType>(aInterface, aLocalComm);
         } catch(...){aInterface->Catch();}
       }
       else
       {
           std::stringstream tStringStream;
           tStringStream << "Plato::OptimizerFactory: " << tOptPackage << " Unknown." << std::endl
           << "Valid options are\n"
           << "\t OC ... Optimality Criteria\n"
           << "\t GCMMA ... Globally Convergent Method of Moving Asymptotes\n"
           << "\t MMA ... Method of Moving Asymptotes\n"
           << "\t KSUC ... Kelley Sachs Unconstrained\n"
           << "\t KSBC ... Kelley Sachs Bound Constrained\n"
           << "\t KSAL ... Kelley Sachs Augmented Lagrangian\n"
           << "\t DerivativeChecker ... Derivative Checker Toolkit\n"
#ifdef ENABLE_ROL
           << "\t ROL ... Rapid Optimization Library\n"
#endif
           << std::endl;
           throw Plato::ParsingException(tStringStream.str());
       }


     }
        catch(...)
        {
            aInterface->Catch();
            tOptimizer = nullptr;
        }

     return (tOptimizer);
   }

private:
    OptimizerFactory(const Plato::OptimizerFactory<ScalarType, OrdinalType>&);
    Plato::OptimizerFactory<ScalarType, OrdinalType> & operator=(const Plato::OptimizerFactory<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif 
