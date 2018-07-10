/*
 * Plato_RapidOptimizationLibraryInterface.hpp
 *
 *  Created on: Feb 8, 2018
 */

#ifndef PLATO_RAPIDOPTIMIZATIONLIBRARYINTERFACE_HPP_
#define PLATO_RAPIDOPTIMIZATIONLIBRARYINTERFACE_HPP_

#include <mpi.h>

#include <string>
#include <memory>
#include <sstream>
#include <cassert>
#include <fstream>

#include "Teuchos_XMLParameterListHelpers.hpp"

#include "ROL_Bounds.hpp"
#include "ROL_OptimizationSolver.hpp"
#include "ROL_OptimizationProblem.hpp"

#include "Plato_Parser.hpp"
#include "Plato_Interface.hpp"
#include "Plato_SerialVectorROL.hpp"
#include "Plato_OptimizerUtilities.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_ReducedObjectiveROL.hpp"
#include "Plato_ReducedConstraintROL.hpp"
#include "Plato_DistributedVectorROL.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class RapidOptimizationLibraryInterface : public Plato::OptimizerInterface<ScalarType, OrdinalType>
{
public:
    /******************************************************************************/
    RapidOptimizationLibraryInterface(Plato::Interface* aInterface, const MPI_Comm & aComm) :
            mComm(aComm),
            mInterface(aInterface),
            mInputData(Plato::OptimizerEngineStageData())
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    virtual ~RapidOptimizationLibraryInterface()
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    Plato::optimizer::algorithm_t type() const
    /******************************************************************************/
    {
        return (Plato::optimizer::algorithm_t::RAPID_OPTIMIZATION_LIBRARY);
    }

    /******************************************************************************/
    void initialize()
    /******************************************************************************/
    {
        Plato::initialize<ScalarType, OrdinalType>(mInterface, mInputData);
    }

    /******************************************************************************/
    void optimize()
    /******************************************************************************/
    {
        mInterface->handleExceptions();

        this->initialize();

        /****************************** GET NUMBER OF DESIGN VARIABLES ******************************/
        const size_t tMY_CONTROL_INDEX = 0;
        std::string tMyName = mInputData.getControlName(tMY_CONTROL_INDEX);
        const OrdinalType tMyNumControls = mInterface->size(tMyName);

        /************************************ SET CONTROL BOUNDS ************************************/
        Teuchos::RCP<Plato::DistributedVectorROL<ScalarType>> tControlLowerBounds =
                Teuchos::rcp(new Plato::DistributedVectorROL<ScalarType>(mComm, tMyNumControls));
        std::vector<ScalarType> tLowerBounds = mInputData.getLowerBoundValues();
        this->setBounds(tLowerBounds, tControlLowerBounds.operator*());

        Teuchos::RCP<Plato::DistributedVectorROL<ScalarType>> tControlUpperBounds =
                Teuchos::rcp(new Plato::DistributedVectorROL<ScalarType>(mComm, tMyNumControls));
        std::vector<ScalarType> tUpperBounds = mInputData.getUpperBoundValues();
        this->setBounds(tUpperBounds, tControlUpperBounds.operator*());
        Teuchos::RCP<ROL::BoundConstraint<ScalarType>> tControlBoundsMng =
                Teuchos::rcp(new ROL::Bounds<ScalarType>(tControlLowerBounds, tControlUpperBounds));

        /******************************** SET CONTROL INITIAL GUESS *********************************/
        Teuchos::RCP<Plato::DistributedVectorROL<ScalarType>> tControls =
                Teuchos::rcp(new Plato::DistributedVectorROL<ScalarType>(mComm, tMyNumControls));
        this->setInitialGuess(tMyName, tControls.operator*());

        /****************************** SET DUAL VECTOR *******************************/
        const OrdinalType tNumConstraints = mInputData.getNumConstraints();
        Teuchos::RCP<Plato::SerialVectorROL<ScalarType>> tDual = Teuchos::rcp(new Plato::SerialVectorROL<ScalarType>(tNumConstraints));

        /********************************* SET OPTIMIZATION PROBLEM *********************************/
        Teuchos::RCP<ROL::Objective<ScalarType>> tObjective = Teuchos::rcp(new Plato::ReducedObjectiveROL<ScalarType>(mInputData, mInterface));
        Teuchos::RCP<ROL::Constraint<ScalarType>> tInequality = Teuchos::rcp(new Plato::ReducedConstraintROL<ScalarType>(mInputData, mInterface));
        ROL::OptimizationProblem<ScalarType> tOptimizationProblem(tObjective, tControls, tControlBoundsMng, tInequality, tDual);

        /******************************** SOLVE OPTIMIZATION PROBLEM ********************************/
        this->solve(tOptimizationProblem);

        this->finalize();
    }

    void finalize()
    {
        mInterface->getStage("Terminate");
    }

private:
    /******************************************************************************/
    void output(const std::stringbuf & aBuffer)
    /******************************************************************************/
    {
        int tMyRank = -1;
        MPI_Comm_rank(mComm, &tMyRank);
        assert(tMyRank >= static_cast<int>(0));
        if(tMyRank == static_cast<int>(0))
        {
            const bool tOutputDiagnosticsToFile = mInputData.getOutputDiagnosticsToFile();
            if(tOutputDiagnosticsToFile == false)
            {
                std::cout << aBuffer.str().c_str() << std::flush;
            }
            else
            {
                std::ofstream tOutputFile;
                tOutputFile.open("ROL_output.txt");
                tOutputFile << aBuffer.str().c_str();
                tOutputFile.close();
            }
        }
    }

    /******************************************************************************/
    void printControl(ROL::OptimizationProblem<ScalarType> & aOptimizationProblem)
    /******************************************************************************/
    {
        int tMyRank = -1;
        MPI_Comm_rank(mComm, &tMyRank);
        assert(tMyRank >= static_cast<int>(0));
        if(tMyRank == static_cast<int>(0))
        {
            const bool tOutputControlToFile = mInputData.getOutputControlToFile();
            if(tOutputControlToFile == true)
            {
                std::ofstream tOutputFile;
                tOutputFile.open("ROL_control_output.txt");
                ROL::Ptr<ROL::Vector<ScalarType>> tSolutionPtr = aOptimizationProblem.getSolutionVector();
                Plato::DistributedVectorROL<ScalarType> & tSolution =
                        dynamic_cast<Plato::DistributedVectorROL<ScalarType>&>(tSolutionPtr.operator*());
                std::vector<ScalarType> & tData = tSolution.vector();
                for(OrdinalType tIndex = 0; tIndex < tData.size(); tIndex++)
                {
                    tOutputFile << tData[tIndex] << "\n";
                }
                tOutputFile.close();
            }
        }
    }

    /******************************************************************************/
    void solve(ROL::OptimizationProblem<ScalarType> & aOptimizationProblem)
    /******************************************************************************/
    {
        std::stringbuf tBuffer;
        std::ostream tOutputStream(&tBuffer);
        std::string tFileName = mInputData.getInputFileName();
        Teuchos::RCP<Teuchos::ParameterList> tParameterList = Teuchos::rcp(new Teuchos::ParameterList);
        Teuchos::updateParametersFromXmlFile(tFileName, tParameterList.ptr());
        if(mInputData.getCheckGradient() == true)
        {
            /**************************** CHECK DERIVATIVES ****************************/
            aOptimizationProblem.check(tOutputStream);
        }
        else
        {
            /************************ SOLVE OPTIMIZATION PROBLEM ***********************/
            ROL::OptimizationSolver<ScalarType> tOptimizer(aOptimizationProblem, *tParameterList);
            tOptimizer.solve(tOutputStream);
        }

        // ********* Print Diagnostics and Control ********* //
        this->output(tBuffer);
        this->printControl(aOptimizationProblem);
    }

    /******************************************************************************/
    void setBounds(const std::vector<ScalarType> & aInputs, Plato::DistributedVectorROL<ScalarType> & aBounds)
    /******************************************************************************/
    {
        assert(aInputs.empty() == false);
        if(aInputs.size() == static_cast<size_t>(1))
        {
            const ScalarType tValue = aInputs[0];
            aBounds.fill(tValue);
        }
        else
        {
            assert(aInputs.size() == static_cast<size_t>(aBounds.dimension()));
            aBounds.set(aInputs);
        }
    }

    /******************************************************************************/
    void setInitialGuess(const std::string & aMyName, Plato::DistributedVectorROL<ScalarType> & aControl)
    /******************************************************************************/
    {
        std::string tInitializationStageName = mInputData.getInitializationStageName();
        if(tInitializationStageName.empty() == false)
        {
            // Use user-defined stage to compute initial guess
            Teuchos::ParameterList tPlatoInitializationStageParameterList;
            tPlatoInitializationStageParameterList.set(aMyName, aControl.vector().data());
            mInterface->compute(tInitializationStageName, tPlatoInitializationStageParameterList);
        }
        else
        {
            // Use user-defined values to compute initial guess. Hence, a stage was not defined by the user.
            std::vector<ScalarType> tInitialGuess = mInputData.getInitialGuess();
            assert(tInitialGuess.empty() == false);
            if(tInitialGuess.size() == static_cast<size_t>(1))
            {
                const ScalarType tValue = tInitialGuess[0];
                aControl.fill(tValue);
            }
            else
            {
                assert(tInitialGuess.size() == static_cast<size_t>(aControl.dimension()));
                aControl.set(tInitialGuess);
            }
        }
    }

public:
    MPI_Comm mComm;
    Plato::Interface* mInterface;
    Plato::OptimizerEngineStageData mInputData;

private:
    RapidOptimizationLibraryInterface(const Plato::RapidOptimizationLibraryInterface<ScalarType> & aRhs);
    Plato::RapidOptimizationLibraryInterface<ScalarType> & operator=(const Plato::RapidOptimizationLibraryInterface<ScalarType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_RAPIDOPTIMIZATIONLIBRARYINTERFACE_HPP_ */
