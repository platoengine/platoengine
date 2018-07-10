/*
 * Plato_AlgebraFactory.hpp
 *
 *  Created on: Feb 21, 2018
 */

#ifndef PLATO_ALGEBRAFACTORY_HPP_
#define PLATO_ALGEBRAFACTORY_HPP_

#include <string>
#include <memory>
#include <sstream>

#include "Plato_Interface.hpp"
#include "Plato_KokkosVector.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_DistributedVector.hpp"
#include "Plato_KokkosReductionOperations.hpp"
#include "Plato_DistributedReductionOperations.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
void copy(const std::vector<ScalarType> & aIn, Plato::Vector<ScalarType, OrdinalType> & aOut)
{
    assert(aIn.size() == aOut.size());
    for(OrdinalType tIndex = 0; tIndex < aOut.size(); tIndex++)
    {
        aOut[tIndex] = aIn[tIndex];
    }
}

template<typename ScalarType, typename OrdinalType = size_t>
class AlgebraFactory
{
public:
    AlgebraFactory()
    {
    }
    ~AlgebraFactory()
    {
    }

    /********************************************************************************************/
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> createVector(const MPI_Comm & aComm,
                                                                         const OrdinalType & aLength,
                                                                         Plato::Interface* aInterface) const
    /********************************************************************************************/
    {
        std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tVector;
        try
        {
            auto tInputData = aInterface->getInputData();

            if( tInputData.size<Plato::Interface>("Optimizer") > 1 )
            {
                std::stringstream tMsg;
                tMsg << "\n\n ********** PLATO ERROR: FILE = " << __FILE__ << ", FUNCTION = " << __PRETTY_FUNCTION__
                        << ", LINE = " << __LINE__ << ", MESSAGE: MULTIPLE 'Optimizer' DEFINITIONS\n\n";
                Plato::ParsingException tParsingException(tMsg.str());
                aInterface->registerException(tParsingException);
            }
            auto tOptimizerNode = tInputData.get<Plato::InputData>("Optimizer");

            std::string tAlgebra;
            if( tOptimizerNode.size<Plato::InputData>("Options") )
            {
                auto tOptionsNode = tOptimizerNode.get<Plato::InputData>("Options");
                tAlgebra = Plato::Get::String(tOptionsNode, "Algebra");
            }

            if(tAlgebra == "Serial")
            {
                try
                {
                    tVector = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aLength);
                }
                catch(...)
                {
                    aInterface->Catch();
                }
            }
            else if(tAlgebra == "Distributed")
            {
                try
                {
                    tVector = std::make_shared<Plato::DistributedVector<ScalarType, OrdinalType>>(aComm, aLength);
                }
                catch(...)
                {
                    aInterface->Catch();
                }
            }
            else if(tAlgebra == "GPU")
            {
                try
                {
                    tVector = std::make_shared<Plato::KokkosVector<ScalarType, OrdinalType>>(aLength);
                }
                catch(...)
                {
                    aInterface->Catch();
                }
            }
            else if(tAlgebra == "")
            {
                /* DEFAULT LINEAR ALGEBRA = DISTRIBUTED */
                try
                {
                    tVector = std::make_shared<Plato::DistributedVector<ScalarType, OrdinalType>>(aComm, aLength);
                }
                catch(...)
                {
                    aInterface->Catch();
                }
            }
            else
            {
                std::stringstream tMsg;
                tMsg << "\n\n ********** PLATO ERROR: FILE = " << __FILE__ << ", FUNCTION = " << __PRETTY_FUNCTION__
                        << ", LINE = " << __LINE__ << ", MESSAGE: User Specified Algebra: " << tAlgebra << ", Unknown."
                        << std::endl << "Valid options are\n"
                        << "\t GPU ... Graphics Processing Units (GPU) Accelerated Algebra\n"
                        << "\t Serial ... Serial Algebra\n" << "\t Distributed ... Distributed Algebra\n" << std::endl;
                throw Plato::ParsingException(tMsg.str());
            }

        }
        catch(...)
        {
            aInterface->Catch();
        }
        return (tVector);
    }
    /********************************************************************************************/
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> createReduction(const MPI_Comm & aComm,
                                                                                         Plato::Interface* aInterface) const
    /********************************************************************************************/
    {
        std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> tReductions;
        try
        {
            auto tInputData = aInterface->getInputData();

            if( tInputData.size<Plato::Interface>("Optimizer") > 1 )
            {
                std::stringstream tMsg;
                tMsg << "\n\n ********** PLATO ERROR: FILE = " << __FILE__ << ", FUNCTION = " << __PRETTY_FUNCTION__
                        << ", LINE = " << __LINE__ << ", MESSAGE: MULTIPLE 'Optimizer' DEFINITIONS\n\n";
                Plato::ParsingException tParsingException(tMsg.str());
                aInterface->registerException(tParsingException);
            }
            auto tOptimizerNode = tInputData.get<Plato::InputData>("Optimizer");

            std::string tAlgebra;
            if( tOptimizerNode.size<Plato::InputData>("Options") )
            {
                auto tOptionsNode = tOptimizerNode.get<Plato::InputData>("Options");
                tAlgebra = Plato::Get::String(tOptionsNode, "Algebra");
            }

            if(tAlgebra == "Serial")
            {
                try
                {
                    tReductions = std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>();
                }
                catch(...)
                {
                    aInterface->Catch();
                }
            }
            else if(tAlgebra == "Distributed")
            {
                try
                {
                    tReductions = std::make_shared<Plato::DistributedReductionOperations<ScalarType, OrdinalType>>(aComm);
                }
                catch(...)
                {
                    aInterface->Catch();
                }
            }
            else if(tAlgebra == "GPU")
            {
                try
                {
                    tReductions = std::make_shared<Plato::KokkosReductionOperations<ScalarType, OrdinalType>>();
                }
                catch(...)
                {
                    aInterface->Catch();
                }
            }
            else if(tAlgebra == "")
            {
                /* DEFAULT REDUCTION OPERATIONS = DISTRIBUTED */
                try
                {
                    tReductions = std::make_shared<Plato::DistributedReductionOperations<ScalarType, OrdinalType>>(aComm);
                }
                catch(...)
                {
                    aInterface->Catch();
                }
            }
            else
            {
                std::stringstream tMsg;
                tMsg << "\n\n ********** PLATO ERROR: FILE = " << __FILE__ << ", FUNCTION = " << __PRETTY_FUNCTION__
                        << ", LINE = " << __LINE__ << ", MESSAGE: User Specified Algebra: " << tAlgebra << ", Unknown."
                        << std::endl << "Valid options are\n"
                        << "\t GPU ... Graphics Processing Units (GPU) Accelerated Algebra\n"
                        << "\t Serial ... Serial Algebra\n" << "\t Distributed ... Distributed Algebra\n" << std::endl;
                throw Plato::ParsingException(tMsg.str());
            }

        }
        catch(...)
        {
            aInterface->Catch();
        }
        return (tReductions);
    }

private:
    AlgebraFactory(const Plato::AlgebraFactory<ScalarType, OrdinalType>&);
    Plato::AlgebraFactory<ScalarType, OrdinalType> & operator=(const Plato::AlgebraFactory<ScalarType, OrdinalType>&);
};
// class AlgebraFactory

} // namespace Plato

#endif /* PLATO_ALGEBRAFACTORY_HPP_ */
