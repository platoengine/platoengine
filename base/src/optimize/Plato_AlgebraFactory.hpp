/*
 * Plato_AlgebraFactory.hpp
 *
 *  Created on: Feb 21, 2018
 */

/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
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
