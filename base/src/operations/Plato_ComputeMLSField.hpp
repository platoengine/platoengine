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

/*
 * Plato_ComputeMLSField.hpp
 *
 *  Created on: Jun 30, 2019
 */

#pragma once

#include "Plato_MLS.hpp"
#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_MetaDataMLS.hpp"
#include "Plato_LocalOperation.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Apply Moving Least Square (MLS) method to compute control field
**********************************************************************************/
template<int SpaceDim, typename ScalarType = double>
class ComputeMLSField : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    ComputeMLSField(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
            Plato::LocalOp(aPlatoApp),
            mInputName("MLS Point Values"),
            mOutputName("MLS Field Values")
    {
        auto tName = Plato::Get::String(aNode, "MLSName");
        auto& tMLS = mPlatoApp->getMovingLeastSquaredData();
        if(tMLS.count(tName) == 0)
        {
            throw Plato::ParsingException("Requested PointArray that doesn't exist.");
        }
        mMLS = mPlatoApp->getMovingLeastSquaredData()[tName];
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~ComputeMLSField()
    {
    }

    /******************************************************************************//**
     * @brief perform local operation - compute control field
    **********************************************************************************/
    void operator()()
    {
        std::vector<double>* tLocalData = mPlatoApp->getValue(mInputName);
        Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> tPointsHost(tLocalData->data(),
                                                                                           tLocalData->size());
        Kokkos::View<ScalarType*, Kokkos::DefaultExecutionSpace::memory_space> tPointValues("point values",
                                                                                             tLocalData->size());
        Kokkos::deep_copy(tPointValues, tPointsHost);

        auto& tLocalOutput = *(mPlatoApp->getNodeField(mOutputName));
        int tMyLength = tLocalOutput.MyLength();

        Kokkos::View<ScalarType*, Kokkos::DefaultExecutionSpace::memory_space> tNodeValues("values", tMyLength);
        Kokkos::View<ScalarType**, Kokkos::LayoutRight, Kokkos::DefaultExecutionSpace::memory_space>
            tNodeCoords("coords", tMyLength, SpaceDim);
        auto tNodeCoordsHost = Kokkos::create_mirror_view(tNodeCoords);
        auto tMesh = mPlatoApp->getLightMP()->getMesh();
        {
            auto tCoordsSub = Kokkos::subview(tNodeCoordsHost, Kokkos::ALL(), /*dim_index=*/0);
            Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> tCoord(tMesh->getX(), tMyLength);
            Kokkos::deep_copy(tCoordsSub, tCoord);
        }
        if(SpaceDim > 1)
        {
            auto tCoordsSub = Kokkos::subview(tNodeCoordsHost, Kokkos::ALL(), /*dim_index=*/1);
            Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> tCoord(tMesh->getY(), tMyLength);
            Kokkos::deep_copy(tCoordsSub, tCoord);
        }
        if(SpaceDim > 2)
        {
            auto tCoordsSub = Kokkos::subview(tNodeCoordsHost, Kokkos::ALL(), /*dim_index=*/2);
            Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> tCoord(tMesh->getZ(), tMyLength);
            Kokkos::deep_copy(tCoordsSub, tCoord);
        }
        Kokkos::deep_copy(tNodeCoords, tNodeCoordsHost);

        Plato::any_cast<MLS_Type>(mMLS->mls).f(tPointValues, tNodeCoords, tNodeValues);

        double* tDataView;
        tLocalOutput.ExtractView(&tDataView);
        Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> tNodeValuesHost(tDataView, tMyLength);
        Kokkos::deep_copy(tNodeValuesHost, tNodeValues);
    }

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
    {
        int tNumPoints = Plato::any_cast<MLS_Type>(mMLS->mls).getNumPoints();
        aLocalArgs.push_back(Plato::LocalArg
            { Plato::data::layout_t::SCALAR_FIELD, mOutputName });
        aLocalArgs.push_back(Plato::LocalArg
            { Plato::data::layout_t::SCALAR, mInputName, tNumPoints });
    }

private:
    typedef typename Plato::Geometry::MovingLeastSquares<SpaceDim, ScalarType> MLS_Type;

    shared_ptr<Plato::MLSstruct> mMLS; /*!< MLS meta data */
    const std::string mInputName; /*!< input field argument name */
    const std::string mOutputName; /*!< output field argument name */
};
// class ComputeMLSField;

}
// namespace Plato
