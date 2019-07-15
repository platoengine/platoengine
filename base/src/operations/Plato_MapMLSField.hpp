/*
 * Plato_MapMLSField.hpp
 *
 *  Created on: Jun 30, 2019
 */

#pragma once

#include "PlatoApp.hpp"
#include "Plato_MLS.hpp"
#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_MetaDataMLS.hpp"
#include "Plato_LocalOperation.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Map Moving Least Square (MLS) field
**********************************************************************************/
template<int SpaceDim, typename ScalarType = double>
class MapMLSField : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    MapMLSField(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
            Plato::LocalOp(aPlatoApp),
            mInputName("MLS Field Values"),
            mOutputName("Mapped MLS Point Values")
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
    ~MapMLSField()
    {
    }

    /******************************************************************************//**
     * @brief perform local operation - compute topology field from MLS points
    **********************************************************************************/
    void operator()()
    {
        // pull field values into Kokkos::View
        //
        auto& tLocalInput = *(mPlatoApp->getNodeField(mInputName));
        int tMyLength = tLocalInput.MyLength();
        double* tDataView;
        tLocalInput.ExtractView(&tDataView);
        Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged> tNodeValuesHost(tDataView, tMyLength);
        Kokkos::View<ScalarType*, Kokkos::DefaultExecutionSpace::memory_space> tNodeValues("values", tMyLength);
        Kokkos::deep_copy(tNodeValues, tNodeValuesHost);

        // pull node coordinates into Kokkos::View
        //
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

        // create output View
        //
        std::vector<double>* tLocalData = mPlatoApp->getValue(mOutputName);
        Kokkos::View<ScalarType*, Kokkos::DefaultExecutionSpace::memory_space>
            tMappedValues("mapped values", tLocalData->size());

        Plato::any_cast<MLS_Type>(mMLS->mls).mapToPoints(tNodeCoords, tNodeValues, tMappedValues);

        // pull from View to local data
        //
        Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged>
            tLocalDataHost(tLocalData->data(), tLocalData->size());
        Kokkos::deep_copy(tLocalDataHost, tMappedValues);
    }

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
    {
        int tNumPoints = Plato::any_cast<MLS_Type>(mMLS->mls).getNumPoints();
        aLocalArgs.push_back(Plato::LocalArg
            { Plato::data::layout_t::SCALAR_FIELD, mInputName });
        aLocalArgs.push_back(Plato::LocalArg
            { Plato::data::layout_t::SCALAR, mOutputName, tNumPoints });
    }

private:
    typedef typename Plato::Geometry::MovingLeastSquares<SpaceDim, ScalarType> MLS_Type;

    shared_ptr<Plato::MLSstruct> mMLS; /*!< MLS meta data */
    const std::string mInputName; /*!< input field argument name */
    const std::string mOutputName; /*!< output field argument name */
};
// class MapMLSField

}
// namespace Plato
