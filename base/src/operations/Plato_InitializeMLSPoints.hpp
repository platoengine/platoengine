/*
 * Plato_InitializeMLSPoints.hpp
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
 * @brief Initialize Moving Least Square (MLS) points
**********************************************************************************/
template<int SpaceDim, typename ScalarType = double>
class InitializeMLSPoints : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    InitializeMLSPoints(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
            Plato::LocalOp(aPlatoApp),
            mOutputName("MLS Point Values")
    {
        auto tName = Plato::Get::String(aNode, "MLSName");
        auto& tMLS = mPlatoApp->getMovingLeastSquaredData();
        if(tMLS.count(tName) == 0)
        {
            throw Plato::ParsingException("Requested PointArray that doesn't exist.");
        }
        mMLS = mPlatoApp->getMovingLeastSquaredData()[tName];

        mFieldName = Plato::Get::String(aNode, "Field");
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~InitializeMLSPoints()
    {
    }

    /******************************************************************************//**
     * @brief perform local operation - initialize MLS points
    **********************************************************************************/
    void operator()()
    {
        auto tFields = Plato::any_cast<MLS_Type>(mMLS->mls).getPointFields();
        auto tField = tFields[mFieldName];

        std::vector<double>* tLocalData = mPlatoApp->getValue(mOutputName);
        Kokkos::View<ScalarType*, Kokkos::HostSpace, Kokkos::MemoryUnmanaged>
            tLocalDataHost(tLocalData->data(), tLocalData->size());
        Kokkos::deep_copy(tLocalDataHost, tField);
    }

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg>& aLocalArgs)
    {
        int tNumPoints = Plato::any_cast<MLS_Type>(mMLS->mls).getNumPoints();
        aLocalArgs.push_back(Plato::LocalArg
            { Plato::data::layout_t::SCALAR, mOutputName, tNumPoints });
    }

private:
    typedef typename Plato::Geometry::MovingLeastSquares<SpaceDim, ScalarType> MLS_Type;

    shared_ptr<Plato::MLSstruct> mMLS; /*!< MLS meta data */
    const std::string mOutputName; /*!< output argument name */
    std::string mFieldName; /*!< field argument name */
};
// class InitializeMLSPoints;

}
// namespace Plato
