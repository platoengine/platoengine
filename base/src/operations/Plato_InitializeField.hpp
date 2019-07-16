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
 * Plato_InitializeField.hpp
 *
 *  Created on: Jun 29, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"

class PlatoApp;
class DistributedVector;

namespace Plato
{

class InputData;

/******************************************************************************//**
 * @brief Set/Compute initial level set field
**********************************************************************************/
class InitializeField : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
    **********************************************************************************/
    InitializeField(PlatoApp* aPlatoApp, Plato::InputData & aNode);

    /******************************************************************************//**
     * @brief perform local operation - initialize level
    **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg> & aLocalArgs);

private:
    /******************************************************************************//**
     * @brief Read initial level set field for restart
     * @param [in] aField distributed topology field vector
     * @param [in] aValues local topology values for my processor
    **********************************************************************************/
    void getInitialValuesForRestart(const DistributedVector &aField, std::vector<double> &aValues);

    /******************************************************************************//**
     * @brief Compute initial "swiss cheese" level set field
     * @param [in] aField distributed topology field vector
     * @param [in] aValues local topology values for my processor
    **********************************************************************************/
    void getInitialValuesForSwissCheeseLevelSet(const DistributedVector &aField, std::vector<double> &aValues);

    /******************************************************************************//**
     * @brief Compute initial "primitive-based" level set field
     * @param [in] aField distributed topology field vector
     * @param [in] aValues local topology values for my processor
    **********************************************************************************/
    void getInitialValuesForPrimitivesLevelSet(const DistributedVector &aField, std::vector<double> &aValues);

    /******************************************************************************//**
     * @brief Evaluate "swiss cheese" function
     * @param [in] aX x-coordinate
     * @param [in] aY y-coordinate
     * @param [in] aZ z-coordinate
     * @param [in] aLowerCoordBoundsOfDomain lower domain bounds on x, y, and z coordinates
     * @param [in] aUpperCoordBoundsOfDomain upper domain bounds on x, y, and z coordinates
     * @param [in] aAverageElemLength average element lenght
    **********************************************************************************/
    double evaluateSwissCheeseLevelSet(const double &aX,
                                       const double &aY,
                                       const double &aZ,
                                       std::vector<double> aLowerCoordBoundsOfDomain,
                                       std::vector<double> aUpperCoordBoundsOfDomain,
                                       double aAverageElemLength);

private:
    bool mCreateSpheres; /*!< create spheres-based "swiss cheese" level set field */
    int mIteration; /*!< read topology field from this optimization iteration */
    double mUniformValue; /*!< value used to initialize uniform design variable field */
    double mMinCoords[3]; /*!< 3D array of minimum coordinates in x, y, and z */
    double mMaxCoords[3]; /*!< 3D array of maximum coordinates in x, y, and z */
    Plato::data::layout_t mOutputLayout; /*!< output data layout, e.g. scalar value, scalar field, etc */

    std::string mFileName; /*!< output file name */
    std::string mStringMethod; /*!< method used to initialized level set field */
    std::string mSphereRadius; /*!< level set spheres' (used by swiss cheese method) radius */
    std::string mOutputFieldName; /*!< operation's output argument name */
    std::string mSpherePackingFactor; /*!< spacing between spheres */
    std::string mSphereSpacingX; /*!< distance in the x-direction between consecutive spheres' centers */
    std::string mSphereSpacingY; /*!< distance in the y-direction between consecutive spheres' centers */
    std::string mSphereSpacingZ; /*!< distance in the z-direction between consecutive spheres' centers */
    std::string mVariableName; /*!< input topology field argument */
    std::vector<int> mLevelSetNodesets; /*!< set of node sets used to store input level set field */
};
// class InitializeField;

}
// namespace Plato
