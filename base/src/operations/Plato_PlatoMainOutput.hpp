/*
 * Plato_PlatoMainOutput.hpp
 *
 *  Created on: Jun 27, 2019
 */

#pragma once

#include "Plato_LocalOperation.hpp"

namespace Plato
{

class InputData;

/******************************************************************************//**
 * @brief Manage PLATO Main output
 **********************************************************************************/
class PlatoMainOutput : public Plato::LocalOp
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aPlatoApp PLATO application
     * @param [in] aNode input XML data
     **********************************************************************************/
    PlatoMainOutput(PlatoApp* aPlatoApp, Plato::InputData& aNode);

    /******************************************************************************//**
     * @brief Destructor
     **********************************************************************************/
    ~PlatoMainOutput();

    /******************************************************************************//**
     * @brief perform local operation - output data
     **********************************************************************************/
    void operator()();

    /******************************************************************************//**
     * @brief Return local operation's argument list
     * @param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg> & aLocalArgs);

private:
    /******************************************************************************//**
     * @brief Extract iso-surface
     * @param [in] aIteration current optimization iteration
     **********************************************************************************/
    void extractIsoSurface(int aIteration);

private:
    std::vector<Plato::LocalArg> mOutputData; /*!< set of output data */
    int mOutputFrequency; /*!< output frequency */
    int mOutputMethod; /*!< epu output data - distributed or serial */
    std::string mDiscretization; /*!< topology representation, density or levelset */
    bool mWriteRestart; /*!< flag - write restart file */
    std::string mBaseName; /*!< output file base name */
    bool mAppendIterationCount; /*!< flag - append optimization iteration count */
    std::vector<std::string> mRequestedFormats; /*!< names of formats to write */
};
// class PlatoMainOutput

}
// namespace Plato
