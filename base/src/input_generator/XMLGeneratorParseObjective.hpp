/*
 * XMLGeneratorParseObjective.hpp
 *
 *  Created on: Jun 17, 2020
 */

#pragma once

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseObjective
 * \brief Parse inputs in objective function blocks and store values in XMLGen::Objective.
**********************************************************************************/
class ParseObjective : public XMLGen::ParseMetadata<std::vector<XMLGen::Objective>>
{
private:
    XMLGen::MetaDataTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::Objective> mData; /*!< objective functions metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn setIdentification
     * \brief Set objective functions identification number/name. If there are objective \n
     * functions without an identification number/name, assign default values.
    **********************************************************************************/
    void setIdentification();

    /******************************************************************************//**
     * \fn checkPerfomerNames
     * \brief If performer keywords are empty, set to default value 'code_name' + 'id', \n
     * where 'code_name' denotes the input code name provided by the user and 'id' \n
     * denotes the identification name/number for an objective function.
    **********************************************************************************/
    void checkPerfomerNames();

    /******************************************************************************//**
     * \fn finalize
     * \brief Perform final checks and member data definitions for objective functions.
    **********************************************************************************/
    void finalize();

    /******************************************************************************//**
     * \fn setName
     * \brief Set objective function's user assigned name.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setName(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setCategory
     * \brief Set objective function category.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setCategory(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setCode
     * \brief Set code responsible for evaluating the objective function.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setCode(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setNumRanks
     * \brief Set number of ranks assign to the code responsible for evaluating the \n
     * objective function.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setNumRanks(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setNumProcessors
     * \brief Set number of processors assign to the code responsible for evaluating \n
     * the objective function.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setNumProcessors(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setWeight
     * \brief Set objective function weight.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setWeight(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setPerformer
     * \brief Set objective function performer, i.e. name use to denote simulation \n
     * code responsible for evaluating the objective function.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setPerformer(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setPenaltyPower
     * \brief Set exponent for material density penalty function.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setPenaltyPower(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setPnormExponent
     * \brief Set p-norm exponent.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setPnormExponent(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setNormalizeObjective
     * \brief Set normalize objective function flag.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setNormalizeObjective(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setMinimumErsatzValue
     * \brief Set minimum Ersatz material parameter used in material density penalty function.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMinimumErsatzValue(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setAnalysisSolverTolerance
     * \brief Set solver tolerance for solver use to solve the finite element system of equations.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setAnalysisSolverTolerance(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setNaturalBoundaryConditionsCaseIDs
     * \brief Set identification numbers associated with each natural boundary condition case.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setNaturalBoundaryConditionsCaseIDs(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setNaturalBoundaryConditionsCaseIDs
     * \brief Set weights associated with each natural boundary conditions.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setNaturalBoundaryConditionsCaseWeights(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setNaturalBoundaryConditionsCaseIDs
     * \brief Set identification numbers associated with the essential boundary conditions.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setEssentialBoundaryConditionsCaseIDs(XMLGen::Objective& aMetadata);

    void setFrequencyResponseFunctionMatchNodesets(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setOutputForPlotting
     * \brief Set keywords for output quantities of interests.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setOutputForPlotting(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setDistributeObjectiveType
     * \brief Set parameters used to identify if we are running multiple performers in \n
     * using the multiple program, multiple data parallel programming model as well as
     * the number of processors used for each criterion use case.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setDistributeObjectiveType(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setStressConstrainedParam
     * \brief Set parameters for stress constrained mass minimization problem.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setStressConstrainedParam(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set objective metadata.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void setMetaData(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn checkCode
     * \brief Throw error if simulation code is not supported.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkCode(const XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn checkLoadCases
     * \brief Throw error if load cases are not properly defined.
     * \param [in/out] aMetadata objective metadata
    **********************************************************************************/
    void checkLoadCases(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn checkMultiLoadCaseFlag
     * \brief Check if multi-laod case flag has been properly set.
     * \param [in/out] aMetadata objective metadata
    **********************************************************************************/
    void checkMultiLoadCaseFlag(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn checkLoadCaseIDs
     * \brief Throw error if list of natural boundary conditions identification numbers is empty.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkLoadCaseIDs(const XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn checkLoadCaseWeights
     * \brief Set weights associated with each natural boundary condition case to 1.0 \n
     * if not defined in the input file.
     * \param [in/out] aMetadata objective metadata
    **********************************************************************************/
    void checkLoadCaseWeights(XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn checkEssentialBoundaryConditionIDs
     * \brief Throw error if list of essential boundary conditions identification numbers is empty.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkEssentialBoundaryConditionIDs(const XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn checkEssentialBoundaryConditionIDs
     * \brief Throw error if output keyword is not supported.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkOutputForPlotting(const XMLGen::Objective& ValidOutputKeys);

    /******************************************************************************//**
     * \fn checkDistributeObjective
     * \brief Throw error if number of processors assigned to distributed objective \n
     * use case is not a finite number.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkDistributeObjective(const XMLGen::Objective& aMetadata);

    /******************************************************************************//**
     * \fn checkMetaData
     * \brief Check if objective metadata is properly defined.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkMetaData(XMLGen::Objective& aMetadata);

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return objective functions metadata.
     * \return container of objective functions and corresponding metadata
    **********************************************************************************/
    std::vector<XMLGen::Objective> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse objective functions metadata.
     * \param [in] aInputFile input metadata
    **********************************************************************************/
    void parse(std::istream& aInputFile) override;
};
// class ParseObjective

}
// namespace XMLGen
