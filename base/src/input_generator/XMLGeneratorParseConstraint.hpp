/*
 * XMLGeneratorParseConstraint.hpp
 *
 *  Created on: Jun 19, 2020
 */

#pragma once

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorParseMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \class ParseConstraint
 * \brief Parse inputs in constraint function blocks and store their values in XMLGen::Constraint.
**********************************************************************************/
class ParseConstraint : public XMLGen::ParseMetadata<std::vector<XMLGen::Constraint>>
{
private:
    XMLGen::UseCaseTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::Constraint> mData; /*!< constraint functions metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate();

    /******************************************************************************//**
     * \fn finalize
     * \brief Perform final checks and member data definitions for constraint functions.
    **********************************************************************************/
    void finalize();

    /******************************************************************************//**
     * \fn checkTargeValue
     * \brief Verify that target constraint keyword is set. A target value must be \n
     * provided for all constrained optimization problems.
    **********************************************************************************/
    void checkTargeValue();

    /******************************************************************************//**
     * \fn setIdentification
     * \brief Set constraint functions identification number/name. If there are constraint \n
     * functions without an identification number/name, assign default values.
    **********************************************************************************/
    void setIdentification();

    /******************************************************************************//**
     * \fn setCode
     * \brief Set simulation code name.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setCode(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn setName
     * \brief Set user assigned constraint name.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setName(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn setCategory
     * \brief Set constraint category.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setCategory(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn setWeight
     * \brief Set constraint weight.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setWeight(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn setPerformer
     * \brief Set constraint performer, i.e. identification name used for the simulation \n
     * code responsible for evaluating the constraint function.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setPerformer(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn setPnormExponent
     * \brief Set p-norm exponent.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setPnormExponent(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn setNormalizedTarget
     * \brief Set p-norm exponent.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setNormalizedTarget(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn setAbsoluteTarget
     * \brief Set absolute constraint target values.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setAbsoluteTarget(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn setMinimumErsatzValue
     * \brief Set minimum Ersatz material constant.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMinimumErsatzValue(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn setPenaltyPower
     * \brief Set material penalty exponent, i.e. \f$ x^p \f$ , where \f$ p \f$ denotes the exponent.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setPenaltyPower(XMLGen::Constraint &aMetadata);

    /******************************************************************************//**
     * \fn setMetaData
     * \brief Set constraint metadata.
     * \param [in/out] aInputFile parsed input metadata
    **********************************************************************************/
    void setMetaData(XMLGen::Constraint& aMetadata);

    // TO BE DEPRECATED SOON!
    void setSurfaceArea(XMLGen::Constraint &aMetadata);

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return list of constraints and corresponding metadata.
     * \return list of uncertainty metadata
    **********************************************************************************/
    std::vector<XMLGen::Constraint> data() const override;

    /******************************************************************************//**
     * \fn parse
     * \brief Parse constraint functions metadata.
     * \param [in] aInputFile input metadata
    **********************************************************************************/
    void parse(std::istream &aInputFile) override;
};

}
