/*
 * XMLGeneratorCriterionMetadata.hpp
 *
 *  Created on: May 14, 2020
 */

#pragma once

#include <unordered_map>
#include <vector>
#include <string>

#include "Plato_FreeFunctions.hpp"
#include "XMLG_Macros.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \struct Criterion
 * \brief Criterion metadata for Plato problems.
**********************************************************************************/
struct Criterion
{
private:
    std::string mID; /*!< criterion identification number */
    std::string mType;  /*!< criterion type */
    std::unordered_map<std::string, std::string> mMetaData; /*!< Scenario metadata, map< tag, value > */
    std::vector<std::string> mCriterionWeights;
    std::vector<std::string> mCriterionIDs;

public:
    /******************************************************************************//**
     * \fn id
     * \brief Return criterion identification number.
     * \return identification number
    **********************************************************************************/
    std::string id() const
    {
        return mID;
    }

    /******************************************************************************//**
     * \fn id
     * \brief Set criterion identification number.
     * \param [in] aID identification number
    **********************************************************************************/
    void id(const std::string& aID)
    {
        mID = aID;
    }

    /******************************************************************************//**
     * \fn type
     * \brief Return criterion type.
     * \return type
    **********************************************************************************/
    std::string type() const
    {
        return mType;
    }

    /******************************************************************************//**
     * \fn type
     * \brief Set criterion type.
     * \param [in] aType type
    **********************************************************************************/
    void type(const std::string& aType)
    {
        mType = aType;
    }

    /******************************************************************************//**
     * \fn value
     * \brief If criterion metadata is defined, return its value; else, return an empty string.
     * \param [in]  aTag    criterion metadata
     * \return criterion metadata value
    **********************************************************************************/
    std::string value(const std::string& aTag) const
    {
        auto tTag = Plato::tolower(aTag);
        auto tItr = mMetaData.find(tTag);
        auto tOutput = tItr == mMetaData.end() ? "" : tItr->second;
        return tOutput;
    }

    /******************************************************************************//**
     * \fn append
     * \brief Append parameter to metadata.
     * \param [in] aTag   parameter tag
     * \param [in] aValue parameter value
    **********************************************************************************/
    void append(const std::string& aTag, const std::string& aValue)
    {
        if (aTag.empty())
        {
            THROWERR(std::string("XML Generator Scenario Metadata: Parameter with tag '") + aTag + "' is empty.")
        }
        auto tTag = Plato::tolower(aTag);
        mMetaData[aTag] = aValue;
    }

    /******************************************************************************//**
     * \fn materialPenaltyExponent
     * \brief Set string value for keyword 'material_penalty_exponent'.
     * \param [in] aInput string value
     **********************************************************************************/
    void materialPenaltyExponent(const std::string& aInput)
    {
        this->append("material_penalty_exponent", aInput);
    }

    /******************************************************************************//**
     * \fn materialPenaltyExponent
     * \brief Return string value for keyword 'material_penalty_exponent'.
     * \return value
     **********************************************************************************/
    std::string materialPenaltyExponent() const
    {
        return (this->value("material_penalty_exponent"));
    }

    /******************************************************************************//**
     * \fn minErsatzMaterialConstant
     * \brief Set string value for keyword 'minimum_ersatz_material_value'.
     * \param [in] aInput string value
     **********************************************************************************/
    void minErsatzMaterialConstant(const std::string& aInput)
    {
        this->append("minimum_ersatz_material_value", aInput);
    }

    /******************************************************************************//**
     * \fn minErsatzMaterialConstant
     * \brief Return string value for keyword 'minimum_ersatz_material_value'.
     * \return value
     **********************************************************************************/
    std::string minErsatzMaterialConstant() const
    {
        return (this->value("minimum_ersatz_material_value"));
    }

    /******************************************************************************//**
     * \fn pnormExponent
     * \brief Set string value for keyword 'pnorm_exponent'.
     * \param [in] aInput string value
     **********************************************************************************/
    void pnormExponent(const std::string& aInput)
    {
        this->append("stress_p_norm_exponent", aInput);
    }

    /******************************************************************************//**
     * \fn pnormExponent
     * \brief Return string value for keyword 'pnorm_exponent'.
     * \return value
     **********************************************************************************/
    std::string pnormExponent() const
    {
        return (this->value("stress_p_norm_exponent"));
    }

    /******************************************************************************//**
     * \fn scmmPenaltyExpansionMultiplier
     * \brief Set string value for keyword 'scmm_penalty_expansion_multiplier'.
     * \param [in] aInput string value
     **********************************************************************************/
    void scmmPenaltyExpansionMultiplier(const std::string& aInput)
    {
        this->append("scmm_penalty_expansion_multiplier", aInput);
    }

    /******************************************************************************//**
     * \fn scmmPenaltyExpansionMultiplier
     * \brief Return string value for keyword 'scmm_penalty_expansion_multiplier'.
     * \return value
     **********************************************************************************/
    std::string scmmPenaltyExpansionMultiplier() const
    {
        return (this->value("scmm_penalty_expansion_multiplier"));
    }

    /******************************************************************************//**
     * \fn scmmPenaltyUpperBound
     * \brief Set string value for keyword 'scmm_penalty_upper_bound'.
     * \param [in] aInput string value
     **********************************************************************************/
    void scmmPenaltyUpperBound(const std::string& aInput)
    {
        this->append("scmm_penalty_upper_bound", aInput);
    }

    /******************************************************************************//**
     * \fn scmmPenaltyUpperBound
     * \brief Return string value for keyword 'scmm_penalty_upper_bound'.
     * \return value
     **********************************************************************************/
    std::string scmmPenaltyUpperBound() const
    {
        return (this->value("scmm_penalty_upper_bound"));
    }

    /******************************************************************************//**
     * \fn scmmInitialPenalty
     * \brief Set string value for keyword 'scmm_initial_penalty'.
     * \param [in] aInput string value
     **********************************************************************************/
    void scmmInitialPenalty(const std::string& aInput)
    {
        this->append("scmm_initial_penalty", aInput);
    }

    /******************************************************************************//**
     * \fn scmmInitialPenalty
     * \brief Return string value for keyword 'scmm_initial_penalty'.
     * \return value
     **********************************************************************************/
    std::string scmmInitialPenalty() const
    {
        return (this->value("scmm_initial_penalty"));
    }

    /******************************************************************************//**
     * \fn scmmStressWeight
     * \brief Set string value for keyword 'scmm_stress_weight'.
     * \param [in] aInput string value
     **********************************************************************************/
    void scmmStressWeight(const std::string& aInput)
    {
        this->append("scmm_stress_weight", aInput);
    }

    /******************************************************************************//**
     * \fn scmmStressWeight
     * \brief Return string value for keyword 'scmm_stress_weight'.
     * \return value
     **********************************************************************************/
    std::string scmmStressWeight() const
    {
        return (this->value("scmm_stress_weight"));
    }

    /******************************************************************************//**
     * \fn scmmMassWeight
     * \brief Set string value for keyword 'scmm_mass_weight'.
     * \param [in] aInput string value
     **********************************************************************************/
    void scmmMassWeight(const std::string& aInput)
    {
        this->append("scmm_mass_weight", aInput);
    }

    /******************************************************************************//**
     * \fn scmmMassWeight
     * \brief Return string value for keyword 'scmm_mass_weight'.
     * \return value
     **********************************************************************************/
    std::string scmmMassWeight() const
    {
        return (this->value("scmm_mass_weight"));
    }

    /******************************************************************************//**
     * \fn stressLimit
     * \brief Set string value for keyword 'stress_limit'.
     * \param [in] aInput string value
     **********************************************************************************/
    void stressLimit(const std::string& aInput)
    {
        this->append("stress_limit", aInput);
    }

    /******************************************************************************//**
     * \fn stressLimit
     * \brief Return string value for keyword 'stress_limit'.
     * \return value
     **********************************************************************************/
    std::string stressLimit() const
    {
        return (this->value("stress_limit"));
    }

    /******************************************************************************//**
     * \fn criterionIDs
     * \brief Set ID strings for composite criteria
     * \param [in] aInput list of IDs
     **********************************************************************************/
    void criterionIDs(const std::vector<std::string>& aInput)
    {
        this->mCriterionIDs = aInput;
    }

    /******************************************************************************//**
     * \fn criterionIDs
     * \brief Return ID strings for composite criteria
     * \return value
     **********************************************************************************/
    std::vector<std::string> criterionIDs() const
    {
        return this->mCriterionIDs;
    }

    /******************************************************************************//**
     * \fn criterionWeights
     * \brief Set weight strings for composite criteria
     * \param [in] aInput list of IDs
     **********************************************************************************/
    void criterionWeights(const std::vector<std::string>& aInput)
    {
        this->mCriterionWeights = aInput;
    }

    /******************************************************************************//**
     * \fn criterionWeights
     * \brief Return ID strings for composite criteria
     * \return value
     **********************************************************************************/
    std::vector<std::string> criterionWeights() const
    {
        return this->mCriterionWeights;
    }

    /* These are all related to stress-constrained mass minimization problems with Sierra/SD */
    std::string volume_misfit_target() const { return this->value("volume_misfit_target"); }
    std::string scmm_constraint_exponent() const { return this->value("scmm_constraint_exponent"); }
    std::string relative_stress_limit() const { return this->value("relative_stress_limit"); }
    std::string relaxed_stress_ramp_factor() const { return this->value("relaxed_stress_ramp_factor"); }
    std::string limit_power_min() const { return this->value("limit_power_min"); }
    std::string limit_power_max() const { return this->value("limit_power_max"); }
    std::string limit_power_feasible_bias() const { return this->value("limit_power_feasible_bias"); }
    std::string limit_power_feasible_slope() const { return this->value("limit_power_feasible_slope"); }
    std::string limit_power_infeasible_bias() const { return this->value("limit_power_infeasible_bias"); }
    std::string limit_power_infeasible_slope() const { return this->value("limit_power_infeasible_slope"); }
    std::string limit_reset_subfrequency() const { return this->value("limit_reset_subfrequency"); }
    std::string limit_reset_count() const { return this->value("limit_reset_count"); }
    std::string inequality_allowable_feasibility_upper() const { return this->value("inequality_allowable_feasibility_upper"); }
    std::string inequality_feasibility_scale() const { return this->value("inequality_feasibility_scale"); }
    std::string inequality_infeasibility_scale() const { return this->value("inequality_infeasibility_scale"); }
    std::string stress_inequality_power() const { return this->value("stress_inequality_power"); }
    std::string stress_favor_final() const { return this->value("stress_favor_final"); }
    std::string stress_favor_updates() const { return this->value("stress_favor_updates"); }
    std::string volume_penalty_power() const { return this->value("volume_penalty_power"); }
    std::string volume_penalty_divisor() const { return this->value("volume_penalty_divisor"); }
    std::string volume_penalty_bias() const { return this->value("volume_penalty_bias"); }
    std::string surface_area_sideset_id() const { return this->value("surface_area_sideset_id"); }

};
// struct Criterion

}
// namespace XMLGen
