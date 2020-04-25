/*
 * Plato_Test_UncertainMaterial.cpp
 *
 *  Created on: Apr 22, 2020
 */

#include "gtest/gtest.h"

#include "Plato_SromXMLUtils.hpp"
#include "Plato_SromMetadata.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \brief Deterministic variable metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct DeterministicVariable
{
    std::string mTag;        /*!< main variable attribute , e.g. Poisson's ratio */
    std::string mValue;      /*!< attribute's deterministic value */
    std::string mAttribute;  /*!< attribute's category, e.g. homogeneous or heterogeneous */
};
// struct DeterministicVariable

struct Material
{
private:
    std::string mBlockID;     /*!< element block identification number */
    std::string mCategory;    /*!< isotropic, orthotropic, anisotropic, etc. */
    std::string mMaterialID;  /*!< material identification number */

    std::vector<Plato::srom::RandomVariable> mRandomVars; /*!< non-deterministic material property */
    std::vector<Plato::srom::DeterministicVariable> mDeterministicVars; /*!< deterministic material property */

public:
    bool isRandom() const
    {
        return (!mRandomVars.empty());
    }
    bool isDeterministic() const
    {
        return (mRandomVars.empty());
    }

    void check() const
    {
        if(mBlockID.empty())
        {
            THROWERR("SROM::Material: Material block identification number is not defined.")
        }

        if(mMaterialID.empty())
        {
            THROWERR("SROM::Material: Material identification number is not defined.")
        }

        auto tNumMaterialProperties = mRandomVars.size() + mDeterministicVars.size();
        if(tNumMaterialProperties > static_cast<size_t>(0))
        {
            std::ostringstream tMsg;
            tMsg << "SROM Material: Material properties for material with ID '" << mMaterialID.c_str()
                << "' in block with ID '" << mBlockID <<"' are not defined.";
            THROWERR(tMsg.str().c_str())
        }
    }

    void blockID(const std::string& aID)
    {
        mBlockID = aID;
    }
    std::string blockID() const
    {
        return (mBlockID);
    }

    void materialID(const std::string& aID)
    {
        mMaterialID = aID;
    }
    std::string materialID() const
    {
        return (mMaterialID);
    }

    void category(const std::string& aCategory)
    {
        mCategory = aCategory;
    }
    std::string category() const
    {
        return mCategory;
    }

    void append(const std::string &aTag,
                const std::string &aAttribute,
                const Plato::srom::Statistics &aStats)
    {
        Plato::srom::RandomVariable tVariable;
        tVariable.mTag = aTag;
        tVariable.mStatistics = aStats;
        tVariable.mAttribute = aAttribute;
        mRandomVars.push_back(tVariable);
    }
    std::vector<Plato::srom::RandomVariable> randomVars() const
    {
        return (mRandomVars);
    }

    void append(const std::string &aTag,
                const std::string &aAttribute,
                const std::string &aValue)
    {
        Plato::srom::DeterministicVariable tVariable;
        tVariable.mTag = aTag;
        tVariable.mValue = aValue;
        tVariable.mAttribute = aAttribute;
        mDeterministicVars.push_back(tVariable);
    }
    std::vector<Plato::srom::DeterministicVariable> deterministicVars() const
    {
        return (mDeterministicVars);
    }
};
// struct Material

using MaterialProps = std::map<std::string,std::pair<std::string, std::string>>;
struct RandomMaterial
{
private:
    double mProbability;       /*!< probability for this material instance */
    std::string mBlockID;      /*!< element block identification number */
    std::string mCategory;     /*!< isotropic, orthotropic, anisotropic, etc. */
    std::string mMaterialID;   /*!< material identification number */

    MaterialProps mTags; /*!< map between material property tag/label and its attribute and value, i.e. tags(tag,(attribute,value)) */

public:
    void check() const
    {
        if(mBlockID.empty())
        {
            THROWERR("SROM::RandomMaterial: Material block identification number is not defined.")
        }

        if(mMaterialID.empty())
        {
            THROWERR("SROM::RandomMaterial: Material identification number is not defined.")
        }

        if(mTags.empty())
        {
            THROWERR("SROM::RandomMaterial: Material tags are not defined.")
        }

        if(std::isfinite(mProbability) == false)
        {
            THROWERR("SROM::RandomMaterial: Probability is not a finite number.")
        }

        if(mProbability > static_cast<double>(0.0))
        {
            THROWERR("SROM::RandomMaterial: Probability is not a positive number.")
        }
    }

    void blockID(const std::string& aID)
    {
        mBlockID = aID;
    }
    std::string blockID() const
    {
        return (mBlockID);
    }

    void materialID(const std::string& aID)
    {
        mMaterialID = aID;
    }
    std::string materialID() const
    {
        return (mMaterialID);
    }

    void category(const std::string& aCategory)
    {
        mCategory = aCategory;
    }
    std::string category() const
    {
        return mCategory;
    }

    void probability(const double& aProbability)
    {
        mProbability = aProbability;
    }
    double probability() const
    {
        return mProbability;
    }

    std::string value(const std::string& aTag) const
    {
        auto tItr = mTags.find(aTag);
        if(tItr == mTags.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material: Material property with tag '" << aTag.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.second);
    }
    std::string attribute(const std::string& aTag) const
    {
        auto tItr = mTags.find(aTag);
        if(tItr == mTags.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material: Material property with tag '" << aTag.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.first);
    }
    std::vector<std::string> tags() const
    {
        std::vector<std::string> tTags;
        for(auto& tItr : mTags)
        {
            tTags.push_back(tItr.first);
        }
        return tTags;
    }
    void append(const std::string& aTag, const std::string& aAttribute, const std::string& aValue)
    {
        mTags.insert(std::pair<std::string, std::pair<std::string, std::string>>
            (aTag, std::pair<std::string, std::string>(aAttribute, aValue)));
    }
    size_t size() const
    {
        return (mTags.size());
    }
};
// struct RandomMaterial

struct RandomMaterialCase
{
private:
    std::string mCaseID;      /*!< random material case identification number */
    std::string mProbability; /*!< probability associated with this random load case */
    std::map<std::string, Plato::srom::RandomMaterial> mMaterials; /*!< set of random materials for random material case */

public:
    std::string caseID() const
    {
        return (mCaseID);
    }
    std::string probability() const
    {
        return (mProbability);
    }
    std::string blockID(const std::string& aMaterialID) const
    {
        auto tItr = mMaterials.find(aMaterialID);
        if(tItr == mMaterials.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material Case: Material with identification number '" << aMaterialID.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.blockID());
    }
    std::string category(const std::string& aMaterialID) const
    {
        auto tItr = mMaterials.find(aMaterialID);
        if(tItr == mMaterials.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material Case: Material with identification number '" << aMaterialID.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.category());
    }
    std::vector<std::string> tags(const std::string& aMaterialID) const
    {
        auto tItr = mMaterials.find(aMaterialID);
        if(tItr == mMaterials.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material Case: Material with identification number '" << aMaterialID.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.tags());
    }
    std::string value(const std::string& aMaterialID, const std::string& aTag) const
    {
        auto tItr = mMaterials.find(aMaterialID);
        if(tItr == mMaterials.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material Case: Material with identification number '" << aMaterialID.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.value(aTag));
    }
    std::string attribute(const std::string& aMaterialID, const std::string& aTag) const
    {
        auto tItr = mMaterials.find(aMaterialID);
        if(tItr == mMaterials.end())
        {
            std::ostringstream tMsg;
            tMsg << "Random Material Case: Material with identification number '" << aMaterialID.c_str() << "' is not defined.";
            THROWERR(tMsg.str().c_str())
        }
        return (tItr->second.attribute(aTag));
    }
    void append(const std::string& aMaterialID, const Plato::srom::RandomMaterial& aMaterial)
    {
        mMaterials.insert(std::pair<std::string, Plato::srom::RandomMaterial>("aMaterialID", aMaterial));
    }
};
// struct RandomMaterialCase

inline void define_deterministic_material(const Plato::srom::Material& aInMaterial, Plato::srom::RandomMaterial& aOutMaterial)
{
    aOutMaterial.probability(1.0);
    aOutMaterial.blockID(aInMaterial.blockID());
    aOutMaterial.category(aInMaterial.category());
    aOutMaterial.materialID(aInMaterial.materialID());
    auto tDeterministicVars = aInMaterial.deterministicVars();
    for (auto tVariable : tDeterministicVars)
    {
        aOutMaterial.append(tVariable.mTag, tVariable.mAttribute, tVariable.mValue);
    }
}

inline void append_deterministic_materials(const std::vector<Plato::srom::Material>& aInput,
                                           std::vector<Plato::srom::RandomMaterialCase>& aOutput)
{
    for (auto& tMaterialCase : aOutput)
    {
        for (auto tMaterial : aInput)
        {
            if (tMaterial.isRandom())
            {
                std::ostringstream tMsg;
                tMsg << "Append Deterministic Materials: Material with identification number '" << tMaterial.materialID()
                    << "' in block with identification number '" << tMaterial.blockID() << "' is not defined.";
                THROWERR(tMsg.str().c_str())
            }

            Plato::srom::RandomMaterial tOutMaterial;
            Plato::srom::define_deterministic_material(tMaterial, tOutMaterial);
            tMaterialCase.append(tMaterial.materialID(), tOutMaterial);
        }
    }
}

inline void split_random_and_deterministic_materials(const std::vector<Plato::srom::Material>& aMaterials,
                                                     std::vector<Plato::srom::Material>& aRandomMats,
                                                     std::vector<Plato::srom::Material>& aDeterministicMats)
{
    if(aMaterials.empty())
    {
        PRINTERR("Expand Random And Deterministic Materials: Input material container is empty.");
    }

    for(auto& tMaterial : aMaterials)
    {
        if(tMaterial.isDeterministic())
        {
            aDeterministicMats.push_back(tMaterial);
        }
        else
        {
            aRandomMats.push_back(tMaterial);
        }
    }
}

inline void initialize_random_material_set(const Plato::srom::Material& aMaterial,
                                           const Plato::srom::SromVariable& aSromVariable,
                                           std::vector<Plato::srom::RandomMaterial>& aRandomMaterialSet)
{
    for(auto& tSample : aSromVariable.mSampleProbPairs.mSamples)
    {
        Plato::srom::RandomMaterial tRandomMaterial;
        tRandomMaterial.blockID(aMaterial.blockID());
        tRandomMaterial.category(aMaterial.category());
        tRandomMaterial.materialID(aMaterial.materialID());

        auto tIndex = &tSample - &aSromVariable.mSampleProbPairs.mSamples[0];
        tRandomMaterial.probability(aSromVariable.mSampleProbPairs.mProbabilities[tIndex]);

        auto tValue = std::to_string(aSromVariable.mSampleProbPairs.mSamples[tIndex]);
        tRandomMaterial.append(aSromVariable.mTag, aSromVariable.mAttribute, tValue);

        aRandomMaterialSet.push_back(tRandomMaterial);
    }
}

inline void update_random_material_set(const Plato::srom::Material& aMaterial,
                                       const Plato::srom::SromVariable& aSromVariable,
                                       std::vector<Plato::srom::RandomMaterial>& aRandomMaterialSet)
{
    auto tOriginalRandomMaterialSet = aRandomMaterialSet;

    for(auto& tRandomMaterial : aRandomMaterialSet)
    {
        auto tUpdatedProbability = tRandomMaterial.probability()
            * aSromVariable.mSampleProbPairs.mProbabilities[0];
        tRandomMaterial.probability(tUpdatedProbability);
        auto tValue = std::to_string(aSromVariable.mSampleProbPairs.mSamples[0]);
        tRandomMaterial.append(aSromVariable.mTag, aSromVariable.mAttribute, tValue);
    }

    auto tBeginItr = aSromVariable.mSampleProbPairs.mSamples.begin();
    for(auto tItr = tBeginItr++; tItr != aSromVariable.mSampleProbPairs.mSamples.end(); tItr++)
    {
        for(auto& tOriginalRandomMaterial : tOriginalRandomMaterialSet)
        {
            Plato::srom::RandomMaterial tNewRandomMaterial;
            tNewRandomMaterial.blockID(aMaterial.blockID());
            tNewRandomMaterial.category(aMaterial.category());
            tNewRandomMaterial.materialID(aMaterial.materialID());

            auto tIndex = std::distance(tBeginItr, tItr);
            auto tUpdatedProbability = tOriginalRandomMaterial.probability()
                * aSromVariable.mSampleProbPairs.mProbabilities[tIndex];
            tNewRandomMaterial.probability(tUpdatedProbability);
            auto tValue = std::to_string(aSromVariable.mSampleProbPairs.mSamples[tIndex]);
            tNewRandomMaterial.append(aSromVariable.mTag, aSromVariable.mAttribute, tValue);

            auto tOriginalTags = tOriginalRandomMaterial.tags();
            for(auto& tTag : tOriginalTags)
            {
                auto tValue = tOriginalRandomMaterial.value(tTag);
                auto tAttribute = tOriginalRandomMaterial.attribute(tTag);
                tNewRandomMaterial.append(tTag, tAttribute, tValue);
            }

            aRandomMaterialSet.push_back(tNewRandomMaterial);
        }
    }
}

inline void append_random_material_properties
(const Plato::srom::Material& aMaterial,
 const std::vector<Plato::srom::SromVariable>& aSromVariables,
 std::vector<Plato::srom::RandomMaterial>& aRandomMaterialSet)
{
    for(auto& tSromVar : aSromVariables)
    {
        if(aRandomMaterialSet.empty())
        {
            Plato::srom::initialize_random_material_set(aMaterial, tSromVar, aRandomMaterialSet);
        }
        else
        {
            Plato::srom::update_random_material_set(aMaterial, tSromVar, aRandomMaterialSet);
        }
    }
}

inline void append_deterministic_material_properties
(const Plato::srom::Material &aMaterial,
 std::vector<Plato::srom::RandomMaterial>& aRandomMaterialSet)
{
    auto tDeterministicVars = aMaterial.deterministicVars();
    for(auto tRandomMaterial : aRandomMaterialSet)
    {
        for(auto &tVar : tDeterministicVars)
        {
            tRandomMaterial.append(tVar.mTag, tVar.mAttribute, tVar.mValue);
            tRandomMaterial.check();
        }
    }
}

inline void build_random_material_set
(const Plato::srom::Material& aMaterial,
 const std::vector<Plato::srom::SromVariable>& aSromVariables,
 std::vector<Plato::srom::RandomMaterial>& aRandomMaterials)
{
    aMaterial.check();
    if(aSromVariables.empty())
    {
        THROWERR("Build Random Material: Input set of SROM variables is empty.")
    }

    Plato::srom::append_random_material_properties(aMaterial, aSromVariables, aRandomMaterials);
    Plato::srom::append_deterministic_material_properties(aMaterial, aRandomMaterials);
}

inline bool generate_material_sroms(const std::vector<Plato::srom::Material>& aInput,
                                    std::vector<Plato::srom::RandomMaterialCase>& aOutput)
{
    if(aInput.empty())
    {
        THROWERR("Generate Material SROMS: Input container of materials is empty.")
    }

    std::vector<Plato::srom::Material> tRandomMaterials, tDeterministicMaterials;
    Plato::srom::split_random_and_deterministic_materials(aInput, tRandomMaterials, tDeterministicMaterials);

    for(auto& tMaterial : tRandomMaterials)
    {
        tMaterial.check();
        if(tMaterial.isDeterministic())
        {
            THROWERR("Generate Material SROMS: Detected a deterministic material while looping over random materials set.")
        }

        std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
        if(Plato::compute_sample_probability_pairs(tMaterial.randomVars(), tMySampleProbPairs) == false)
        {
            std::ostringstream tMsg;
            tMsg << "Generate Material SROMS: Failed to compute Sample-Probability pairs for material "
                << "with identification number '" << tMaterial.materialID() << "' in block with identification "
                << "number '" << tMaterial.blockID() << "'.";
            PRINTERR(tMsg.str().c_str());
        }

        std::vector<Plato::srom::RandomMaterial> tRandomMaterials;
        Plato::srom::build_random_material_set(tMaterial, tMySampleProbPairs, tRandomMaterials);
    }
    return (true);
}

}
// namespace srom

}
// namespace Plato

namespace UncertainMaterial
{

TEST(PlatoTest, ComputeSampleProbabilityPairs_HomogeneousElasticModulus_Beta)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.mTag = "elastic modulus";
    tMyRandomVar.mAttribute = "homogeneous";
    tMyRandomVar.mStatistics.mDistribution = "beta";
    tMyRandomVar.mStatistics.mMean = "1e9";
    tMyRandomVar.mStatistics.mUpperBound = "1e10";
    tMyRandomVar.mStatistics.mLowerBound = "1e8";
    tMyRandomVar.mStatistics.mStandardDeviation = "2e8";
    tMyRandomVar.mStatistics.mNumSamples = "3";
    std::vector<Plato::srom::RandomVariable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // SOLVE SROM PROBLEM
    std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
    Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs);

    // TEST RESULTS
    ASSERT_EQ(1u, tMySampleProbPairs.size());
    ASSERT_STREQ("elastic modulus", tMySampleProbPairs[0].mTag.c_str());
    ASSERT_STREQ("homogeneous", tMySampleProbPairs[0].mAttribute.c_str());
    ASSERT_EQ(3, tMySampleProbPairs[0].mSampleProbPairs.mNumSamples);
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mSamples.size());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mProbabilities.size());

    // TEST RESULTS
    double tSum = 0;
    double tTolerance = 1e-4;
    const Plato::srom::SampleProbabilityPairs& tSampleProbabilityPairs = tMySampleProbPairs[0].mSampleProbPairs;
    std::vector<double> tGoldSamples = {1312017818.6197019, 659073448.54796219, 656356599.33196139};
    std::vector<double> tGoldProbabilities = {0.43210087774761252, 0.31840063469163404, 0.24868340186995475};
    for(int tIndex = 0; tIndex < tSampleProbabilityPairs.mNumSamples; tIndex++)
    {
        tSum += tSampleProbabilityPairs.mProbabilities[tIndex];
        EXPECT_NEAR(tGoldSamples[tIndex], tSampleProbabilityPairs.mSamples[tIndex], tTolerance);
        EXPECT_NEAR(tGoldProbabilities[tIndex], tSampleProbabilityPairs.mProbabilities[tIndex], tTolerance);
    }
    tTolerance = 1e-2;
    EXPECT_NEAR(1.0, tSum, tTolerance);

    std::system("rm -f plato_cdf_output.txt");
    std::system("rm -f plato_srom_diagnostics.txt");
    std::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

}
// namespace UncertainMaterial
