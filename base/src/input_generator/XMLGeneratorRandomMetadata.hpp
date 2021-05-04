/*
 * XMLGeneratorRandomMetadata.hpp
 *
 *  Created on: May 14, 2020
 */

#pragma once

#include "XMLGeneratorMaterialMetadata.hpp"
#include "XMLGeneratorBoundaryMetadata.hpp"
#include <unordered_map>

namespace XMLGen
{


/*!< map between block identification number and material metadata, i.e. map<block id, material metadata> */
using MaterialSet = std::unordered_map<std::string, XMLGen::Material>;
/******************************************************************************//**
 * \struct RandomSample
 * \brief Random sample metadata.
**********************************************************************************/
struct RandomSample
{
private:
    std::string mProbability; /*!< sample probability */
    std::pair<XMLGen::MaterialSet, XMLGen::LoadCase> mSample;  /*!< random sample */
    std::pair<bool, bool> mSampleDrawn = std::make_pair(false,false); /*!< indicate if sample category was drawn */

    // Note: the std::pair can be replaced by an std::tuple in the future if
    // another random variable category is added, e.g. boundary conditions.
public:
    /******************************************************************************//**
     * \struct loadSampleDrawn
     * \brief Return true if a load sample was drawn.
     * \return flag
    **********************************************************************************/
    bool loadSampleDrawn() const
    {
        return (mSampleDrawn.second);
    }

    /******************************************************************************//**
     * \struct loadSampleDrawn
     * \brief Return true if a material sample was drawn.
     * \return flag
    **********************************************************************************/
    bool materialSampleDrawn() const
    {
        return (mSampleDrawn.first);
    }

    /******************************************************************************//**
     * \struct probability
     * \brief Return sample's probability.
     * \return probability
    **********************************************************************************/
    std::string probability() const
    {
        return mProbability;
    }

    /******************************************************************************//**
     * \struct probability
     * \brief Set sample's probability.
     * \param [in] aProbability sample's probability
    **********************************************************************************/
    void probability(const std::string& aProbability)
    {
        mProbability = aProbability;
    }

    /******************************************************************************//**
     * \struct load
     * \brief Return constant reference to load case metadata.
     * \return load case metadata
    **********************************************************************************/
    const XMLGen::LoadCase& loadcase() const
    {
        return (mSample.second);
    }

    /******************************************************************************//**
     * \struct load
     * \brief Set load case metadata.
     * \param [in] aLoadCase load case metadata
    **********************************************************************************/
    void load(const XMLGen::LoadCase& aLoadCase)
    {
        mSampleDrawn.second = true;
        mSample.second = aLoadCase;
    }

    /******************************************************************************//**
     * \struct material
     * \brief Return constant reference to material metadata.
     * \return material metadata
    **********************************************************************************/
    const XMLGen::Material& material(const std::string& aBlockID) const
    {
        auto tIterator = mSample.first.find(aBlockID);
        if (tIterator == mSample.first.end())
        {
            THROWERR(std::string("Random Sample: Material block with identification number '") + aBlockID + "' is not defined.")
        }
        return (tIterator->second);
    }

    /******************************************************************************//**
     * \struct material
     * \brief Set materials metadata.
     * \param [in] aMaterial materials metadata
    **********************************************************************************/
    void material(const MaterialSet& aMaterial)
    {
        mSampleDrawn.first = true;
        mSample.first = aMaterial;
    }

    /******************************************************************************//**
     * \struct material
     * \brief Set materials metadata.
     * \param [in] aMaterial materials metadata
    **********************************************************************************/
    std::vector<std::string> materialBlockIDs() const
    {
        std::vector<std::string> tIDs;
        for(auto& tPair : mSample.first)
        {
            tIDs.push_back(tPair.first);
        }
        return tIDs;
    }
};
// struct RandomSample

/*!< load case - probability pair */
using RandomLoadCase = std::pair<double, XMLGen::LoadCase>;
/*!< random material case probability - map between block identification number and \n
 *   material metadata pair, i.e. pair<probability, map<block id, material metadata>> */
using RandomMaterialCase = std::pair<double, XMLGen::MaterialSet>;
/******************************************************************************//**
 * \struct RandomMetaData
 * \brief Random use case metadata.
**********************************************************************************/
struct RandomMetaData
{
private:
    std::vector<XMLGen::RandomSample> mSamples; /*!< set of random samples */
    std::vector<XMLGen::RandomLoadCase> mRandomLoads; /*!< load case - probability pairs */
    std::vector<XMLGen::RandomMaterialCase> mRandomMaterials; /*!< maps between block id and material metadata - probability pair */

private:
    /******************************************************************************//**
     * \fn check
     * \brief Check input material metadata.
     * \param [in] aMaterial material metadata
    **********************************************************************************/
    void check(const XMLGen::Material& aMaterial)
    {
        if(aMaterial.id().empty())
        {
            THROWERR("Random Use Case: Material identification number is empty, i.e. material id is not defined.")
        }
        if(aMaterial.materialModel().empty())
        {
            THROWERR("Random Use Case: Material category is empty, i.e. category is not defined.")
        }
        if(aMaterial.tags().empty())
        {
            THROWERR("Random Use Case: Material properties are empty, i.e. properties are not defined.")
        }
    }

    /******************************************************************************//**
     * \fn check
     * \brief Check load case metadata.
     * \param [in] aLoadCase load case metadata
    **********************************************************************************/
    void check(const XMLGen::LoadCase& aLoadCase)
    {
        if(aLoadCase.id.empty())
        {
            THROWERR("Random Use Case: Load case identification number is empty, i.e. load case id is not defined.")
        }
        if(aLoadCase.loads.empty())
        {
            THROWERR("Random Use Case: Load case loads container is empty, i.e. set of loads is not defined.")
        }
    }

    /******************************************************************************//**
     * \fn finalizeLoadUseCase
     * \brief Finalize stochastic load use case.
    **********************************************************************************/
    void finalizeLoadUseCase()
    {
        for(auto& tLoadSample : mRandomLoads)
        {
            XMLGen::RandomSample tSample;
            tSample.probability(Plato::to_string(tLoadSample.first));
            tSample.load(tLoadSample.second);
            mSamples.push_back(tSample);
        }
    }

    /******************************************************************************//**
     * \fn finalizeMaterialUseCase
     * \brief Finalize stochastic material use case.
    **********************************************************************************/
    void finalizeMaterialUseCase()
    {
        for(auto& tMaterialSample : mRandomMaterials)
        {
            XMLGen::RandomSample tSample;
            tSample.probability(Plato::to_string(tMaterialSample.first));
            tSample.material(tMaterialSample.second);
            mSamples.push_back(tSample);
        }
    }

    /******************************************************************************//**
     * \fn finalizeMixedUseCase
     * \brief Finalize mixed stochastic use case.
    **********************************************************************************/
    void finalizeMixedUseCase()
    {
        for(auto& tLoadSample : mRandomLoads)
        {
            for(auto& tMaterialSample : mRandomMaterials)
            {
                XMLGen::RandomSample tSample;
                auto tProbability = tLoadSample.first * tMaterialSample.first;
                tSample.probability(Plato::to_string(tProbability));
                tSample.load(tLoadSample.second);
                tSample.material(tMaterialSample.second);
                mSamples.push_back(tSample);
            }
        }
    }

public:
    /******************************************************************************//**
     * \fn numSamples
     * \brief Return number of samples.
     * \return number of samples
    **********************************************************************************/
    size_t numSamples() const
    {
        return (mSamples.size());
    }

    /******************************************************************************//**
     * \fn clear
     * \brief Erases all elements from the random containers. After this call, all \n
     *   random containers will have a size of zero.
    **********************************************************************************/
    void clear()
    {
        mSamples.clear();
        mRandomLoads.clear();
        mRandomMaterials.clear();
    }

    /******************************************************************************//**
     * \fn finalize
     * \brief Perform tensor product and compute the final set of sample-probability pairs.
    **********************************************************************************/
    void finalize()
    {
        if(mRandomLoads.empty() && mRandomMaterials.empty())
        {
            THROWERR("Random MetaData: Random material and load use cases are not defined, i.e. both lists are empty.")
        }

        if(!mRandomLoads.empty() && !mRandomMaterials.empty())
        {
            this->finalizeMixedUseCase();
        }
        else if(!mRandomLoads.empty() && mRandomMaterials.empty())
        {
            this->finalizeLoadUseCase();
        }
        else if(mRandomLoads.empty() && !mRandomMaterials.empty())
        {
            this->finalizeMaterialUseCase();
        }
    }

    /******************************************************************************//**
     * \fn append
     * \brief Append load set metadata.
     * \param [in] aLoadSet load set metadata
    **********************************************************************************/
    void append(const XMLGen::RandomLoadCase& aLoadset)
    {
        this->check(aLoadset.second);
        mRandomLoads.push_back(aLoadset);
    }

    /******************************************************************************//**
     * \fn append
     * \brief Append material set metadata.
     * \param [in] aMaterialSet material set metadata
    **********************************************************************************/
    void append(const XMLGen::RandomMaterialCase& aMaterialSet)
    {
        if(aMaterialSet.second.empty())
        {
            THROWERR("Random MetaData: Block identification number to material metadata is empty.")
        }

        for(auto& tPair : aMaterialSet.second)
        {
            if(tPair.first.empty())
            {
                THROWERR(std::string("Random MetaData: Block identification number for material with identification number '") + "' is empty.")
            }
            this->check(tPair.second);
        }
        mRandomMaterials.push_back(aMaterialSet);
    }

    /******************************************************************************//**
     * \fn sample
     * \brief Return const reference to sample's metadata.
     * \param [in] aIndex sample index
    **********************************************************************************/
    const XMLGen::RandomSample& sample(const size_t& aIndex) const
    {
        if(aIndex >= mSamples.size())
        {
            THROWERR(std::string("Random MetaData: Input index '") + std::to_string(aIndex)
                + "' is greater or equal to the size of the samples list. The samples list has size '"
                + std::to_string(mSamples.size()) + ".")
        }
        return (mSamples[aIndex]);
    }

    /******************************************************************************//**
     * \fn samples
     * \brief Return list of random samples.
     * \return list of random samples
    **********************************************************************************/
    std::vector<XMLGen::RandomSample> samples() const
    {
        return mSamples;
    }

    /******************************************************************************//**
     * \fn materials
     * \brief Return random materials.
     * \return list of random materials
    **********************************************************************************/
    std::vector<XMLGen::Material> materials() const
    {
        if(mSamples.empty())
        {
            THROWERR("Random MetaData: Requested random materials; however, samples container is empty.")
        }
        std::vector<XMLGen::Material> tMaterials;
        auto tBlockIDs = mSamples[0].materialBlockIDs();
        for(auto& tID : tBlockIDs)
        {
            tMaterials.push_back(mSamples[0].material(tID));
        }
        return tMaterials;
    }

    /******************************************************************************//**
     * \fn loadcase
     * \brief Return random load case.
     * \return list of random load case
    **********************************************************************************/
    XMLGen::LoadCase loadcase() const
    {
        if(mSamples.empty())
        {
            THROWERR("Random MetaData: Requested random load cases; however, samples container is empty.")
        }
        return (mSamples[0].loadcase());
    }

    /******************************************************************************//**
     * \fn samplesDrawn
     * \brief Return true if samples were drawn.
     * \return flag
    **********************************************************************************/
    bool samplesDrawn() const
    {
        auto tIsRandomUseCase = mSamples.empty() ? false : true;
        return tIsRandomUseCase;
    }

    /******************************************************************************//**
     * \fn loadSamplesDrawn
     * \brief Return true if random load samples were drawn.
     * \return flag
    **********************************************************************************/
    bool loadSamplesDrawn() const
    {
        if(mSamples.empty())
        {
            THROWERR("Random MetaData: Samples container is empty.")
        }
        return (mSamples[0].loadSampleDrawn());
    }

    /******************************************************************************//**
     * \fn materialSamplesDrawn
     * \brief Return true if random material samples were drawn.
     * \return flag
    **********************************************************************************/
    bool materialSamplesDrawn() const
    {
        if(mSamples.empty())
        {
            THROWERR("Random MetaData: Samples container is empty.")
        }
        return (mSamples[0].materialSampleDrawn());
    }

    /******************************************************************************//**
     * \fn empty
     * \brief Return 'true' if random metadata is empty; else, return 'false'.
     * \return flag
    **********************************************************************************/
    bool empty() const
    {
        return mSamples.empty();
    }
};
// struct RandomMetaData

}
// namespace XMLGen
