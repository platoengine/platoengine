/*
 * XMLGeneratorDefinesFileUtilities.cpp
 *
 *  Created on: May 20, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorDefinesFileUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
void append_basic_attributes_to_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData,
 const XMLGen::UncertaintyMetaData& aUncertaintyMetaData,
 pugi::xml_document& aDocument)
{
    if(aRandomMetaData.numSamples() == 0u)
        { THROWERR("Append Basic Attributes To Define XML File: Cannot assign zero samples.") }
    auto tNumSamplesString = std::to_string(aRandomMetaData.numSamples());
    XMLGen::append_attributes("Define", {"name", "type", "value"}, {"NumSamples", "int", tNumSamplesString}, aDocument);

    if(aUncertaintyMetaData.numPerformers == 0u)
        { THROWERR("Append Basic Attributes To Define XML File: Cannot assign zero MPI processes.") }
    auto tNumPerformers = XMLGen::compute_greatest_divisor(aRandomMetaData.numSamples(), aUncertaintyMetaData.numPerformers);
    auto tNumPerformersString = std::to_string(tNumPerformers);

    XMLGen::append_attributes("Define", {"name", "type", "value"}, {"NumPerformers", "int", tNumPerformersString}, aDocument);
    XMLGen::append_attributes("Define", {"name", "type", "value"}, {"NumSamplesPerPerformer", "int", "{NumSamples/NumPerformers}"}, aDocument);

    XMLGen::append_attributes("Define", {"name", "type", "from", "to"}, {"Samples", "int", "0", "{NumSamples-1}"}, aDocument);
    XMLGen::append_attributes("Define", {"name", "type", "from", "to"}, {"Performers", "int", "0", "{NumPerformers-1}"}, aDocument);
    XMLGen::append_attributes("Define", {"name", "type", "from", "to"}, {"PerformerSamples", "int", "0", "{NumSamplesPerPerformer-1}"}, aDocument);
}
// function append_basic_attributes_to_define_xml_file
/******************************************************************************/

/******************************************************************************/
std::vector<std::vector<std::vector<std::string>>>
allocate_random_tractions_container_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    std::vector<std::vector<std::vector<std::string>>> tOutput;
    auto tLoadCase = aRandomMetaData.loadcase();
    for(auto& tLoad : tLoadCase.loads)
    {
        if(tLoad.mIsRandom)
        {
            // allocate storage
            tOutput.push_back({});
            auto tLoadIndex = &tLoad - &tLoadCase.loads[0];
            for (auto &tValue : tLoad.values)
            {
                tOutput[tLoadIndex].push_back({});
            }
        }
    }
    return (tOutput);
}
// function allocate_random_tractions_container_for_define_xml_file
/******************************************************************************/

/******************************************************************************/
std::vector<std::string>
prepare_probabilities_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    std::vector<std::string> tProbabilities;
    auto tSamples = aRandomMetaData.samples();
    for(auto& tSample : tSamples)
    {
        // append probabilities
        tProbabilities.push_back(tSample.probability());
    }
    return tProbabilities;
}
// function prepare_probabilities_for_define_xml_file
/******************************************************************************/

/******************************************************************************/
std::unordered_map<std::string, std::vector<std::string>>
return_random_tractions_tags_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    if(aRandomMetaData.samples().empty())
    {
        THROWERR("Return Random Tractions Tags For Define Xml File: Samples vector is empty.")
    }

    // traction load index to tags map, i.e. map<load index, vector<tags>>
    std::unordered_map<std::string, std::vector<std::string>> tOutput;

    XMLGen::ValidAxesKeys tValidDofs;
    auto tLoadCase = aRandomMetaData.loadcase();
    for(auto& tLoad : tLoadCase.loads)
    {
        auto tLoadTagLower = Plato::tolower(tLoad.type);
        auto tIsTractionLoad = tLoadTagLower == "traction";
        if(tLoad.mIsRandom && tIsTractionLoad)
        {
            auto tLoadIndex = &tLoad - &tLoadCase.loads[0];
            auto tLoadIndexString = std::to_string(tLoadIndex);
            for (auto &tValue : tLoad.values)
            {
                auto tDimIndex = &tValue - &tLoad.values[0];
                auto tItr = tValidDofs.mKeys.find(tDimIndex);
                if(tItr == tValidDofs.mKeys.end())
                {
                    THROWERR(std::string("Return Random Tractions Tags for Define XML File: Invalid dimension key '")
                        + std::to_string(tDimIndex) + "'. Valid dimensions are: 1D, 2D, and 3D.")
                }
                auto tTag = tLoadTagLower + " load-id-" + tLoadIndexString + " " + tItr->second + "-axis";
                tOutput[tLoadIndexString].push_back(tTag);
            }
        }
    }

    return (tOutput);
}
// function return_random_tractions_tags_for_define_xml_file
/******************************************************************************/

/******************************************************************************/
std::vector<std::vector<std::vector<std::string>>>
prepare_tractions_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    auto tValues = XMLGen::allocate_random_tractions_container_for_define_xml_file(aRandomMetaData);
    auto tSamples = aRandomMetaData.samples();
    for(auto& tSample : tSamples)
    {
        auto tLoadCase = tSample.loadcase();
        for(auto& tLoad : tLoadCase.loads)
        {
            if(tLoad.mIsRandom)
            {
                // append values
                auto tLoadIndex = &tLoad - &tLoadCase.loads[0];
                for(auto& tValue : tLoad.values)
                {
                    auto tDimIndex = &tValue - &tLoad.values[0];
                    tValues[tLoadIndex][tDimIndex].push_back(tValue);
                }
            }
        }
    }
    return tValues;
}
// function prepare_tractions_for_define_xml_file
/******************************************************************************/

/******************************************************************************/
void append_probabilities_to_define_xml_file
(const std::vector<std::string>& aProbabilities,
 pugi::xml_document& aDocument)
{
    if(aProbabilities.empty())
    {
        THROWERR("Append Probabilities To Define XML File: Input probability container is empty.")
    }

    auto tValues = XMLGen::transform_tokens(aProbabilities);
    XMLGen::append_attributes("Array", {"name", "type", "value"}, {"Probabilities", "real", tValues}, aDocument);
}
// function append_probabilities_to_define_xml_file
/******************************************************************************/

/******************************************************************************/
void append_tractions_to_define_xml_file
(const std::unordered_map<std::string, std::vector<std::string>>& aTags,
 const std::vector<std::vector<std::vector<std::string>>>& aValues,
 pugi::xml_document& aDocument)
{
    if(aValues.empty())
    {
        return;
    }

    for(auto tLoadItr = aValues.begin(); tLoadItr != aValues.end(); ++tLoadItr)
    {
        auto tLoadIndex = std::distance(aValues.begin(), tLoadItr);
        auto tLoadIndexString = std::to_string(tLoadIndex);
        for(auto tDimItr = tLoadItr->begin(); tDimItr != tLoadItr->end(); ++tDimItr)
        {
            auto tDimIndex = std::distance(tLoadItr->begin(), tDimItr);
            auto tTagItr = aTags.find(tLoadIndexString);
            if(tTagItr == aTags.end())
            {
                THROWERR(std::string("Append Tractions To Define XML File: ") + "Did not find load index '"
                    + tLoadIndexString + "' in Load Index to Tags Map. ")
            }
            auto tTag = tTagItr->second[tDimIndex];
            auto tValues = XMLGen::transform_tokens(tDimItr.operator*());
            XMLGen::append_attributes("Array", {"name", "type", "value"}, {tTag, "real", tValues}, aDocument);
        }
    }
}
// function append_tractions_to_define_xml_file
/******************************************************************************/

/******************************************************************************/
std::unordered_map<std::string, std::vector<std::string>>
return_material_property_tags_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    if(aRandomMetaData.samples().empty())
    {
        THROWERR("Return Material Property Tags For Define XML File: Samples vector is empty.")
    }

    std::unordered_map<std::string, std::vector<std::string>> tBlockIdToTagsMap;

    auto tSample = aRandomMetaData.sample(0);
    auto tBlockIDs = tSample.materialBlockIDs();
    for(auto& tID : tBlockIDs)
    {
        auto tMaterial = tSample.material(tID);
        auto tMaterialPropertiesTags = tMaterial.tags();
        for(auto& tMaterialPropertyTag : tMaterialPropertiesTags)
        {
            auto tArgumentNameTag = tMaterialPropertyTag + " block-id-" + tID;
            tBlockIdToTagsMap[tID].push_back(tArgumentNameTag);
        }
    }

    return (tBlockIdToTagsMap);
}
// function return_material_property_tags_for_define_xml_file
/******************************************************************************/

/******************************************************************************/
std::unordered_map<std::string, std::vector<std::string>> /*!< material property tag to samples map */
prepare_material_properties_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    std::unordered_map<std::string, std::vector<std::string>> tMatTagToSamplesMap;
    auto tSamples = aRandomMetaData.samples();
    for(auto& tSample : tSamples)
    {
        auto tBlockIDs = tSample.materialBlockIDs();
        for(auto& tID : tBlockIDs)
        {
            auto tMaterial = tSample.material(tID);
            auto tMaterialPropertyTags = tMaterial.tags();
            for(auto& tMaterialPropertyTag : tMaterialPropertyTags)
            {
                auto tTag = tMaterialPropertyTag + " block-id-" + tID;
                tMatTagToSamplesMap[tTag].push_back(tMaterial.property(tMaterialPropertyTag));
            }
        }
    }

    return (tMatTagToSamplesMap);
}
// function prepare_material_properties_for_define_xml_file
/******************************************************************************/

/******************************************************************************/
void append_material_properties_to_define_xml_file
(const std::unordered_map<std::string, std::vector<std::string>>& aMaterials,
 pugi::xml_document& aDocument)
{
    if(aMaterials.empty())
    {
        return;
    }

    for(auto& tPair : aMaterials)
    {
        auto tValues = XMLGen::transform_tokens(tPair.second);
        XMLGen::append_attributes("Array", {"name", "type", "value"}, {tPair.first, "real", tValues}, aDocument);
    }
}
// function append_material_properties_to_define_xml_file
/******************************************************************************/

/******************************************************************************/
void write_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData,
 const XMLGen::UncertaintyMetaData& aUncertaintyMetaData)
{
    pugi::xml_document tDocument;
    XMLGen::append_basic_attributes_to_define_xml_file(aRandomMetaData, aUncertaintyMetaData, tDocument);

    auto tTractionTags = XMLGen::return_random_tractions_tags_for_define_xml_file(aRandomMetaData);
    auto tTractionValues = XMLGen::prepare_tractions_for_define_xml_file(aRandomMetaData);
    XMLGen::append_tractions_to_define_xml_file(tTractionTags, tTractionValues, tDocument);

    auto tMaterialValues = XMLGen::prepare_material_properties_for_define_xml_file(aRandomMetaData);
    XMLGen::append_material_properties_to_define_xml_file(tMaterialValues, tDocument);

    auto tProbabilities = XMLGen::prepare_probabilities_for_define_xml_file(aRandomMetaData);
    XMLGen::append_probabilities_to_define_xml_file(tProbabilities, tDocument);

    tDocument.save_file("defines.xml", "  ");
}
// function write_define_xml_file
/******************************************************************************/

}
// namespace XMLGen
