/*
 * XMLGeneratorAnalyzeDefinesFileUtilities.cpp
 *
 *  Created on: May 20, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorAnalyzeDefinesFileUtilities.hpp"

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

    if(aUncertaintyMetaData.numPeformers == 0u)
        { THROWERR("Append Basic Attributes To Define XML File: Cannot assign zero MPI processes.") }
    auto tNumPerformers = XMLGen::compute_greatest_divisor(aRandomMetaData.numSamples(), aUncertaintyMetaData.numPeformers);
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
    auto tSample = aRandomMetaData.sample(0);
    auto tLoadCase = tSample.load();
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
std::vector<std::vector<std::vector<std::string>>>
prepare_tractions_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    auto tValues = XMLGen::allocate_random_tractions_container_for_define_xml_file(aRandomMetaData);
    auto tSamples = aRandomMetaData.samples();
    for(auto& tSample : tSamples)
    {
        auto tLoadCase = tSample.load();
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
(const std::vector<std::vector<std::vector<std::string>>>& aRandomTractions,
 pugi::xml_document& aDocument)
{
    if(aRandomTractions.empty())
    {
        return;
    }

    std::vector<std::string> tValidAxis = {"X", "Y", "Z"};
    for(auto tLoadItr = aRandomTractions.begin(); tLoadItr != aRandomTractions.end(); ++tLoadItr)
    {
        auto tLoadIndex = std::distance(aRandomTractions.begin(), tLoadItr);
        for(auto tDimItr = tLoadItr->begin(); tDimItr != tLoadItr->end(); ++tDimItr)
        {
            auto tDimIndex = std::distance(tLoadItr->begin(), tDimItr);
            auto tTag = std::string("RandomLoad") + std::to_string(tLoadIndex) + "_" + tValidAxis[tDimIndex] + "-Axis";
            auto tValues = XMLGen::transform_tokens(tDimItr.operator*());
            XMLGen::append_attributes("Array", {"name", "type", "value"}, {tTag, "real", tValues}, aDocument);
        }
    }
}
// function append_tractions_to_define_xml_file
/******************************************************************************/

/******************************************************************************/
std::unordered_map<std::string, std::vector<std::string>> /*!< material property tag to samples map */
prepare_material_properties_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    std::unordered_map<std::string, std::vector<std::string>> tMatNameToSamplesMap;
    auto tSamples = aRandomMetaData.samples();
    for(auto& tSample : tSamples)
    {
        auto tBlockIDs = tSample.materialBlockIDs();
        for(auto& tID : tBlockIDs)
        {
            auto tMaterial = tSample.material(tID);
            auto tTags = tMaterial.tags();
            for(auto& tTag : tTags)
            {
                auto tName = tTag + " " + "blockID-" + tID;
                tMatNameToSamplesMap[tName].push_back(tMaterial.property(tTag));
            }
        }
    }

    return (tMatNameToSamplesMap);
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
    auto tTractionValues = XMLGen::prepare_tractions_for_define_xml_file(aRandomMetaData);
    XMLGen::append_tractions_to_define_xml_file(tTractionValues, tDocument);
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
