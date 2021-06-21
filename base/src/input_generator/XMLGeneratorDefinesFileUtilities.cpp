/*
 * XMLGeneratorDefinesFileUtilities.cpp
 *
 *  Created on: May 20, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoAnalyzeProblem.hpp"
#include "XMLGeneratorDefinesFileUtilities.hpp"
#include "XMLGeneratorAnalyzeLoadTagFunctionInterface.hpp"

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

    XMLGen::append_attributes("Array", {"name", "type", "from", "to"}, {"Samples", "int", "0", "{NumSamples-1}"}, aDocument);
    XMLGen::append_attributes("Array", {"name", "type", "from", "to"}, {"Performers", "int", "0", "{NumPerformers-1}"}, aDocument);
    XMLGen::append_attributes("Array", {"name", "type", "from", "to"}, {"PerformerSamples", "int", "0", "{NumSamplesPerPerformer-1}"}, aDocument);
}
// function append_basic_attributes_to_define_xml_file
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

    // traction load index to tags map, i.e. map<load_identifier, vector<load_component_tags>>
    std::unordered_map<std::string, std::vector<std::string>> tOutput;

    XMLGen::ValidAxesKeys tValidDofs;
    XMLGen::LoadTag tLoadNameFuncInterface;

    auto tLoadCase = aRandomMetaData.loadcase();
    for(auto& tLoad : tLoadCase.loads)
    {
        auto tLoadTagLower = Plato::tolower(tLoad.type());
        auto tIsTractionLoad = tLoadTagLower == "traction";
        auto tLoadIdentifier = tLoadNameFuncInterface.call(tLoad);
        if(tLoad.is_random() && tIsTractionLoad)
        {
            for (auto &tValue : tLoad.load_values())
            {
                auto tDimIndex = &tValue - &tLoad.load_values()[0];
                auto tDimension = tValidDofs.value(tDimIndex);
                if(tDimension.empty())
                {
                    THROWERR(std::string("Return Random Tractions Tags for Define XML File: Invalid dimension key '")
                        + std::to_string(tDimIndex) + "'. Valid dimensions are: 1D, 2D, and 3D.")
                }
                auto tTag = tLoadTagLower + "_load_id_" + tLoad.id() + "_" + tDimension + "_axis";
                tOutput[tLoadIdentifier].push_back(tTag);
            }
        }
    }

    return (tOutput);
}
// function return_random_tractions_tags_for_define_xml_file
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
    XMLGen::append_attributes("Array", {"name", "type", "values"}, {"Probabilities", "real", tValues}, aDocument);
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
        auto tTagItr = aTags.find(tLoadIndexString);
        if(tTagItr == aTags.end())
        {
            THROWERR(std::string("Append Tractions To Define XML File: ") + "Did not find load identifier '"
                + tLoadIndexString + "' in map from load identifier to load component tags.")
        }

        for(auto tDimItr = tLoadItr->begin(); tDimItr != tLoadItr->end(); ++tDimItr)
        {
            auto tDimIndex = std::distance(tLoadItr->begin(), tDimItr);
            auto tTag = tTagItr->second[tDimIndex];
            auto tValues = XMLGen::transform_tokens(tDimItr.operator*());
            XMLGen::append_attributes("Array", {"name", "type", "values"}, {tTag, "real", tValues}, aDocument);
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
            auto tArgumentNameTag = tMaterialPropertyTag + "_block_id_" + tID;
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
                auto tTag = tMaterialPropertyTag + "_block_id_" + tID;
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
        XMLGen::append_attributes("Array", {"name", "type", "values"}, {tPair.first, "real", tValues}, aDocument);
    }
}
// function append_material_properties_to_define_xml_file
/******************************************************************************/

/******************************************************************************/
std::map<std::string, std::vector<std::vector<std::string>>>
allocate_map_from_random_load_identifier_to_load_samples
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    std::map<std::string, std::vector<std::vector<std::string>>> tOutput;

    auto tLoadCase = aRandomMetaData.loadcase();
    XMLGen::LoadTag tIdentifierInterface;
    for(auto& tLoad : tLoadCase.loads)
    {
        if(tLoad.is_random())
        {
            auto tIdentifier = tIdentifierInterface.call(tLoad);
            for (size_t i=0; i<tLoad.load_values().size(); ++i)
            {
                tOutput[tIdentifier].push_back({});
            }
        }
    }
    return (tOutput);
}
// function allocate_map_from_random_load_identifier_to_load_samples
/******************************************************************************/

/******************************************************************************/
std::map<std::string, std::vector<std::vector<std::string>>>
prepare_tractions_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    auto tMapFromIdentifierToLoadValues =
        XMLGen::allocate_map_from_random_load_identifier_to_load_samples(aRandomMetaData);

    auto tSamples = aRandomMetaData.samples();
    XMLGen::LoadTag tIdentifierInterface;
    for(auto& tSample : tSamples)
    {
        auto tLoadCase = tSample.loadcase();
        for(auto& tLoad : tLoadCase.loads)
        {
            if(tLoad.is_random())
            {
                // append random load values for this sample
                auto tIdentifier = tIdentifierInterface.call(tLoad);
                for(auto& tValue : tLoad.load_values())
                {
                    auto tDimIndex = &tValue - &tLoad.load_values()[0];
                    tMapFromIdentifierToLoadValues[tIdentifier][tDimIndex].push_back(tValue);
                }
            }
        }
    }
    return (tMapFromIdentifierToLoadValues);
}
// function prepare_tractions_for_define_xml_file
/******************************************************************************/

/******************************************************************************/
void append_tractions_to_define_xml_file
(const std::unordered_map<std::string, std::vector<std::string>>& aTags,
 const std::map<std::string, std::vector<std::vector<std::string>>>& aValues,
 pugi::xml_document& aDocument)
{
    if(aValues.empty())
    {
        return;
    }

    for(auto& tPair : aValues)
    {
        auto tTagsItr = aTags.find(tPair.first);
        if(tTagsItr == aTags.end())
        {
            THROWERR(std::string("Append Tractions To Define XML File: ") + "Did not find load identifier '"
                + tPair.first + "' in map from load identifier to load component tags.")
        }

        for(auto& tLoadComponentSamples : tPair.second)
        {
            auto tDimIndex = &tLoadComponentSamples - &tPair.second[0];
            auto tTag = tTagsItr->second[tDimIndex];
            auto tValues = XMLGen::transform_tokens(tLoadComponentSamples);
            XMLGen::append_attributes("Array", {"name", "type", "values"}, {tTag, "real", tValues}, aDocument);
        }
    }
}
// function append_tractions_to_define_xml_file
/******************************************************************************/

/******************************************************************************/
void write_define_xml_file
(const XMLGen::InputData& aMetaData)
{
    pugi::xml_document tDocument;
    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        add_robust_optimization_data_to_define_xml_file(aMetaData, tDocument);
    }
    if(is_shape_optimization_problem(aMetaData))
    {
        add_shape_optimization_data_to_define_xml_file(aMetaData, tDocument);
    }
    tDocument.save_file("defines.xml", "  ");
}
// function write_define_xml_file
/******************************************************************************/

/******************************************************************************/
void add_robust_optimization_data_to_define_xml_file
(const XMLGen::InputData& aMetaData,
 pugi::xml_document &aDocument)
{
    auto &tRandomMetaData = aMetaData.mRandomMetaData;
    auto &tUncertaintyMetaData = aMetaData.m_UncertaintyMetaData;

    XMLGen::append_basic_attributes_to_define_xml_file(tRandomMetaData, tUncertaintyMetaData, aDocument);

    auto tTractionTags = XMLGen::return_random_tractions_tags_for_define_xml_file(tRandomMetaData);
    auto tTractionValues = XMLGen::prepare_tractions_for_define_xml_file(tRandomMetaData);
    XMLGen::append_tractions_to_define_xml_file(tTractionTags, tTractionValues, aDocument);

    auto tMaterialValues = XMLGen::prepare_material_properties_for_define_xml_file(tRandomMetaData);
    XMLGen::append_material_properties_to_define_xml_file(tMaterialValues, aDocument);

    auto tProbabilities = XMLGen::prepare_probabilities_for_define_xml_file(tRandomMetaData);
    XMLGen::append_probabilities_to_define_xml_file(tProbabilities, aDocument);
}
// function add_robust_optimization_data_to_define_xml_file
/******************************************************************************/

/******************************************************************************/
void add_shape_optimization_data_to_define_xml_file
(const XMLGen::InputData& aMetaData,
 pugi::xml_document &aDocument)
{
    pugi::xml_node tTmpNode = aDocument.append_child("Define");
    tTmpNode.append_attribute("name") = "NumParameters";
    tTmpNode.append_attribute("value") = aMetaData.optimization_parameters().num_shape_design_variables().c_str();
    tTmpNode = aDocument.append_child("Array");
    tTmpNode.append_attribute("name") = "Parameters";
    tTmpNode.append_attribute("type") = "int";
    tTmpNode.append_attribute("from") = "1";
    tTmpNode.append_attribute("to") = "{NumParameters}";
}

// function add_shape_optimization_data_to_define_xml_file
/******************************************************************************/

}
// namespace XMLGen
