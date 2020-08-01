/*
 * XMLGeneratorParseObjective.cpp
 *
 *  Created on: Jun 17, 2020
 */

#include <iostream>
#include <algorithm>

#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParseObjective.hpp"

namespace XMLGen
{

void ParseObjective::allocate()
{
    mTags.clear();

    mTags.insert({ "type", { { {"type"}, ""}, "" } });
    mTags.insert({ "code", { { {"code"}, ""}, "" } });
    mTags.insert({ "name", { { {"name"}, ""}, "" } });
    mTags.insert({ "weight", { { {"weight"}, ""}, "" } });
    mTags.insert({ "load ids", { { {"load", "ids"}, ""}, "" } });
    mTags.insert({ "stress_limit", { { {"stress_limit"}, ""}, "" } });
    mTags.insert({ "ls tet type", { { {"ls","tet","type"}, ""}, "" } });
    mTags.insert({ "number ranks", { { {"number", "ranks"}, ""}, "" } });
    mTags.insert({ "multi load case", { { {"multi","load","case"}, ""}, "" } });
    mTags.insert({ "number processors", { { {"number", "processors"}, ""}, "" } });
    mTags.insert({ "load case weights", { { {"load","case","weights"}, ""}, "" } });
    mTags.insert({ "normalize objective", { { {"normalize","objective"}, ""}, "" } });
    mTags.insert({ "frf match nodesets", { { {"frf","match","nodesets"}, ""}, "" } });
    mTags.insert({ "reference frf file", { { {"reference","frf","file"}, ""}, "" } });
    mTags.insert({ "distribute objective", { { {"distribute","objective"}, ""}, "" } });
    mTags.insert({ "output for plotting", { { {"output", "for", "plotting"}, ""}, "" } });
    mTags.insert({ "raleigh damping beta", { { {"raleigh","damping","beta"}, ""}, "" } });
    mTags.insert({ "raleigh damping alpha", { { {"raleigh","damping","alpha"}, ""}, "" } });
    mTags.insert({ "relative_stress_limit", { { {"relative_stress_limit"}, ""}, "" } });
    mTags.insert({ "scmm_mass_criterion_weight", { { {"scmm_mass_criterion_weight"}, ""}, "" } });
    mTags.insert({ "scmm_stress_criterion_weight", { { {"scmm_stress_criterion_weight"}, ""}, "" } });
    mTags.insert({ "scmm_penalty_upper_bound", { { {"scmm_penalty_upper_bound"}, ""}, "" } });
    mTags.insert({ "scmm_initial_penalty", { { {"scmm_initial_penalty"}, ""}, "" } });
    mTags.insert({ "complex error measure", { { {"complex","error","measure"}, ""}, "" } });
    mTags.insert({ "analyze new workflow", { { {"analyze", "new", "workflow"}, ""}, "" } });
    mTags.insert({ "boundary condition ids", { { {"boundary","condition", "ids"}, ""}, "" } });
    mTags.insert({ "weightmass scale factor", { { {"weightmass","scale","factor"}, ""}, "" } });
    mTags.insert({ "scmm_constraint_exponent", { { {"scmm_constraint_exponent"}, ""}, "" } });
    mTags.insert({ "analysis solver tolerance", { { {"analysis","solver","tolerance"}, ""}, "" } });
    mTags.insert({ "scmm_penalty_expansion_factor", { { {"scmm_penalty_expansion_factor"}, ""}, "" } });

    mTags.insert({ "performer", { { {"performer"}, ""}, "" } });
    mTags.insert({ "penalty power", { { {"penalty", "power"}, ""}, "" } });
    mTags.insert({ "pnorm exponent", { { {"pnorm", "exponent"}, ""}, "" } });
    mTags.insert({ "minimum ersatz material value", { { {"minimum", "ersatz", "material", "value"}, ""}, "" } });
    mTags.insert({ "standard_deviation_multiplier", { { {"standard_deviation_multiplier"}, ""}, "0" } });
}

void ParseObjective::setName(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("name");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.name = tItr->second.first.second;
    }
}

void ParseObjective::setCategory(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("type");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.type = tItr->second.first.second;
    }
    else
    {
        THROWERR(std::string("Parse Objective: objective 'type' keyword is empty. ")
            + "At least one objective must be defined for an optimization problem.")
    }
}

void ParseObjective::setCode(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("code");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.code_name = tItr->second.first.second;
    }
    else
    {
        aMetadata.code_name = "plato_analyze";
    }
}

void ParseObjective::setNumRanks(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("number ranks");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.num_ranks = tItr->second.first.second;
    }
    else
    {
        aMetadata.num_ranks = "1";
    }
}

void ParseObjective::setNumProcessors(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("number processors");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.num_procs = tItr->second.first.second;
    }
    else
    {
        aMetadata.num_procs = "1";
    }
}

void ParseObjective::setWeight(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("weight");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.weight = tItr->second.first.second;
    }
    else
    {
        aMetadata.weight = "1.0";
    }
}

void ParseObjective::setPerformer(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("performer");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.mPerformerName = tItr->second.first.second;
    }
    else
    {
        // erase any value, keyword will be redefined in this->checkPerfomerNames
        aMetadata.mPerformerName.clear();
    }
}

void ParseObjective::setPenaltyPower(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("penalty power");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.mPenaltyParam = tItr->second.first.second;
    }
    else
    {
        aMetadata.mPenaltyParam = "3.0";
    }
}

void ParseObjective::setPnormExponent(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("pnorm exponent");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.mPnormExponent = tItr->second.first.second;
    }
    else
    {
        aMetadata.mPnormExponent = "6.0";
    }
}

void ParseObjective::setNormalizeObjective(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("normalize objective");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.normalize_objective = tItr->second.first.second;
    }
    else
    {
        aMetadata.normalize_objective = "true";
    }
}

void ParseObjective::setMinimumErsatzValue(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("minimum ersatz material value");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.mMinimumErsatzValue = tItr->second.first.second;
    }
    else
    {
        aMetadata.mMinimumErsatzValue = "1e-9";
    }
}

void ParseObjective::setAnalysisSolverTolerance(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("analysis solver tolerance");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.analysis_solver_tolerance = tItr->second.first.second;
    }
    else
    {
        aMetadata.analysis_solver_tolerance = "1e-7";
    }
}

void ParseObjective::setNaturalBoundaryConditionsCaseIDs(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("load ids");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        std::vector<std::string> tTokens;
        XMLGen::split(tItr->second.first.second, tTokens);
        for (auto &tToken : tTokens)
        {
            aMetadata.load_case_ids.push_back(tToken);
        }
    }
}

void ParseObjective::setNaturalBoundaryConditionsCaseWeights(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("load case weights");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        std::vector<std::string> tTokens;
        XMLGen::split(tItr->second.first.second, tTokens);
        for (auto &tToken : tTokens)
        {
            aMetadata.load_case_weights.push_back(tToken);
        }
    }
}

void ParseObjective::setEssentialBoundaryConditionsCaseIDs(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("boundary condition ids");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        std::vector<std::string> tTokens;
        XMLGen::split(tItr->second.first.second, tTokens);
        for (auto &tToken : tTokens)
        {
            aMetadata.bc_ids.push_back(tToken);
        }
    }
}

void ParseObjective::setFrequencyResponseFunctionMatchNodesets(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("frf match nodesets");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        std::vector<std::string> tTokens;
        XMLGen::split(tItr->second.first.second, tTokens);
        for (auto &tToken : tTokens)
        {
            aMetadata.frf_match_nodesets.push_back(tToken);
        }
    }
}

void ParseObjective::setOutputForPlotting(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("output for plotting");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        std::vector<std::string> tTokens;
        XMLGen::split(tItr->second.first.second, tTokens);
        for (auto &tToken : tTokens)
        {
            aMetadata.output_for_plotting.push_back(tToken);
        }
    }
}

void ParseObjective::setDistributeObjectiveType(XMLGen::Objective &aMetadata)
{
    // original token names: distribute objective at most {number} processors
    // token index: 0          1         2  3    4        5
    // split token names: at most {number} processors
    // token index:       0  1    2        3
    auto tOuterItr = mTags.find("distribute objective");
    if (tOuterItr != mTags.end() && !tOuterItr->second.first.second.empty())
    {
        std::vector<std::string> tTokens;
        XMLGen::split(tOuterItr->second.first.second, tTokens);
        auto tInnerItr = std::find(tTokens.begin(), tTokens.end(), "none");
        if (tInnerItr != tTokens.end())
        {
            aMetadata.multi_load_case = "false";
            aMetadata.distribute_objective_type = "none";
        }
        else
        {
            if (tTokens.size() < 4u)
            {
                THROWERR(
                    "Parse Objective: 'atmost' distributed objective expects at least six input tokens, " + "e.g. expects 'distribute objective at most {number} processors' input keyword. Replace " + "'{number}' with an integer denoting the number of available processors.")
            }

            auto tItrOne = std::find(tTokens.begin(), tTokens.end(), "at");
            auto tItrTwo = std::find(tTokens.begin(), tTokens.end(), "most");
            if ((tItrOne != tTokens.end()) && (tItrTwo != tTokens.end()))
            {
                aMetadata.multi_load_case = "true";
                aMetadata.distribute_objective_type = "atmost";
                aMetadata.atmost_total_num_processors = tTokens[2];
            }
        }
    }
    else
    {
        aMetadata.distribute_objective_type = "none";
    }
}

inline void parse_stress_limit
(const XMLGen::MetaDataTags& aTags,
 XMLGen::Objective& aMetadata)
{
    auto tItr = aTags.find("stress_limit");
    if(tItr == aTags.end())
    {
        THROWERR("Parse Objective: 'stress_limit' keyword is not defined in keyword-to-value map.")
    }

    if(tItr->second.first.second.empty())
    {
        if(aTags.find("type")->second.first.second.compare("stress constrained mass minimization") == 0)
        {
            THROWERR(std::string("Parse Objective: Stress Constrained Mass Minimization (SCMM) problem was ")
                + "requested but the 'stress limit' keyword is not defined. User must defined the 'stress limit' "
                + "for a SCMM problem.")
        }
    }
    aMetadata.stress_limit = aTags.find("stress_limit")->second.first.second;
}

inline void parse_stress_and_mass_criterion_weights
(const XMLGen::MetaDataTags& aTags,
 XMLGen::Objective& aMetadata)
{
    auto tItr = aTags.find("scmm_mass_criterion_weight");
    if(tItr == aTags.end())
    {
        THROWERR("Parse Objective: 'scmm_mass_criterion_weight' keyword is not defined in keyword-to-value map.")
    }
    auto tWeight = tItr->second.first.second.empty() ? std::string("1.0") : tItr->second.first.second;
    aMetadata.scmm_criterion_weights["mass"] = tWeight;

    tItr = aTags.find("scmm_stress_criterion_weight");
    if(tItr == aTags.end())
    {
        THROWERR("Parse Objective: 'scmm_stress_criterion_weight' keyword is not defined in keyword-to-value map.")
    }
    tWeight = tItr->second.first.second.empty() ? std::string("1.0") : tItr->second.first.second;
    aMetadata.scmm_criterion_weights["stress"] = tWeight;
}

void ParseObjective::setStressConstrainedParam(XMLGen::Objective& aMetadata)
{
    aMetadata.scmm_initial_penalty = mTags.find("scmm_initial_penalty")->second.first.second;
    aMetadata.relative_stress_limit = mTags.find("relative_stress_limit")->second.first.second;
    aMetadata.scmm_constraint_exponent = mTags.find("scmm_constraint_exponent")->second.first.second;
    aMetadata.scmm_penalty_upper_bound = mTags.find("scmm_penalty_upper_bound")->second.first.second;
    aMetadata.scmm_penalty_expansion_factor = mTags.find("scmm_penalty_expansion_factor")->second.first.second;
    XMLGen::parse_stress_limit(mTags, aMetadata);
    XMLGen::parse_stress_and_mass_criterion_weights(mTags, aMetadata);
}

void ParseObjective::setMetaData(XMLGen::Objective &aMetadata)
{
    this->setCategory(aMetadata);
    this->setName(aMetadata);
    this->setWeight(aMetadata);
    this->setPerformer(aMetadata);
    this->setPenaltyPower(aMetadata);
    this->setPnormExponent(aMetadata);
    this->setNormalizeObjective(aMetadata);
    this->setMinimumErsatzValue(aMetadata);
    this->setStressConstrainedParam(aMetadata);

    // service info
    this->setCode(aMetadata);
    this->setNumRanks(aMetadata);
    this->setNumProcessors(aMetadata);
    this->setAnalysisSolverTolerance(aMetadata);
    this->setDistributeObjectiveType(aMetadata);
    aMetadata.convert_to_tet10 = mTags.find("ls tet type")->second.first.second;
    aMetadata.ref_frf_file = mTags.find("reference frf file")->second.first.second;
    aMetadata.raleigh_damping_beta = mTags.find("raleigh damping beta")->second.first.second;
    aMetadata.raleigh_damping_alpha = mTags.find("raleigh damping alpha")->second.first.second;
    aMetadata.wtmass_scale_factor = mTags.find("weightmass scale factor")->second.first.second;
    aMetadata.complex_error_measure = mTags.find("complex error measure")->second.first.second;

    // array info, plus these should be inside the service block, excluding the output for plotting
    this->setOutputForPlotting(aMetadata);
    this->setNaturalBoundaryConditionsCaseIDs(aMetadata);
    this->setEssentialBoundaryConditionsCaseIDs(aMetadata);
    this->setNaturalBoundaryConditionsCaseWeights(aMetadata);
    this->setFrequencyResponseFunctionMatchNodesets(aMetadata);
}

void ParseObjective::checkType(const XMLGen::Objective &aMetadata)
{
    XMLGen::ValidCriterionKeys tValidKeys;
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), aMetadata.type);
    if (tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Parse Objective: 'type' keyword '") + aMetadata.type + "' is not supported. ")
    }
}

void ParseObjective::checkCode(const XMLGen::Objective &aMetadata)
{
    XMLGen::ValidCodeKeys tValidKeys;
    if (std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), aMetadata.code_name) == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Parse Objective: 'code' keyword '") + aMetadata.code_name + "' is not supported. ")
    }
}

void ParseObjective::checkLoadCases(XMLGen::Objective &aMetadata)
{
    this->checkLoadCaseIDs(aMetadata);
    this->checkLoadCaseWeights(aMetadata);
    this->checkMultiLoadCaseFlag(aMetadata);
    if (aMetadata.load_case_ids.size() != aMetadata.load_case_weights.size())
    {
        THROWERR(std::string("Parse Objective: Length mismatch in load case ids and weights. Check that the 'load_case_ids'")
            + " and 'load_case_weights' keywords have the same number of inputs.")
    }
}

void ParseObjective::checkMultiLoadCaseFlag(XMLGen::Objective &aMetadata)
{
    if (aMetadata.distribute_objective_type.compare("none") == 0)
    {
        aMetadata.multi_load_case = "false";
    }
}

void ParseObjective::checkLoadCaseIDs(const XMLGen::Objective &aMetadata)
{
    if (aMetadata.load_case_ids.empty())
    {
        THROWERR("Parse Objective: 'load case ids' keyword is empty, at least one load case id should be defined.")
    }
}

void ParseObjective::checkLoadCaseWeights(XMLGen::Objective &aMetadata)
{
    if (aMetadata.load_case_weights.empty())
    {
        for (auto tItr = aMetadata.load_case_ids.begin(); tItr != aMetadata.load_case_ids.end(); ++tItr)
        {
            aMetadata.load_case_weights.push_back("1.0");
        }
    }
}

void ParseObjective::checkEssentialBoundaryConditionIDs(const XMLGen::Objective &aMetadata)
{
    if (aMetadata.bc_ids.empty())
    {
        THROWERR("Parse Objective: 'boundary condition ids' keyword is empty, at least one essential boundary condition id should be defined.")
    }
}

void ParseObjective::checkOutputForPlotting(const XMLGen::Objective &ValidOutputToLayoutKeys)
{
    XMLGen::ValidOutputToLayoutKeys tValidKeys;
    for (auto &tKeyword : ValidOutputToLayoutKeys.output_for_plotting)
    {
        auto tLowerKey = Plato::tolower(tKeyword);
        auto tItr =  tValidKeys.mKeys.find(tLowerKey);
        if (tItr == tValidKeys.mKeys.end())
        {
            THROWERR(std::string("Parse Objective: 'output' keyword '") + tLowerKey + "' is not supported. ")
        }
    }
}

void ParseObjective::checkDistributeObjective(const XMLGen::Objective &aMetadata)
{
    if (aMetadata.distribute_objective_type.compare("atmost") == 0)
    {
        if (!XMLGen::is_integer(aMetadata.atmost_total_num_processors))
        {
            THROWERR("Parse Objective: distributed objective is of type 'at most' and the number of processors is not a finite number.")
        }
    }
}

void ParseObjective::checkMetaData(XMLGen::Objective &aMetadata)
{
    this->checkType(aMetadata);
    this->checkCode(aMetadata);
    this->checkLoadCases(aMetadata);
    this->checkMultiLoadCaseFlag(aMetadata);
    this->checkOutputForPlotting(aMetadata);
    this->checkDistributeObjective(aMetadata);
    this->checkEssentialBoundaryConditionIDs(aMetadata);
}

void ParseObjective::setIdentification()
{
    for (auto &tOuterObjective : mData)
    {
        // For each code name we will make sure there are names set
        auto tMyCodeName = tOuterObjective.code_name;
        size_t tObjectiveIdentificationNumber = 0;
        for (auto &tInnerObjective : mData)
        {
            if (!tInnerObjective.code_name.compare(tMyCodeName))
            {
                tObjectiveIdentificationNumber++;
                if (tInnerObjective.name.empty())
                {
                    tInnerObjective.name = std::to_string(tObjectiveIdentificationNumber);
                }
            }
        }
    }
}

void ParseObjective::checkPerfomerNames()
{
    for(auto& tObjective : mData)
    {
        if(tObjective.mPerformerName.empty())
        {
            tObjective.mPerformerName = tObjective.code_name + "_" + tObjective.name;
        }
    }
}

void ParseObjective::finalize()
{
    this->setIdentification();
    this->checkPerfomerNames();
}

std::vector<XMLGen::Objective> ParseObjective::data() const
{
    return mData;
}

void ParseObjective::parse(std::istream &aInputFile)
{
    mData.clear();
    this->allocate();
    constexpr int MAX_CHARS_PER_LINE = 10000;
    std::vector<char> tBuffer(MAX_CHARS_PER_LINE);
    while (!aInputFile.eof())
    {
        // read an entire line into memory
        std::vector<std::string> tTokens;
        aInputFile.getline(tBuffer.data(), MAX_CHARS_PER_LINE);
        XMLGen::parse_tokens(tBuffer.data(), tTokens);
        XMLGen::to_lower(tTokens);

        std::string tTag;
        if (XMLGen::parse_single_value(tTokens, { "begin", "objective" }, tTag))
        {
            XMLGen::Objective tMetadata;
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata( { "end", "objective" }, aInputFile, mTags);
            this->setMetaData(tMetadata);
            this->checkMetaData(tMetadata);
            mData.push_back(tMetadata);
        }
    }
    this->finalize();
}

}
// namespace XMLGen
