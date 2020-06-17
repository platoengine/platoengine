/*
 * XMLGeneratorParseObjective_UnitTester.cpp
 *
 *  Created on: Jun 16, 2020
 */

#include <gtest/gtest.h>

#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

struct ValidCriterionKeys
{
    /*!<
     * \brief Valid plato xml generator parser criterion keys.
     **/
    std::vector<std::string> mKeys =
        {"maximize total work", "stress constrained mass minimization", "maximize stiffness", "compliance",
         "volume", "minimize thermoelastic energy", "thermoelastic energy", "maximize heat conduction",
         "stress p-norm", "flux p-norm", "effective energy", "minimize effective energy", "minimize stress",
         "minimize flux", "electroelastic energy", "minimize electroelastic energy", "thermal energy"};
};
// struct ValidCriterionKeys

struct ValidOutputKeys
{
    /*!<
     * \brief Valid plato xml generator parser output keys.
     **/
    std::vector<std::string> mKeys =
        {"vonmises", "dispx", "dispy", "dispz", "temperature", "accumulated_plastic_strain", "potential"};
};
// struct ValidCriterionKeys

class ParseObjective
{
private:
    XMLGen::UseCaseTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */
    std::vector<XMLGen::Objective> mData; /*!< objective functions metadata */

private:
    /******************************************************************************//**
     * \fn allocate
     * \brief Allocate map from valid tags to valid tokens-value pair
    **********************************************************************************/
    void allocate()
    {
        mTags.clear();

        mTags.insert({ "type", { {"type"}, "" } });
        mTags.insert({ "code", { {"code"}, "" } });
        mTags.insert({ "name", { {"name"}, "" } });
        mTags.insert({ "weight", { {"weight"}, "" } });
        mTags.insert({ "load ids", { {"load", "ids"}, "" } });
        mTags.insert({ "stress limit", { {"stress","limit"}, "" } });
        mTags.insert({ "ls tet type", { {"ls","tet","type"}, "" } });
        mTags.insert({ "number ranks", { {"number", "ranks"}, "" } });
        mTags.insert({ "multi load case", { {"multi","load","case"}, "" } });
        mTags.insert({ "number processors", { {"number", "processors"}, "" } });
        mTags.insert({ "load case weights", { {"load","case","weights"}, "" } });
        mTags.insert({ "normalize objective", { {"normalize","objective"}, "" } });
        mTags.insert({ "frf match nodesets", { {"frf","match","nodesets"}, "" } });
        mTags.insert({ "reference frf file", { {"reference","frf","file"}, "" } });
        mTags.insert({ "distribute objective", { {"distribute","objective"}, "" } });
        mTags.insert({ "output for plotting", { {"output", "for", "plotting"}, "" } });
        mTags.insert({ "raleigh damping beta", { {"raleigh","damping","beta"}, "" } });
        mTags.insert({ "raleigh damping alpha", { {"raleigh","damping","alpha"}, "" } });
        mTags.insert({ "relative stress limit", { {"relative","stress","limit"}, "" } });
        mTags.insert({ "scmm initial penalty", { {"scmm", "initial", "penalty"}, "" } });
        mTags.insert({ "complex error measure", { {"complex","error","measure"}, "" } });
        mTags.insert({ "boundary condition ids", { {"boundary","condition", "ids"}, "" } });
        mTags.insert({ "weightmass scale factor", { {"weightmass","scale","factor"}, "" } });
        mTags.insert({ "scmm constraint exponent", { {"scmm", "constraint", "exponent"}, "" } });
        mTags.insert({ "analysis solver tolerance", { {"analysis","solver","tolerance"}, "" } });
        mTags.insert({ "scmm penalty expansion factor", { {"scmm", "penalty", "expansion", "factor"}, "" } });

        mTags.insert({ "penalty power", { {"penalty", "power"}, "" } });
        mTags.insert({ "pnorm exponent", { {"pnorm", "exponent"}, "" } });
        mTags.insert({ "minimum ersatz material value", { {"minimum", "ersatz", "material", "value"}, "" } });
    }

    /******************************************************************************//**
     * \fn erase
     * \brief Erases value key content in map from valid tags to valid tokens-value pair.
    **********************************************************************************/
    void erase()
    {
        for(auto& tTag : mTags)
        {
            tTag.second.second.clear();
        }
    }

    /******************************************************************************//**
     * \fn setName
     * \brief Set objective function's user assigned name.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setName(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("name");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.name = tItr->second.second;
        }
    }

    /******************************************************************************//**
     * \fn setType
     * \brief Set objective function's type, i.e. category.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setType(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("type");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.type = tItr->second.second;
        }
        else
        {
            aMetadata.type = "compliance";
        }
    }

    /******************************************************************************//**
     * \fn setCode
     * \brief Set code responsible for evaluating the objective function.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setCode(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("code");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.code_name = tItr->second.second;
        }
        else
        {
            aMetadata.code_name = "plato_analyze";
        }
    }

    /******************************************************************************//**
     * \fn setNumRanks
     * \brief Set number of ranks assign to the code responsible for evaluating the \n
     * objective function.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setNumRanks(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("number ranks");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.num_ranks = tItr->second.second;
        }
        else
        {
            aMetadata.num_ranks = "1";
        }
    }

    /******************************************************************************//**
     * \fn setNumProcessors
     * \brief Set number of processors assign to the code responsible for evaluating \n
     * the objective function.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setNumProcessors(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("number processors");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.num_procs = tItr->second.second;
        }
        else
        {
            aMetadata.num_procs = "1";
        }
    }

    /******************************************************************************//**
     * \fn setWeight
     * \brief Set objective function weight.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setWeight(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("weight");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.weight = tItr->second.second;
        }
        else
        {
            aMetadata.weight = "1.0";
        }
    }

    /******************************************************************************//**
     * \fn setPenaltyPower
     * \brief Set exponent for material density penalty function.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setPenaltyPower(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("penalty power");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.mPenaltyParam = tItr->second.second;
        }
        else
        {
            aMetadata.mPenaltyParam = "3.0";
        }
    }

    /******************************************************************************//**
     * \fn setPnormExponent
     * \brief Set p-norm exponent.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setPnormExponent(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("pnorm exponent");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.mPnormExponent = tItr->second.second;
        }
        else
        {
            aMetadata.mPnormExponent = "6.0";
        }
    }

    /******************************************************************************//**
     * \fn setNormalizeObjective
     * \brief Set normalize objective function flag.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setNormalizeObjective(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("normalize objective");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.normalize_objective = tItr->second.second;
        }
        else
        {
            aMetadata.normalize_objective = "true";
        }
    }

    /******************************************************************************//**
     * \fn setMinimumErsatzValue
     * \brief Set minimum Ersatz material parameter used in material density penalty function.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setMinimumErsatzValue(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("minimum ersatz material value");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.mMinimumErsatzValue = tItr->second.second;
        }
        else
        {
            aMetadata.mMinimumErsatzValue = "1e-9";
        }
    }

    /******************************************************************************//**
     * \fn setAnalysisSolverTolerance
     * \brief Set solver tolerance for solver use to solve the finite element system of equations.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setAnalysisSolverTolerance(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("analysis solver tolerance");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            aMetadata.analysis_solver_tolerance = tItr->second.second;
        }
        else
        {
            aMetadata.analysis_solver_tolerance = "1e-7";
        }
    }

    /******************************************************************************//**
     * \fn setNaturalBoundaryConditionsCaseIDs
     * \brief Set identification numbers associated with each natural boundary condition case.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setNaturalBoundaryConditionsCaseIDs(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("load ids");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            std::vector<std::string> tTokens;
            XMLGen::split(tItr->second.second, tTokens);
            for(auto& tToken : tTokens)
            {
                aMetadata.load_case_ids.push_back(tToken);
            }
        }
    }

    /******************************************************************************//**
     * \fn setNaturalBoundaryConditionsCaseIDs
     * \brief Set weights associated with each natural boundary conditions.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setNaturalBoundaryConditionsCaseWeights(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("load case weights");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            std::vector<std::string> tTokens;
            XMLGen::split(tItr->second.second, tTokens);
            for(auto& tToken : tTokens)
            {
                aMetadata.load_case_weights.push_back(tToken);
            }
        }
    }

    /******************************************************************************//**
     * \fn setNaturalBoundaryConditionsCaseIDs
     * \brief Set identification numbers associated with the essential boundary conditions.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setEssentialBoundaryConditionsCaseIDs(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("boundary condition ids");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            std::vector<std::string> tTokens;
            XMLGen::split(tItr->second.second, tTokens);
            for(auto& tToken : tTokens)
            {
                aMetadata.bc_ids.push_back(tToken);
            }
        }
    }

    void setFrequencyResponseFunctionMatchNodesets(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("frf match nodesets");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            std::vector<std::string> tTokens;
            XMLGen::split(tItr->second.second, tTokens);
            for(auto& tToken : tTokens)
            {
                aMetadata.frf_match_nodesets.push_back(tToken);
            }
        }
    }

    /******************************************************************************//**
     * \fn setOutputForPlotting
     * \brief Set keywords for output quantities of interests.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setOutputForPlotting(XMLGen::Objective& aMetadata)
    {
        auto tItr = mTags.find("output for plotting");
        if(tItr != mTags.end() && !tItr->second.second.empty())
        {
            std::vector<std::string> tTokens;
            XMLGen::split(tItr->second.second, tTokens);
            for(auto& tToken : tTokens)
            {
                aMetadata.output_for_plotting.push_back(tToken);
            }
        }
    }

    /******************************************************************************//**
     * \fn setDistributeObjectiveType
     * \brief Set parameters used to identify if we are running multiple performers in \n
     * using the multiple program, multiple data parallel programming model as well as
     * the number of processors used for each criterion use case.
     * \param [in/out] aInputFile parsed input file metadata
    **********************************************************************************/
    void setDistributeObjectiveType(XMLGen::Objective& aMetadata)
    {
        // original token names: distribute objective at most {number} processors
        // token index: 0          1         2  3    4        5
        // split token names: at most {number} processors
        // token index:       0  1    2        3
        auto tOuterItr = mTags.find("distribute objective");
        if(tOuterItr != mTags.end() && !tOuterItr->second.second.empty())
        {
            std::vector<std::string> tTokens;
            XMLGen::split(tOuterItr->second.second, tTokens);
            auto tInnerItr = std::find(tTokens.begin(), tTokens.end(), "none");
            if (tInnerItr != tTokens.end())
            {
                aMetadata.multi_load_case = "false";
                aMetadata.distribute_objective_type = "none";
            }
            else
            {
                if(tTokens.size() < 4u)
                {
                    THROWERR("Parse Objective: 'atmost' distributed objective expects at least six input tokens, " +
                        "e.g. expects 'distribute objective at most {number} processors' input keyword. Replace " +
                        "'{number}' with an integer denoting the number of available processors.")
                }

                auto tItrOne = std::find(tTokens.begin(), tTokens.end(), "at");
                auto tItrTwo = std::find(tTokens.begin(), tTokens.end(), "most");
                if( (tItrOne != tTokens.end()) && (tItrTwo != tTokens.end()) )
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

    /******************************************************************************//**
     * \fn setMetadata
     * \brief Set objective metadata.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void setMetadata(XMLGen::Objective& aMetadata)
    {
        this->setType(aMetadata);
        this->setName(aMetadata);
        this->setWeight(aMetadata);
        this->setPenaltyPower(aMetadata);
        this->setPnormExponent(aMetadata);
        this->setNormalizeObjective(aMetadata);
        this->setMinimumErsatzValue(aMetadata);
        aMetadata.stress_limit = mTags.find("stress limit")->second.second;
        aMetadata.scmm_initial_penalty = mTags.find("scmm initial penalty")->second.second;
        aMetadata.complex_error_measure = mTags.find("complex error measure")->second.second;
        aMetadata.relative_stress_limit = mTags.find("relative stress limit")->second.second;
        aMetadata.scmm_constraint_exponent = mTags.find("scmm constraint exponent")->second.second;
        aMetadata.scmm_penalty_expansion_factor = mTags.find("scmm penalty expansion factor")->second.second;

        // scenario info
        this->setCode(aMetadata);
        this->setNumRanks(aMetadata);
        this->setNumProcessors(aMetadata);
        this->setAnalysisSolverTolerance(aMetadata);
        this->setDistributeObjectiveType(aMetadata);
        aMetadata.convert_to_tet10 = mTags.find("ls tet type")->second.second;
        aMetadata.ref_frf_file = mTags.find("reference frf file")->second.second;
        aMetadata.raleigh_damping_beta = mTags.find("raleigh damping beta")->second.second;
        aMetadata.raleigh_damping_alpha = mTags.find("raleigh damping alpha")->second.second;
        aMetadata.wtmass_scale_factor = mTags.find("weightmass scale factor")->second.second;

        // array info, plus these should be inside the scenario block, excluding the output for plotting
        this->setOutputForPlotting(aMetadata);
        this->setNaturalBoundaryConditionsCaseIDs(aMetadata);
        this->setEssentialBoundaryConditionsCaseIDs(aMetadata);
        this->setNaturalBoundaryConditionsCaseWeights(aMetadata);
        this->setFrequencyResponseFunctionMatchNodesets(aMetadata);
    }

    /******************************************************************************//**
     * \fn checkType
     * \brief Throw error if input objective function category is not supported.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkType(const XMLGen::Objective& aMetadata)
    {
        XMLGen::ValidCriterionKeys tValidKeys;
        auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), aMetadata.type);
        if (tItr == tValidKeys.mKeys.end())
        {
            THROWERR(std::string("Parse Objective: 'type' keyword '") + aMetadata.type + "' is not supported. ")
        }
    }

    /******************************************************************************//**
     * \fn checkCode
     * \brief Throw error if simulation code is not supported.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkCode(const XMLGen::Objective& aMetadata)
    {
        XMLGen::ValidPhysicsPerformerKeys tValidKeys;
        if (std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), aMetadata.code_name) == tValidKeys.mKeys.end())
        {
            std::ostringstream tMsg;
            tMsg << "Parse Objective: 'code' keyword '" << aMetadata.code_name << "' is not supported. ";
            THROWERR(tMsg.str().c_str())
        }
    }

    /******************************************************************************//**
     * \fn checkPerformer
     * \brief Check if performer keyword is empty, if empty, set to the code name. \n
     * Furthermore, check if simulation code is supported, if not, throw error.
     * \param [in/out] aMetadata objective metadata
    **********************************************************************************/
    void checkPerformer(XMLGen::Objective& aMetadata)
    {
        if(aMetadata.mPerformerName.empty())
        {
            aMetadata.mPerformerName = aMetadata.code_name;
            XMLGen::ValidPhysicsPerformerKeys tValidKeys;
            if (std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), aMetadata.mPerformerName) == tValidKeys.mKeys.end())
            {
                std::ostringstream tMsg;
                tMsg << "Parse Objective: 'performer' keyword '" << aMetadata.mPerformerName << "' is not supported. ";
                THROWERR(tMsg.str().c_str())
            }
        }
    }

    /******************************************************************************//**
     * \fn checkLoadCases
     * \brief Throw error if load cases are not properly defined.
     * \param [in/out] aMetadata objective metadata
    **********************************************************************************/
    void checkLoadCases(XMLGen::Objective& aMetadata)
    {
        this->checkLoadCaseIDs(aMetadata);
        this->checkLoadCaseWeights(aMetadata);
        this->checkMultiLoadCaseFlag(aMetadata);
        if(aMetadata.load_case_ids.size() != aMetadata.load_case_weights.size())
        {
            THROWERR(std::string("Parse Objective: Length mismatch in load case ids and weights. Check that the 'load_case_ids'") +
                " and 'load_case_weights' keywords have the same number of inputs.")
        }
    }

    /******************************************************************************//**
     * \fn checkMultiLoadCaseFlag
     * \brief Check if multi-laod case flag has been properly set.
     * \param [in/out] aMetadata objective metadata
    **********************************************************************************/
    void checkMultiLoadCaseFlag(XMLGen::Objective& aMetadata)
    {
        if(aMetadata.distribute_objective_type.compare("none") == 0)
        {
            aMetadata.multi_load_case = "false";
        }
    }

    /******************************************************************************//**
     * \fn checkLoadCaseIDs
     * \brief Throw error if list of natural boundary conditions identification numbers is empty.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkLoadCaseIDs(const XMLGen::Objective& aMetadata)
    {
        if (aMetadata.load_case_ids.empty())
        {
            THROWERR("Parse Objective: 'load case ids' keyword is empty, at least one load case id should be defined.")
        }
    }

    /******************************************************************************//**
     * \fn checkLoadCaseWeights
     * \brief Set weights associated with each natural boundary condition case to 1.0 \n
     * if not defined in the input file.
     * \param [in/out] aMetadata objective metadata
    **********************************************************************************/
    void checkLoadCaseWeights(XMLGen::Objective& aMetadata)
    {
        if (aMetadata.load_case_weights.empty())
        {
            for(auto tItr = aMetadata.load_case_ids.begin(); tItr != aMetadata.load_case_ids.end(); ++tItr)
            {
                aMetadata.load_case_weights.push_back("1.0");
            }
        }
    }

    /******************************************************************************//**
     * \fn checkEssentialBoundaryConditionIDs
     * \brief Throw error if list of essential boundary conditions identification numbers is empty.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkEssentialBoundaryConditionIDs(const XMLGen::Objective& aMetadata)
    {
        if (aMetadata.bc_ids.empty())
        {
            THROWERR("Parse Objective: 'boundary condition ids' keyword is empty, at least one essential boundary condition id should be defined.")
        }
    }

    /******************************************************************************//**
     * \fn checkEssentialBoundaryConditionIDs
     * \brief Throw error if output keyword is not supported.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkOutputForPlotting(const XMLGen::Objective& ValidOutputKeys)
    {
        XMLGen::ValidOutputKeys tValidKeys;
        for(auto& tToken : ValidOutputKeys.output_for_plotting)
        {
            if (std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tToken) == tValidKeys.mKeys.end())
            {
                std::ostringstream tMsg;
                tMsg << "Parse Objective: 'output' keyword '" << tToken << "' is not supported. ";
                THROWERR(tMsg.str().c_str())
            }
        }
    }

    /******************************************************************************//**
     * \fn checkDistributeObjective
     * \brief Throw error if number of processors assigned to distributed objective \n
     * use case is not a finite number.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkDistributeObjective(const XMLGen::Objective& aMetadata)
    {
        if(aMetadata.distribute_objective_type.compare("atmost") == 0)
        {
            if(!XMLGen::is_number(aMetadata.atmost_total_num_processors))
            {
                THROWERR("Parse Objective: distributed objective is of type 'at most' and the number of processors is not a finite number.")
            }
        }
    }

    /******************************************************************************//**
     * \fn checkMetadata
     * \brief Check if objective metadata is properly defined.
     * \param [in] aMetadata objective metadata
    **********************************************************************************/
    void checkMetadata(XMLGen::Objective& aMetadata)
    {
        this->checkType(aMetadata);
        this->checkCode(aMetadata);
        this->checkPerformer(aMetadata);
        this->checkLoadCases(aMetadata);
        this->checkMultiLoadCaseFlag(aMetadata);
        this->checkOutputForPlotting(aMetadata);
        this->checkDistributeObjective(aMetadata);
        this->checkEssentialBoundaryConditionIDs(aMetadata);
    }

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return objective functions metadata.
     * \return container of objective functions and corresponding metadata
    **********************************************************************************/
    std::vector<XMLGen::Objective> data() const
    {
        return mData;
    }

    /******************************************************************************//**
     * \fn parse
     * \brief Parse objective functions metadata.
     * \param [in] aInputFile parsed input file metadata
    **********************************************************************************/
    void parse(std::istream& aInputFile)
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
            if(XMLGen::parse_single_value(tTokens, {"begin","objective"}, tTag))
            {
                XMLGen::Objective tMetadata;
                this->erase();
                XMLGen::parse_input_metadata({"end","objective"}, aInputFile, mTags);
                this->setMetadata(tMetadata);
                this->checkMetadata(tMetadata);
                mData.push_back(tMetadata);
            }
        }
    }
};

}

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorInvalidCriterion)
{
    std::string tStringInput =
        "begin objective\n"
        "   type hippo\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorDimMistmachNaturalBCArrays)
{
    std::string tStringInput =
        "begin objective\n"
        "   type stress p-norm\n"
        "   load ids 10\n"
        "   load case weights 1 2\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorInvalidCode)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code dog\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorInvalidOutputKey)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz bats\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorEmptyNaturalBCIDs)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorEmptyEssentialBCIDs)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorNumProcessorIsNotNumber)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "   distribute objective at most P3 processors\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_OneObjective)
{
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    tObjectiveParser.parse(tInputSS);
    auto tObjectiveMetadata = tObjectiveParser.data();
    ASSERT_EQ(1u, tObjectiveMetadata.size());

    ASSERT_STREQ("maximize stiffness", tObjectiveMetadata[0].type.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_procs.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_ranks.c_str());
    ASSERT_STREQ("1.0", tObjectiveMetadata[0].weight.c_str());
    ASSERT_STREQ("3.0", tObjectiveMetadata[0].mPenaltyParam.c_str());
    ASSERT_STREQ("6.0", tObjectiveMetadata[0].mPnormExponent.c_str());
    ASSERT_STREQ("1e-9", tObjectiveMetadata[0].mMinimumErsatzValue.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[0].code_name.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[0].mPerformerName.c_str());
    ASSERT_STREQ("true", tObjectiveMetadata[0].normalize_objective.c_str());
    ASSERT_STREQ("1e-7", tObjectiveMetadata[0].analysis_solver_tolerance.c_str());
    ASSERT_STREQ("false", tObjectiveMetadata[0].multi_load_case.c_str());
    ASSERT_STREQ("none", tObjectiveMetadata[0].distribute_objective_type.c_str());

    ASSERT_EQ(1u, tObjectiveMetadata[0].bc_ids.size());
    ASSERT_STREQ("11", tObjectiveMetadata[0].bc_ids[0].c_str());
    ASSERT_EQ(1u, tObjectiveMetadata[0].load_case_ids.size());
    ASSERT_STREQ("10", tObjectiveMetadata[0].load_case_ids[0].c_str());

    ASSERT_EQ(3u, tObjectiveMetadata[0].output_for_plotting.size());
    std::vector<std::string> tGoldOutputs = {"dispx", "dispy", "dispz"};
    for(auto& tValue : tObjectiveMetadata[0].output_for_plotting)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[0].output_for_plotting[0];
        ASSERT_STREQ(tGoldOutputs[tIndex].c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseObjective_TwoObjective)
{
    std::string tStringInput =
        "begin objective\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n"
        "begin objective\n"
        "   type stress p-norm\n"
        "   normalize objective false\n"
        "   load ids 10 20\n"
        "   boundary condition ids 11 21\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dIspy dispz vonmises\n"
        "   distribute objective at most 3 processors\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    tObjectiveParser.parse(tInputSS);
    auto tObjectiveMetadata = tObjectiveParser.data();
    ASSERT_EQ(2u, tObjectiveMetadata.size());

    // TEST OBJECTIVE 1
    ASSERT_STREQ("compliance", tObjectiveMetadata[0].type.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_procs.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_ranks.c_str());
    ASSERT_STREQ("1.0", tObjectiveMetadata[0].weight.c_str());
    ASSERT_STREQ("3.0", tObjectiveMetadata[0].mPenaltyParam.c_str());
    ASSERT_STREQ("6.0", tObjectiveMetadata[0].mPnormExponent.c_str());
    ASSERT_STREQ("1e-9", tObjectiveMetadata[0].mMinimumErsatzValue.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[0].code_name.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[0].mPerformerName.c_str());
    ASSERT_STREQ("true", tObjectiveMetadata[0].normalize_objective.c_str());
    ASSERT_STREQ("1e-7", tObjectiveMetadata[0].analysis_solver_tolerance.c_str());
    ASSERT_STREQ("false", tObjectiveMetadata[0].multi_load_case.c_str());
    ASSERT_STREQ("none", tObjectiveMetadata[0].distribute_objective_type.c_str());

    ASSERT_EQ(1u, tObjectiveMetadata[0].bc_ids.size());
    ASSERT_STREQ("11", tObjectiveMetadata[0].bc_ids[0].c_str());
    ASSERT_EQ(1u, tObjectiveMetadata[0].load_case_ids.size());
    ASSERT_STREQ("10", tObjectiveMetadata[0].load_case_ids[0].c_str());

    ASSERT_EQ(3u, tObjectiveMetadata[0].output_for_plotting.size());
    std::vector<std::string> tGoldOutputs = {"dispx", "dispy", "dispz"};
    for(auto& tValue : tObjectiveMetadata[0].output_for_plotting)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[0].output_for_plotting[0];
        ASSERT_STREQ(tGoldOutputs[tIndex].c_str(), tValue.c_str());
    }

    // TEST OBJECTIVE 2
    ASSERT_STREQ("stress p-norm", tObjectiveMetadata[1].type.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[1].num_procs.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[1].num_ranks.c_str());
    ASSERT_STREQ("1.0", tObjectiveMetadata[1].weight.c_str());
    ASSERT_STREQ("3.0", tObjectiveMetadata[1].mPenaltyParam.c_str());
    ASSERT_STREQ("6.0", tObjectiveMetadata[1].mPnormExponent.c_str());
    ASSERT_STREQ("1e-9", tObjectiveMetadata[1].mMinimumErsatzValue.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[1].code_name.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[1].mPerformerName.c_str());
    ASSERT_STREQ("false", tObjectiveMetadata[1].normalize_objective.c_str());
    ASSERT_STREQ("1e-7", tObjectiveMetadata[1].analysis_solver_tolerance.c_str());
    ASSERT_STREQ("true", tObjectiveMetadata[1].multi_load_case.c_str());
    ASSERT_STREQ("3", tObjectiveMetadata[1].atmost_total_num_processors.c_str());
    ASSERT_STREQ("atmost", tObjectiveMetadata[1].distribute_objective_type.c_str());

    std::vector<std::string> tGoldEssentialBCIDs = {"11", "21"};
    ASSERT_EQ(2u, tObjectiveMetadata[1].bc_ids.size());
    for(auto& tValue : tObjectiveMetadata[1].bc_ids)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[1].bc_ids[0];
        ASSERT_STREQ(tGoldEssentialBCIDs[tIndex].c_str(), tValue.c_str());
    }

    std::vector<std::string> tGoldNaturalBCIDs = {"10", "20"};
    ASSERT_EQ(2u, tObjectiveMetadata[1].load_case_ids.size());
    for(auto& tValue : tObjectiveMetadata[1].load_case_ids)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[1].load_case_ids[0];
        ASSERT_STREQ(tGoldNaturalBCIDs[tIndex].c_str(), tValue.c_str());
    }

    ASSERT_EQ(4u, tObjectiveMetadata[1].output_for_plotting.size());
    tGoldOutputs = {"dispx", "dispy", "dispz", "vonmises"};
    for(auto& tValue : tObjectiveMetadata[0].output_for_plotting)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[0].output_for_plotting[0];
        ASSERT_STREQ(tGoldOutputs[tIndex].c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, Split)
{
    std::string tInput("1 2 3 4 5 6 7 8");
    std::vector<std::string> tOutput;
    XMLGen::split(tInput, tOutput);
    ASSERT_EQ(8u, tOutput.size());

    std::vector<std::string> tGold = {"1", "2", "3", "4", "5", "6", "7", "8"};
    for(auto& tValue : tOutput)
    {
        auto tIndex = &tValue - &tOutput[0];
        ASSERT_STREQ(tGold[tIndex].c_str(), tValue.c_str());
    }
}

}
// namespace PlatoTestXMLGenerator
