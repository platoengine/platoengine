/*
 //@HEADER
 // *************************************************************************
 //   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
 //                    Solutions of Sandia, LLC (NTESS).
 //
 // Under the terms of Contract DE-NA0003525 with NTESS,
 // the U.S. Government retains certain rights in this software.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are
 // met:
 //
 // 1. Redistributions of source code must retain the above copyright
 // notice, this list of conditions and the following disclaimer.
 //
 // 2. Redistributions in binary form must reproduce the above copyright
 // notice, this list of conditions and the following disclaimer in the
 // documentation and/or other materials provided with the distribution.
 //
 // 3. Neither the name of the Sandia Corporation nor the names of the
 // contributors may be used to endorse or promote products derived from
 // this software without specific prior written permission.
 //
 // THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
 // EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 // PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
 // CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 // EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 // PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 // PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 // LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 // NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 // SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 //
 // Questions? Contact the Plato team (plato3D-help@sandia.gov)
 //
 // *************************************************************************
 //@HEADER
 */

/*
 * Plato_Test_WriteParameterStudyData.cpp
 *
 *  Created on: Jun 21, 2021
 */

#include <gtest/gtest.h>

#include <locale>


#include "PlatoApp.hpp"
#include "Plato_Utils.hpp"
#include "Plato_Macros.hpp"
#include "Plato_InputData.hpp"
#include "Plato_UnitTestUtils.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

/******************************************************************************//**
 * \class WriteParameterStudyData
 * \brief Write design paraemters, criterion values and criterion gradients to text file.
**********************************************************************************/
class WriteParameterStudyData : public Plato::LocalOp
{
public:
    /******************************************************************************//**
    * \fn WriteParameterStudyData
    * \brief Constructor.
    * \param [in] aPlatoApp Plato Engine main application interface.
    * \param [in] aOperationNode input database for operation node 
    **********************************************************************************/
    WriteParameterStudyData(PlatoApp* aPlatoApp, const Plato::InputData& aOperationNode) : 
        Plato::LocalOp(aPlatoApp),
        mOperationInputData(aOperationNode)
    {
        this->initialize();
    }

    /******************************************************************************//**
    * \fn WriteParameterStudyData
    * \brief Destructor.
    **********************************************************************************/
    virtual ~WriteParameterStudyData(){}

    /******************************************************************************//**
     * \fn void operator()()
     * \brief Output parameter study data to a text file. Data includes criterion value, 
     *        criterion gradient and design/optimization parameters. 
    **********************************************************************************/
    void operator()() override
    {
        int tMyRank = 0;
        MPI_Comm_rank(mPlatoApp->getComm(), &tMyRank);
        if (tMyRank == 0)
        {
            mFile.open(mOutputTextFile);
            if (mFile.is_open())
            {
                this->writeDesignParameters();
                this->writeCriterionValue();
                this->writeCriterionGradients();
                mFile << "\n";
            }
            else
            {
                THROWERR(std::string("Output file '") + mOutputTextFile + "' could not be opened. Output to file failed.")
            }
            mFile.close();
        }
    }

    /******************************************************************************//**
     * \fn void getArguments
     * \brief Get local operation argument list from the operation.
     * \param [out] aLocalArgs argument list
    **********************************************************************************/
    void getArguments(std::vector<Plato::LocalArg>& aLocalArgs) override 
    {
        std::vector<Plato::LocalArg> tOutput;
        this->appendArguments(mDesignParamArguments, tOutput);
        this->appendArguments(mCriterionValueArguments, tOutput);
        this->appendArguments(mCriterionGradientArguments, tOutput);
        aLocalArgs = tOutput;
    }

// private functions
private:
    /******************************************************************************//**
     * \fn void initialize
     * \brief Initialize class member data. 
    **********************************************************************************/
    void initialize()
    {
        this->parseOperationMetadata();
        this->parseDesignParameters();
        this->parseCriterionValues();
        this->parseCriterionGradients();

        this->writeHeader();
    }

    /******************************************************************************//**
     * \fn void parseOperationMetadata
     * \brief Parse operation metadata, e.g. operation name, operation identifier 
     *        ("function name"), output text file, etc.
    **********************************************************************************/
    void parseOperationMetadata()
    {
        mOperation = Plato::Get::String(mOperationInputData, "Name");
        mFunction = Plato::Get::String(mOperationInputData, "Function");
        mOutputTextFile = Plato::Get::String(mOperationInputData, "OutputTextFile");
        if(mOutputTextFile.empty())
            { mOutputTextFile = "plato_parameter_study.dat"; }
    }

    /******************************************************************************//**
     * \fn void parseDesignParameters
     * \brief Parse design parameter argument names. 
    **********************************************************************************/
    void parseDesignParameters()
    {
        for (auto &tDesignParameterNode : mOperationInputData.getByName<Plato::InputData>("DesignParameters"))
        {
            this->parseNodeInputs(tDesignParameterNode, mDesignParamArguments);
        }
    }

    /******************************************************************************//**
     * \fn void parseCriterionValues
     * \brief Parse criterion value argument names. 
    **********************************************************************************/
    void parseCriterionValues()
    {
        for (auto &tCriterionValueNode : mOperationInputData.getByName<Plato::InputData>("CriterionValues"))
        {
            this->parseNodeInputs(tCriterionValueNode, mCriterionValueArguments);
        }
    }

    /******************************************************************************//**
     * \fn void parseCriterionGradients
     * \brief Parse criterion gradient argument names. 
    **********************************************************************************/
    void parseCriterionGradients()
    {
        for (auto &tCriterionGradientNode : mOperationInputData.getByName<Plato::InputData>("CriterionGradients"))
        {
            this->parseNodeInputs(tCriterionGradientNode, mCriterionGradientArguments);
        }
    }

    /******************************************************************************//**
     * \fn void parseNodeInputs
     * \brief Parse inputs for a given parent node. 
     * \param [in] aParentNode parent node with corresponding input data
     * \param [out] aList list of Plato arugments 
    **********************************************************************************/
    void parseNodeInputs
    (const Plato::InputData &aParentNode,
     std::vector<Plato::LocalArg>& aList)
    {
        auto tDataLayout = this->parseDataLayout(aParentNode);
        for (auto &tInputNode : aParentNode.getByName<Plato::InputData>("Input"))
        {
            auto tArgumentName = Plato::Get::String(tInputNode, "ArgumentName");
            if ( tArgumentName.empty() )
                { THROWERR("ARGUMENT NAME IS EMPTY. ARGUMENT CANNOT BE AN EMPTY STRING.\n"); }
            aList.push_back(Plato::LocalArg{tDataLayout, tArgumentName});
        }
    }

    /******************************************************************************//**
     * \fn Plato::data::layout_t parseDataLayout
     * \brief Parse data layout. 
     * \param [in] aParentNode parent node with corresponding input data
     * \return data layout enum
    **********************************************************************************/
    Plato::data::layout_t parseDataLayout(const Plato::InputData& aParentNode)
    {
        auto tLayout = Plato::Get::String(aParentNode, "Layout", true);
        if(tLayout.empty() == true)
        {
            THROWERR(std::string("'LAYOUT' KEYWORD IS NOT DEFINED IN BLOCK '") + 
                Plato::toupper(aParentNode.name()) + "'. DATA 'LAYOUT' KEYWORD MUST BE DEFINED.")
        }
        auto tOutputValue = Plato::getLayout(tLayout);
        return tOutputValue;
    }

    /******************************************************************************//**
     * \fn void writeHeader
     * \brief Write header to output text file.
    **********************************************************************************/
    void writeHeader()
    {
        int tMyRank = 0;
        MPI_Comm_rank(mPlatoApp->getComm(), &tMyRank);
        if (tMyRank == 0)
        {
            mFile.open(mOutputTextFile);
            if (mFile.is_open())
            {
                this->writeArgumentNames(mDesignParamArguments);
                this->writeArgumentNames(mCriterionValueArguments);
                this->writeArgumentNames(mCriterionGradientArguments, " ");
            }
            else
            {
                THROWERR(std::string("Output file '") + mOutputTextFile + "' could not be opened. Output to file failed.")
            }
            mFile.close();
        }
    }

    /******************************************************************************//**
     * \fn void writeArgumentNames
     * \brief Write argument names to output text file. 
     * \param [in] aArgumentList argument list
     * \param [in] aEndOfLineDelimiter end-of-line delimiter (default = ", ")
    **********************************************************************************/
    void writeArgumentNames
    (const std::vector<Plato::LocalArg>& aArgumentList,
     std::string aEndOfLineDelimiter = ", ")
    {
        for(auto& tArgument : aArgumentList)
        {
            auto tIndex = &tArgument - &aArgumentList[0];
            std::string tDelimiter = static_cast<size_t>(tIndex + 1) != aArgumentList.size() ? ", " : aEndOfLineDelimiter;
            mFile << tArgument.mName << tDelimiter;
        }
    }

    /******************************************************************************//**
     * \fn void writeCriterionValue
     * \brief Write criterion values to output text file. 
    **********************************************************************************/
    void writeCriterionValue()
    {
        for(auto& tArgument : mCriterionValueArguments)
        {
            auto tParamValue = mPlatoApp->getValue(tArgument.mName);
            auto tLength = tParamValue->size();
            for(size_t tIndex = 0; tIndex < tLength; tIndex++)
            {
                mFile << std::to_string((*tParamValue)[tIndex]) << " ";
            }
        }
    }

    /******************************************************************************//**
     * \fn void writeCriterionGradients
     * \brief Write criterion gradients to output text file. 
    **********************************************************************************/
    void writeCriterionGradients()
    {
        for(auto& tArgument : mCriterionGradientArguments)
        {
            auto tLength = mPlatoApp->getNodeFieldLength(tArgument.mName);
            double* tField = mPlatoApp->getNodeFieldData(tArgument.mName);
            for(size_t tIndex = 0; tIndex < tLength; tIndex++)
            {
                mFile << std::to_string(tField[tIndex]) << " ";
            }
        }
    }

    /******************************************************************************//**
     * \fn void writeDesignParameters
     * \brief Write design parameters to output text file. 
    **********************************************************************************/
    void writeDesignParameters()
    {
        for(auto& tArgument : mDesignParamArguments)
        {
            auto tParamValue = mPlatoApp->getValue(tArgument.mName);
            auto tLength = tParamValue->size();
            for(size_t tIndex = 0; tIndex < tLength; tIndex++)
            {
                mFile << std::to_string((*tParamValue)[tIndex]) << " ";
            }
        }
    }

    /******************************************************************************//**
     * \fn void appendArguments
     * \brief Append argument list. 
     * \param [in] aFrom argument list to append to output list
     * \param [in] aTo output argument list
    **********************************************************************************/
    void appendArguments
    (const std::vector<Plato::LocalArg>& aFrom, 
     std::vector<Plato::LocalArg>& aTo)
    {
        for(const auto& tArgument : aFrom)
        {
            aTo.push_back(tArgument);
        }
    }

// private member data
private:
    std::ofstream mFile; /*!< output text file handle */
    std::string mFunction; /*!< function name */
    std::string mOperation; /*!< operation name */
    std::string mOutputTextFile; /*!< output text file name */

    std::vector<Plato::LocalArg> mDesignParamArguments; /*!< design parameter argument list */
    std::vector<Plato::LocalArg> mCriterionValueArguments; /*!< criterion value argument list */
    std::vector<Plato::LocalArg> mCriterionGradientArguments; /*!< criterion gradient argument list */

    const Plato::InputData& mOperationInputData; /*!< operation input metadata */
};
// class WriteParameterStudyData

}
// namespace Plato

namespace WriteParameterStudyData
{

TEST(PlatoTest, WriteParameterStudyData_Constructor)
{
    // define operation block
    Plato::InputData tOperation("Operation");
    tOperation.add<std::string>("Function", "WriteParameterStudyData");
    tOperation.add<std::string>("Name", "Write Parameter Study Data to Text File");
    tOperation.add<std::string>("OutputTextFile", "plato_parameter_study.txt");

    // define design parameter block
    Plato::InputData tDesignParameters("DesignParameters");
    tDesignParameters.add<std::string>("Layout", "Scalar");
    Plato::InputData tInputDP1("Input");
    tInputDP1.add<std::string>("ArgumentName", "left_radius");
    tDesignParameters.add<Plato::InputData>("Input", tInputDP1);
    Plato::InputData tInputDP2("Input");
    tInputDP2.add<std::string>("ArgumentName", "right_radius");
    tDesignParameters.add<Plato::InputData>("Input", tInputDP2);
    Plato::InputData tInputDP3("Input");
    tInputDP3.add<std::string>("ArgumentName", "height");
    tDesignParameters.add<Plato::InputData>("Input", tInputDP3);
    // append criterion value block to operation block
    tOperation.add<Plato::InputData>("DesignParameters", tDesignParameters);

    // define criterion value block
    Plato::InputData tCriterionValues("CriterionValues");
    tCriterionValues.add<std::string>("Layout", "Scalar");
    Plato::InputData tInputCV1("Input");
    tInputCV1.add<std::string>("ArgumentName", "average_stress_value");
    tCriterionValues.add<Plato::InputData>("Input", tInputCV1);
    Plato::InputData tInputCV2("Input");
    tInputCV2.add<std::string>("ArgumentName", "volume_value");
    tCriterionValues.add<Plato::InputData>("Input", tInputCV2);
    Plato::InputData tInputCV3("Input");
    tInputCV3.add<std::string>("ArgumentName", "compliance_value");
    tCriterionValues.add<Plato::InputData>("Input", tInputCV3);
    // append criterion value block to operation block
    tOperation.add<Plato::InputData>("CriterionValues", tCriterionValues);

    // define criterion gradient block
    Plato::InputData tCriterionGradients("CriterionGradients");
    tCriterionGradients.add<std::string>("Layout", "Scalar");
    Plato::InputData tInputCG1("Input");
    tInputCG1.add<std::string>("ArgumentName", "average_stress_gradient_wrt_left_radius");
    tCriterionGradients.add<Plato::InputData>("Input", tInputCG1);
    Plato::InputData tInputCG2("Input");
    tInputCG2.add<std::string>("ArgumentName", "average_stress_gradient_wrt_right_radius");
    tCriterionGradients.add<Plato::InputData>("Input", tInputCG2);
    Plato::InputData tInputCG3("Input");
    tInputCG3.add<std::string>("ArgumentName", "average_stress_gradient_wrt_height");
    tCriterionGradients.add<Plato::InputData>("Input", tInputCG3);
    Plato::InputData tInputCG4("Input");
    tInputCG4.add<std::string>("ArgumentName", "volume_gradient_wrt_left_radius");
    tCriterionGradients.add<Plato::InputData>("Input", tInputCG4);
    Plato::InputData tInputCG5("Input");
    tInputCG5.add<std::string>("ArgumentName", "volume_gradient_wrt_right_radius");
    tCriterionGradients.add<Plato::InputData>("Input", tInputCG5);
    Plato::InputData tInputCG6("Input");
    tInputCG6.add<std::string>("ArgumentName", "volume_gradient_wrt_height");
    tCriterionGradients.add<Plato::InputData>("Input", tInputCG6);
    Plato::InputData tInputCG7("Input");
    tInputCG7.add<std::string>("ArgumentName", "compliance_gradient_wrt_left_radius");
    tCriterionGradients.add<Plato::InputData>("Input", tInputCG7);
    Plato::InputData tInputCG8("Input");
    tInputCG8.add<std::string>("ArgumentName", "compliance_gradient_wrt_right_radius");
    tCriterionGradients.add<Plato::InputData>("Input", tInputCG8);
    Plato::InputData tInputCG9("Input");
    tInputCG9.add<std::string>("ArgumentName", "compliance_gradient_wrt_height");
    tCriterionGradients.add<Plato::InputData>("Input", tInputCG9);
    // append criterion value block to operation block
    tOperation.add<Plato::InputData>("CriterionGradients", tCriterionGradients);

    // create operation
    MPI_Comm tMyComm = MPI_COMM_WORLD;
    PlatoApp tPlatoApp(tMyComm);
    Plato::WriteParameterStudyData tWriteParameterStudyData(&tPlatoApp, tOperation);

    // test operation
    std::vector<Plato::LocalArg> tLocalArguments;
    tWriteParameterStudyData.getArguments(tLocalArguments);
    ASSERT_EQ(15u, tLocalArguments.size());
    std::vector<std::string> tGoldArgumentNames = { "left_radius", "right_radius", "height", "compliance_value", "average_stress_value", // @suppress("Invalid arguments")
        "volume_value", "compliance_value", "average_stress_gradient_wrt_left_radius", "average_stress_gradient_wrt_right_radius", 
        "average_stress_gradient_wrt_height", "volume_gradient_wrt_left_radius", "volume_gradient_wrt_right_radius",
        "volume_gradient_wrt_height", "compliance_gradient_wrt_left_radius", "compliance_gradient_wrt_right_radius",
        "compliance_gradient_wrt_height" };
    for(auto& tArgument : tLocalArguments)
    {
        bool tFound = std::find(tGoldArgumentNames.begin(), tGoldArgumentNames.end(), tArgument.mName) != tGoldArgumentNames.end();
        ASSERT_TRUE(tFound);
    }

    auto tReadData = PlatoTest::read_data_from_file("plato_parameter_study.txt");
    auto tGold = std::string("left_radius,right_radius,height,average_stress_value,volume_value,compliance_value,average_stress_gradient_wrt_left_radius,average_stress_gradient_wrt_right_radius,average_stress_gradient_wrt_height,volume_gradient_wrt_left_radius,volume_gradient_wrt_right_radius,volume_gradient_wrt_height,compliance_gradient_wrt_left_radius,compliance_gradient_wrt_right_radius,compliance_gradient_wrt_height");
    EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());

    auto tTrash = std::system("rm plato_parameter_study.txt");
    Plato::Utils::ignore_unused(tTrash);
}

}
// namespace WriteParameterStudyData