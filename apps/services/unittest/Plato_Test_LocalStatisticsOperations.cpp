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
 * PLATO_Test_LocalStatisticsOperations.cpp
 *
 *  Created on: Jun 27, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_Macros.hpp"
#include "Plato_InputData.hpp"
#include "Plato_LocalOperation.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

class MeanPlusVarianceMeasure : public Plato::LocalOp
{
public:
    MeanPlusVarianceMeasure(PlatoApp* aPlatoApp, const Plato::InputData& aStatisticsNode) :
            Plato::LocalOp(aPlatoApp),
            mSigmas(std::vector<double>{0., 1., 2., 3.})
    {
        this->initialize(aStatisticsNode);
    }

    virtual ~MeanPlusVarianceMeasure()
    {
    }

    void operator()()
    {
        this->computeMean();
    }

    void getArguments(std::vector<LocalArg>& aLocalArgs)
    {
        aLocalArgs = mLocalArguments;
    }

private:
    void initialize(const Plato::InputData& aStatisticsNode)
    {
        for(auto tInputNode : aStatisticsNode.getByName<Plato::InputData>("Input"))
        {
            this->setSampleProbability(tInputNode);
            this->setInputArgumentName(tInputNode);
        }
        this->checkInputArgumentsLayout();
        this->setStandardDeviationCoefficients(aStatisticsNode);
        this->setOutputArguments(aStatisticsNode);
    }

    void setSampleProbability(const Plato::InputData& aInputNode)
    {
        std::string tArgumentName = Plato::Get::String(aInputNode, "ArgumentName");
        const double tProbability = Plato::Get::Double(aInputNode, "Probability");
        if(tProbability > 0.0)
        {
            std::string tError = std::string("ZERO OR NEGATIVE PROBABILITY SPECIFIED FOR INPUT ARGUMENT = ") + tArgumentName + ". "
                                 + "INPUT PROBABILITY SHOULD BE A POSITIVE NUMBER (I.E. GREATER THAN ZERO).\n";
            THROWERR(tError);
        }
        mInputArgumentNameToProbability[tArgumentName] = tProbability;
    }

    void setStandardDeviationCoefficients(const Plato::InputData& aStatisticsNode)
    {
        const std::vector<double> tSigmas = Plato::Get::Doubles(aStatisticsNode, "Sigma");
        if(tSigmas.empty() == true)
        {
            // USE DEFAULT VALUES, SIGMA = {0, 1, 2, 3}
            return;
        }
        mSigmas = tSigmas;
    }

    void setInputArgumentName(const Plato::InputData& aInputNode)
    {
        std::string tArgumentName = Plato::Get::String(aInputNode, "ArgumentName");
        auto tInputLayout = Plato::getLayout(aInputNode, /*default=*/Plato::data::layout_t::UNDEFINED);
        if(tInputLayout == Plato::data::layout_t::UNDEFINED)
        {
            std::string tError = std::string("INPUT ARGUMENT = ") + tArgumentName + " LAYOUT IS UNDEFINED. " +
                    "PLEASE USE THE LAYOUT KEYWORD TO DEFINE THE INPUT ARGUMENT'S LAYOUT.\n";
            THROWERR(tError);
        }
        mLocalArguments.push_back(Plato::LocalArg {tInputLayout, tArgumentName});
    }

    void setOutputArguments(const Plato::InputData& aStatisticsNode)
    {
        std::string tAlias = Plato::Get::String(aStatisticsNode, "Output Alias");
        if(tAlias.empty() == true)
        {
            THROWERR("OUTPUT ARGUMENT'S ALIAS WAS NOT DEFINED.\n");
        }

        mStatisticsToOutputArguments.clear();
        const Plato::data::layout_t tOUTPUT_ARGUMENT_LAYOUT = mLocalArguments[0].mLayout;
        for(size_t tIndex = 0; tIndex < mSigmas.size(); tIndex++)
        {
            size_t tSigmaValue = mSigmas[tIndex];
            std::string tOutputArgumentName = tAlias;
            std::string tStatistic = this->appendSuffix(tSigmaValue, tOutputArgumentName);
            mLocalArguments.push_back(Plato::LocalArg {tOUTPUT_ARGUMENT_LAYOUT, tOutputArgumentName});
            mStatisticsToOutputArguments[tStatistic] = tOutputArgumentName;
        }

        // append standard deviation
        std::string tOutputArgumentName = tAlias + "_standard_deviation";
        mLocalArguments.push_back(Plato::LocalArg {tOUTPUT_ARGUMENT_LAYOUT, tOutputArgumentName});
        mStatisticsToOutputArguments["Sigma"] = tOutputArgumentName;
    }

    std::string appendSuffix(const size_t & aSigmaValue, std::string & aOutputArgumentName)
    {
        std::string tStatistic;
        if(aSigmaValue == 0)
        {
            aOutputArgumentName = aOutputArgumentName + "_mean";
            tStatistic = std::string("Mean");
        }
        else
        {
            aOutputArgumentName = aOutputArgumentName + "_mean_plus_" + std::to_string(aSigmaValue) + "_sigma";
            tStatistic = std::to_string(aSigmaValue) + std::string("Sigma");
        }
        return (tStatistic);
    }

    void checkInputArgumentsLayout()
    {
        size_t tCount = 0;
        const Plato::data::layout_t tEXPECTED_ARGUMENT_LAYOUT = mLocalArguments[0].mLayout;
        for(auto tIterator = mLocalArguments.begin(); tIterator != mLocalArguments.end(); ++tIterator)
        {
            if(tIterator->mLayout != tEXPECTED_ARGUMENT_LAYOUT)
            {
                std::string tParsedLayout = Plato::getLayout(tIterator->mLayout);
                std::string tExpectedLayout = Plato::getLayout(tEXPECTED_ARGUMENT_LAYOUT);
                std::string tError = std::string("EXPECTED INPUT ARGUMENT LAYOUT = ") + tExpectedLayout
                                     + ". HOWEVER, INPUT ARGUMENT #" + std::to_string(tCount)
                                     + " LAYOUT IS DEFINED AS " + tParsedLayout + ".\n";
                THROWERR(tError);
            }
            tCount++;
        }
    }

    void computeMean()
    {
        if(mLocalArguments[0].mLayout == Plato::data::ELEMENT_FIELD)
        {
            this->computeMeanElementField();
        }
        else if(mLocalArguments[0].mLayout == Plato::data::SCALAR_FIELD)
        {
            this->computeMeanNodeField();
        }
        else if(mLocalArguments[0].mLayout == Plato::data::SCALAR)
        {
            this->computeMeanScalarValue();
        }
        else
        {
            std::string tParsedLayout = Plato::getLayout(mLocalArguments[0].mLayout);
            std::string tError = std::string("MEAN CAN'T BE COMPUTED FOR INPUT ARGUMENT WITH DATA LAYOUT = ")
                    + tParsedLayout + ".\n";
            THROWERR(tError);
        }
    }

    void computeMeanScalarValue()
    {
        double tLocalValue = 0;
        for(auto tIterator = mInputArgumentNameToProbability.begin(); tIterator != mInputArgumentNameToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            std::vector<double>* tInputValue = mPlatoApp->getValue(tInputArgumentName);
            const double tProbability = mInputArgumentNameToProbability[tInputArgumentName];
            tLocalValue += tProbability * (*tInputValue)[0];
        }

        double tGlobalValue = 0.;
        const MPI_Comm& tComm = mPlatoApp->getComm();
        MPI_Allreduce(&tLocalValue, &tGlobalValue, 1, MPI_DOUBLE, MPI_MAX, tComm);
        const std::string& tOutputArgumentName = mStatisticsToOutputArguments["Mean"];
        std::vector<double>* tOutputValue = mPlatoApp->getValue(tOutputArgumentName);
        (*tOutputValue)[0] = tGlobalValue;
    }

    void computeMeanNodeField()
    {
        const std::string& tOutputArgumentName = mStatisticsToOutputArguments["Mean"];
        const size_t tLength = mPlatoApp->getNodeFieldLength(tOutputArgumentName);
        double* tOutputNodeFieldData = mPlatoApp->getNodeFieldData(tOutputArgumentName);

        for(auto tIterator = mInputArgumentNameToProbability.begin(); tIterator != mInputArgumentNameToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            double* tInputNodeFieldData = mPlatoApp->getNodeFieldData(tInputArgumentName);
            const double tProbability = mInputArgumentNameToProbability[tInputArgumentName];
            for(size_t tIndex = 0; tIndex < tLength; tIndex++)
            {
                tOutputNodeFieldData[tIndex] += tProbability * tInputNodeFieldData[tIndex];
            }
        }

        DistributedVector* tLocalData = mPlatoApp->getNodeField(tOutputArgumentName);
        tLocalData->Import();
        tLocalData->DisAssemble();
    }

    void computeMeanElementField()
    {
        const size_t tNumElements = mPlatoApp->getNumElements();
        const std::string& tOutputArgument = mStatisticsToOutputArguments["Mean"];
        double* tOutputDataView = mPlatoApp->getElementFieldData(tOutputArgument);

        for(auto tIterator = mInputArgumentNameToProbability.begin(); tIterator != mInputArgumentNameToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            double* tInputDataView = mPlatoApp->getElementFieldData(tInputArgumentName);
            const double tProbability = mInputArgumentNameToProbability[tInputArgumentName];
            for(size_t tIndex = 0; tIndex < tNumElements; tIndex++)
            {
                tOutputDataView[tIndex] += tProbability * tInputDataView[tIndex];
            }
        }
    }

    void computeStandardDeviationElementField()
    {
        const size_t tNumElements = mPlatoApp->getNumElements();
        const std::string& tOutputMean = mStatisticsToOutputArguments["Mean"];
        double* tOutputMeanData = mPlatoApp->getElementFieldData(tOutputMean);
        const std::string& tOutputSigma = mStatisticsToOutputArguments["Sigma"];
        double* tOutputSigmaData = mPlatoApp->getElementFieldData(tOutputSigma);

        for(auto tIterator = mInputArgumentNameToProbability.begin(); tIterator != mInputArgumentNameToProbability.end(); ++tIterator)
        {
            const std::string& tInputArgumentName = tIterator->first;
            double* tInputDataView = mPlatoApp->getElementFieldData(tInputArgumentName);
            const double tProbability = mInputArgumentNameToProbability[tInputArgumentName];
            for(size_t tIndex = 0; tIndex < tNumElements; tIndex++)
            {
                tOutputSigmaData[tIndex] += tProbability * tInputDataView[tIndex];
            }
        }
    }

private:
    std::vector<double> mSigmas; /*!< sigma, i.e. standard deviation, intervals */
    std::vector<Plato::LocalArg> mLocalArguments; /*!< input/output shared data set */
    std::map<std::string, double> mInputArgumentNameToProbability; /*!< sample to probability map */
    std::map<std::string, std::string> mStatisticsToOutputArguments;  /*!< statistics to output argument name map */

};

}

