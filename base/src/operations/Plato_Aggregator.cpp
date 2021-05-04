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
 * Plato_Aggregator.cpp
 *
 *  Created on: Jun 27, 2019
 */

#include "PlatoApp.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_Aggregator.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

Aggregator::Aggregator(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        Plato::LocalOp(aPlatoApp)
{
    mReportStatus = Get::Bool(aNode, "Report", false);

    Plato::InputData tWeightNode = Plato::Get::InputData(aNode, "Weighting");
    for(auto tNode : tWeightNode.getByName<Plato::InputData>("Weight"))
    {
        mWeights.push_back(Plato::Get::Double(tNode, "Value"));
    }
    mWeightMethod = Plato::Get::String(tWeightNode, "Method",/*toUpper=*/true);
    if(mWeightMethod == "FIXED")
    {
        mLimitWeight = Plato::Get::Double(tWeightNode, "Limit");
        if(mLimitWeight == 0.0)
            mLimitWeight = 1e9;
        Plato::InputData tBasesNode = Plato::Get::InputData(tWeightNode, "Bases");
        for(auto tInputNode : tBasesNode.getByName<Plato::InputData>("Input"))
        {
            std::string tBasisName = Plato::Get::String(tInputNode, "ArgumentName");
            mWeightBases.push_back(tBasisName);
        }
    }
    Plato::InputData tNormalsNode = Plato::Get::InputData(tWeightNode, "Normals");
    for(auto tInputNode : tNormalsNode.getByName<Plato::InputData>("Input"))
    {
        std::string tNormalName = Plato::Get::String(tInputNode, "ArgumentName");
        mWeightNormals.push_back(tNormalName);
    }
    // TODO check that number of weights == number of fields, etc.

    for(Plato::InputData tNode : aNode.getByName<Plato::InputData>("Aggregate"))
    {
        AggStruct tNewAggStruct;
        Plato::InputData tOutputNode = Plato::Get::InputData(tNode, "Output");
        if(tNode.size<Plato::InputData>("Output") > 1)
        {
            throw ParsingException("PlatoApp::Aggregator: more than one Output specified.");
        }
        tNewAggStruct.mOutputName = Plato::Get::String(tOutputNode, "ArgumentName");

        for(Plato::InputData tInputNode : tNode.getByName<Plato::InputData>("Input"))
        {
            tNewAggStruct.mInputNames.push_back(Plato::Get::String(tInputNode, "ArgumentName"));
        }
        tNewAggStruct.mLayout = Plato::getLayout(tNode);

        mAggStructs.push_back(tNewAggStruct);
    }
}

void Aggregator::reportStatus(const std::stringstream& aStream) const
{
    reportStatus(aStream.str());
}

void Aggregator::reportStatus(const std::string& aString) const
{
    if(mReportStatus)
    {
        Plato::Console::Alert(aString);
    }
}


void Aggregator::operator()()
{
    // begin timer if timing
    if(mPlatoApp->getTimersTree())
    {
        mPlatoApp->getTimersTree()->begin_partition(Plato::timer_partition_t::timer_partition_t::aggregator);
    }

    auto tWeights = getWeights();

    reportStatus("#--- Aggregator ----------------------------------------------------------------");
    for(AggStruct& tMyAggStruct : mAggStructs)
    {
        aggregate(tMyAggStruct, tWeights);
    }
    reportStatus("#-------------------------------------------------------------------------------");

    // end: "aggregator"
    if(mPlatoApp->getTimersTree())
    {
        mPlatoApp->getTimersTree()->end_partition();
    }

    return;
}

void Aggregator::aggregateScalarField(const AggStruct& aAggStruct, const decltype(mWeights)& aWeights)
{
    using std::setw;
    using std::setprecision;

    const int fw = 10; // field width
    const int pn = 4;  // precision

    std::stringstream tMessage;
    tMessage << "# Scalar Field";
    reportStatus(tMessage.str());
    tMessage.str(std::string());
    tMessage << "#";
    tMessage << setw(fw) << "Input";
    tMessage << setw(fw) << "Norm";
    tMessage << setw(fw) << "Weight";
    if(!mWeightNormals.empty())
    {
        tMessage << setw(fw) << "Normal";
        tMessage << setw(fw) << "Adj Wt";
    }
    tMessage << setw(fw) << "Weighted";
    tMessage << setw(fw) << "Name";
    reportStatus(tMessage.str());

    auto& tField = *(mPlatoApp->getNodeField(aAggStruct.mOutputName));
    double* tToData;
    tField.ExtractView(&tToData);
    int tDataLength = tField.MyLength();

    int tNvals = aAggStruct.mInputNames.size();
    std::vector<double*> tFromData(tNvals);
    for(int tIval = 0; tIval < tNvals; tIval++)
    {
        auto tPfield = mPlatoApp->getNodeField(aAggStruct.mInputNames[tIval]);
        tPfield->ExtractView(&tFromData[tIval]);

    }

    std::vector<double> tFromDataNorm(tNvals, 0.0);
    for(int tIndex = 0; tIndex < tDataLength; tIndex++)
    {
        tToData[tIndex] = 0.0;
        for(int tIval = 0; tIval < tNvals; tIval++)
        {
            tToData[tIndex] += tFromData[tIval][tIndex] * aWeights[tIval];
            tFromDataNorm[tIval] += tFromData[tIval][tIndex]*tFromData[tIval][tIndex];
        }
    }
    for(int tIval = 0; tIval < tNvals; tIval++)
    {
        if(tFromDataNorm[tIval] != 0.0)
        {
            tFromDataNorm[tIval] = sqrt(tFromDataNorm[tIval]);
        }

        auto tInputName = mPlatoApp->getSharedDataName(aAggStruct.mInputNames[tIval]);
        tMessage << "#";
        tMessage << setw(fw) << tIval;
        tMessage << setw(fw) << setprecision(pn) << tFromDataNorm[tIval];
        tMessage << setw(fw) << setprecision(pn) << mWeights[tIval];
        if(!mWeightNormals.empty())
        {
            std::vector<double>* data = mPlatoApp->getValue(mWeightNormals[tIval]);
            tMessage << setw(fw) << setprecision(pn) << *(data->data());
            tMessage << setw(fw) << setprecision(pn) << aWeights[tIval];
        }
        tMessage << setw(fw) << setprecision(pn) << tFromDataNorm[tIval]*aWeights[tIval];
        tMessage << "      " << tInputName;
        reportStatus(tMessage.str());
    }
}

void Aggregator::aggregateScalar(const AggStruct& aAggStruct, const decltype(mWeights)& aWeights)
{
    using std::setw;
    using std::setprecision;

    const int fw = 10;
    const int pn = 4;

    std::stringstream tMessage;
    tMessage << "# Scalar";
    reportStatus(tMessage.str());
    tMessage.str(std::string());
    tMessage << "#";
    tMessage << setw(fw) << "Input";
    tMessage << setw(fw) << "Value";
    tMessage << setw(fw) << "Weight";
    if(!mWeightNormals.empty())
    {
        tMessage << setw(fw) << "Normal";
        tMessage << setw(fw) << "Adj Wt";
    }
    tMessage << setw(fw) << "Weighted";
    tMessage << setw(fw) << "Name";
    reportStatus(tMessage.str());

    std::vector<double>& tToData = *(mPlatoApp->getValue(aAggStruct.mOutputName));

    unsigned int tDataLength = 0;
    int tNvals = aAggStruct.mInputNames.size();
    std::vector<double*> tFromData(tNvals);

    // read first input value
    std::vector<double>* tMyValue = mPlatoApp->getValue(aAggStruct.mInputNames[0]);
    tFromData[0] = tMyValue->data();
    tDataLength = tMyValue->size();

    // read remaining input values
    for(int tIval = 1; tIval < tNvals; tIval++)
    {
        tMyValue = mPlatoApp->getValue(aAggStruct.mInputNames[tIval]);
        tFromData[tIval] = tMyValue->data();
        if(tMyValue->size() != tDataLength)
        {
            throw ParsingException("PlatoApp::Aggregator: attempted to aggregate vectors of differing lengths.");
        }
    }

    tToData.resize(tDataLength);
    std::fill(tToData.begin(), tToData.end(), 0.0);
    for(unsigned int tIndex = 0; tIndex < tDataLength; tIndex++)
    {
        for(int tIval = 0; tIval < tNvals; tIval++)
        {
            auto tInputName = mPlatoApp->getSharedDataName(aAggStruct.mInputNames[tIval]);
            tMessage << "#";
            tMessage << setw(fw) << tIval;
            tMessage << setw(fw) << setprecision(pn) << tFromData[tIval][tIndex];
            tMessage << setw(fw) << setprecision(pn) << mWeights[tIval];
            if(!mWeightNormals.empty())
            {
                std::vector<double>* data = mPlatoApp->getValue(mWeightNormals[tIval]);
                tMessage << setw(fw) << setprecision(pn) << *(data->data());
                tMessage << setw(fw) << setprecision(pn) << aWeights[tIval];
            }
            tMessage << setw(fw) << setprecision(pn) << tFromData[tIval][tIndex]*aWeights[tIval];
            tMessage << "      " << tInputName;
            reportStatus(tMessage);

            tToData[tIndex] += tFromData[tIval][tIndex] * aWeights[tIval];
        }

        tMessage << "#";
        tMessage << setw(fw) << "Output";
        tMessage << setw(fw) << setprecision(pn) << tToData[tIndex];
        reportStatus(tMessage);
    }
}

void Aggregator::aggregate(const AggStruct& aAggStruct, decltype(mWeights) aWeights)
{
    if(aAggStruct.mLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        aggregateScalarField(aAggStruct, aWeights);
    }
    else if(aAggStruct.mLayout == Plato::data::layout_t::SCALAR)
    {
        aggregateScalar(aAggStruct, aWeights);
    }
    else
    {
        std::stringstream tMessage;
        tMessage << "Unknown 'Layout' (" << aAggStruct.mLayout << ") specified in PlatoApp::Aggregator."
                 << std::endl;
        Plato::ParsingException tParsingException(tMessage.str());
        throw tParsingException;
    }
}

decltype(Aggregator::mWeights) Aggregator::getWeights()
{
    decltype(mWeights) tWeights(mWeights);
    if(!mWeightBases.empty())
    {
        int tNvals = mWeightBases.size();
        std::vector<double> tWeightBases(tNvals);
        for(int tIval = 0; tIval < tNvals; tIval++)
        {
            std::vector<double>* tData = mPlatoApp->getValue(mWeightBases[tIval]);
            tWeightBases[tIval] = *(tData->data());
        }
        double tEtaSum = 0.0;
        for(int tIval = 1; tIval < tNvals; tIval++)
        {
            tEtaSum += mWeights[tIval];
        }
        if(!mWeightNormals.empty())
        {
            tNvals = mWeightNormals.size();
            std::vector<double> tNormals(tNvals);
            for(int tIval = 0; tIval < tNvals; tIval++)
            {
                std::vector<double>* data = mPlatoApp->getValue(mWeightNormals[tIval]);
                tNormals[tIval] = *(data->data());
            }
            for(int tIval = 0; tIval < tNvals; tIval++)
            {
                tWeights[tIval] = tWeightBases[0] * mWeights[tIval] / (tWeightBases[tIval] * (1.0 - tEtaSum));
                tWeights[tIval] *= tNormals[tIval] / tNormals[0];
                if(tWeights[tIval] > mLimitWeight)
                    tWeights[tIval] = mLimitWeight;
            }
        }
        else
        {
            for(int tIval = 0; tIval < tNvals; tIval++)
            {
                tWeights[tIval] = tWeightBases[0] * mWeights[tIval] / (tWeightBases[tIval] * (1.0 - tEtaSum));
                if(tWeights[tIval] > mLimitWeight)
                    tWeights[tIval] = mLimitWeight;
            }
        }
    }
//    else
//    {
        if(!mWeightNormals.empty())
        {
            int tNvals = mWeightNormals.size();
            std::vector<double> N(tNvals);
            for(int tIval = 0; tIval < tNvals; tIval++)
            {
                std::vector<double>* data = mPlatoApp->getValue(mWeightNormals[tIval]);
                N[tIval] = *(data->data());
            }
            for(int tIval = 0; tIval < tNvals; tIval++)
            {
                tWeights[tIval] /= N[tIval];
            }
        }
//    }
    return tWeights;
}

void Aggregator::getArguments(std::vector<Plato::LocalArg> & aLocalArgs)
{
    for(auto& tArgument : mAggStructs)
    {
        aLocalArgs.push_back(Plato::LocalArg {tArgument.mLayout, tArgument.mOutputName});
        for(auto& tInput : tArgument.mInputNames)
        {
            aLocalArgs.push_back(Plato::LocalArg {tArgument.mLayout, tInput});
        }
    }
    for(auto& tArgument : mWeightBases)
    {
        aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR, tArgument});
    }
    for(auto& tArgument : mWeightNormals)
    {
        aLocalArgs.push_back(Plato::LocalArg {Plato::data::layout_t::SCALAR, tArgument});
    }
}

}
