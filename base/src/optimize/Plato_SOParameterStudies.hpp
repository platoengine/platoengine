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
 * Plato_SOParamtereStudies.hpp
 *
 *  Created on: Oct 13, 2020
 */

#pragma once

#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include <sstream>
#include <cassert>
#include <iomanip>
#include <cstdlib>
#include <iostream>

#include "Plato_Utils.hpp"
#include "Plato_Macros.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Class use to access the diagnostics tools needed to check the analytical
 * criterion gradients and Hessians.
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class SOParameterStudies
{
public:
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    SOParameterStudies() 
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~SOParameterStudies()
    {
    }

    void doParameterStudies(Plato::Criterion<ScalarType, OrdinalType> & aObjective,
                            Plato::Criterion<ScalarType, OrdinalType> & aConstraint,
                            Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        int tNumTotalVars, tNumChangingVars;
        std::vector<int> tVarIndices;
        std::vector<double> tDefaultValues, tVarRangeBegin;
        std::vector<double> tVarRangeEnd, tVarStep;
        readInputFile(tNumTotalVars, tNumChangingVars,
                       tDefaultValues, tVarIndices,
                       tVarRangeBegin, tVarRangeEnd, tVarStep);
        setDefaultValues(aControl, tDefaultValues);
        if(tNumChangingVars == 1)
            outputSingleVariableData(aObjective, aConstraint, aControl,
                              tVarIndices, tVarRangeBegin,
                              tVarRangeEnd, tVarStep);
        else if(tNumChangingVars == 2)
            outputDoubleVariableData(aObjective, aConstraint, aControl,
                              tVarIndices, tVarRangeBegin,
                              tVarRangeEnd, tVarStep);
        return;

    }

    void readInputFile(int &aNumTotalVars,
                       int &aNumChangingVars,
                       std::vector<double> &aDefaultValues,
                       std::vector<int> &aVariableIndices,
                       std::vector<double> &aVariableRangeBegin,
                       std::vector<double> &aVariableRangeEnd,
                       std::vector<double> &aVariableStep)
    {
        // NumTotalVariables NumChangingVariables
        // DefaultVariableValues (NumTotalVariables long sequence)
        // Var1 Index Begin End Step
        // Var2 Index Begin End Step
        // ...
        FILE *fp = fopen("CalculateObjectiveInput.txt", "r");
        if(fp)
        {
            auto tTrash = fscanf(fp, "%d %d\n", &aNumTotalVars, &aNumChangingVars);
            Plato::Utils::ignore_unused(tTrash); 
            for(int i=0; i<aNumTotalVars; ++i)
            {
                double tVal;
                tTrash = fscanf(fp, "%lf", &tVal);
                Plato::Utils::ignore_unused(tTrash);
                aDefaultValues.push_back(tVal);
            }
            for(int i=0; i<aNumChangingVars; ++i)
            {
                int tIndex;
                double tBegin, tEnd, tStep;
                tTrash = fscanf(fp, "%d %lf %lf %lf", &tIndex, &tBegin, &tEnd, &tStep);
                Plato::Utils::ignore_unused(tTrash);
                aVariableIndices.push_back(tIndex);
                aVariableRangeBegin.push_back(tBegin);
                aVariableRangeEnd.push_back(tEnd);
                aVariableStep.push_back(tStep);
            }
        }
        fclose(fp);
    }

    void setDefaultValues(Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                          std::vector<double> &aDefaultValues)
    {
        for(size_t i=0; i<aDefaultValues.size(); i++)
            aControl[0][i] = aDefaultValues[i]; 
    }

    void outputSingleVariableData(Plato::Criterion<ScalarType, OrdinalType> & aObjective,
                                       Plato::Criterion<ScalarType, OrdinalType> & aConstraint,
                              Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                              std::vector<int> &aVariableIndices,
                              std::vector<double> &aVariableRangeBegin,
                              std::vector<double> &aVariableRangeEnd,
                              std::vector<double> &aVariableStep)
    {
        double tStart = aVariableRangeBegin[0];
        double tEnd = aVariableRangeEnd[0];
        double tCur = tStart;
        double tStep = aVariableStep[0];
        FILE *fp = fopen("diag.txt", "w");
        fclose(fp);
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tObjGradient = aControl.create();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tConGradient = aControl.create();
        while(tCur <= (tEnd + .00001))
        {
            aControl[0][aVariableIndices[0]] = tCur;
            // Objective/Gradient
            double tObjValue = aObjective.value(aControl);
            aObjective.cacheData();
            aObjective.gradient(aControl, *tObjGradient);
            // Constraint/Gradient
            double tConValue = aConstraint.value(aControl);
            aConstraint.cacheData();
            aConstraint.gradient(aControl, *tConGradient);

            fp = fopen("diag.txt", "a");
            fprintf(fp, "%.10e %.10e %.10e %.10e %.10e\n", tCur, tObjValue, tConValue, 
                                       (*tObjGradient)[0][0], (*tConGradient)[0][0]); 
            fclose(fp);
            tCur += tStep;
        }
    }

    void outputDoubleVariableData(Plato::Criterion<ScalarType, OrdinalType> & aObjective,
                                  Plato::Criterion<ScalarType, OrdinalType> & aConstraint,
                              Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                              std::vector<int> &aVariableIndices,
                              std::vector<double> &aVariableRangeBegin,
                              std::vector<double> &aVariableRangeEnd,
                              std::vector<double> &aVariableStep)
    {
        double tOuterStart = aVariableRangeBegin[0];
        double tInnerStart = aVariableRangeBegin[1];
        double tOuterEnd = aVariableRangeEnd[0];
        double tInnerEnd = aVariableRangeEnd[1];
        double tOuterCur = tOuterStart;
        double tOuterStep = aVariableStep[0];
        double tInnerStep = aVariableStep[1];
        int tOuterIndex = aVariableIndices[0];
        int tInnerIndex = aVariableIndices[1];
        FILE *fp = fopen("diag.txt", "w");
        fclose(fp);
        while(tOuterCur <= (tOuterEnd + .00001))
        {
            aControl[0][tOuterIndex] = tOuterCur;
            double tInnerCur = tInnerStart;
            while(tInnerCur <= (tInnerEnd + .0001))
            {
                aControl[0][tInnerIndex] = tInnerCur;
                double tObjValue = aObjective.value(aControl);
                fp = fopen("diag.txt", "a");
                fprintf(fp, "%.10e %.10e %.10e\n", tOuterCur, tInnerCur, tObjValue); 
                fclose(fp);
    
                tInnerCur += tInnerStep;
            }
            tOuterCur += tOuterStep;
        }
    }

private:

    SOParameterStudies(const Plato::SOParameterStudies<ScalarType, OrdinalType> & aRhs);
    Plato::SOParameterStudies<ScalarType, OrdinalType> & operator=(const Plato::SOParameterStudies<ScalarType, OrdinalType> & aRhs);
};
// class SOParameterStudies

}
// namespace Plato

