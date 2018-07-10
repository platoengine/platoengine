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
 * Plato_StructuralTopologyOptimization.hpp
 *
 *  Created on: Nov 16, 2017
 */

#ifndef PLATO_STRUCTURALTOPOLOGYOPTIMIZATION_HPP_
#define PLATO_STRUCTURALTOPOLOGYOPTIMIZATION_HPP_

#include <vector>

#include "Epetra_SerialDenseSolver.h"
#include "Epetra_SerialDenseMatrix.h"
#include "Epetra_SerialDenseVector.h"

namespace Plato
{

/* THIS SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER FOLLOWS THE IMPLEMENTATION
 * FROM A 99 LINE TOPOLOGY OPTIMIZATION CODE BY OLE SIGMUND, JANUARY 2000 */
class StructuralTopologyOptimization
{
public:
    StructuralTopologyOptimization(const double & aPoissonRatio,
                                   const double & aElasticModulus,
                                   const int & aNumElementsXdirection,
                                   const int & aNumElementsYdirection);
    ~StructuralTopologyOptimization();

    int getGlobalNumDofs() const;
    int getNumDesignVariables() const;
    double getVolumeFraction() const;
    double getStiffnessMatrixOneNorm() const;

    const Epetra_SerialDenseVector & getFixedDOFs() const;
    const Epetra_SerialDenseVector & getForceVector() const;
    const Epetra_SerialDenseVector & getDisplacements() const;
    const Epetra_SerialDenseMatrix & getStiffnessMatrix() const;
    const Epetra_SerialDenseMatrix & getElementStiffnessMatrix() const;

    void setPenaltyFactor(const double & aInput);
    void setFilterRadius(const double & aInput);
    void setVolumeFraction(const double & aInput);
    void setNumElements(const int & aNumElementsXdirection, const int & aNumElementsYdirection);

    void setFixedDOFs(const Epetra_SerialDenseVector & aInput);
    void setForceVector(const Epetra_SerialDenseVector & aInput);

    void cacheState();
    void disableCacheState();
    bool isStateCached() const;
    void solve(const Epetra_SerialDenseVector & aControl);

    double computeCompliance(const Epetra_SerialDenseVector & aControl);
    double computeVolumeMisfit(const Epetra_SerialDenseVector & aControl);
    void computeVolumeGradient(const Epetra_SerialDenseVector & aControl, Epetra_SerialDenseVector & aOutput);
    void computeComplianceGradient(const Epetra_SerialDenseVector & aControl, Epetra_SerialDenseVector & aOutput);
    void computeComplianceHessianTimesVector(const Epetra_SerialDenseVector & aControl,
                                             const Epetra_SerialDenseVector & aVector,
                                             Epetra_SerialDenseVector & aOutput);
    void applySensitivityFilter(const Epetra_SerialDenseVector & aControl,
                                const Epetra_SerialDenseVector & aUnfilteredGradient,
                                Epetra_SerialDenseVector & aFilteredGradient);

private:
    void initialize();
    void checkInputs();

    void fillElementStiffnessMatrix(const std::vector<double> & aStiffness);
    void assembleStiffnessMatrix(const Epetra_SerialDenseVector & aControl);
    void setElementDisplacements(const std::vector<int> & aElemDofs, const Epetra_SerialDenseVector & aDisplacements);

private:
    int mNumElemDOFs;
    int mGlobalNumDofs;
    int mNumDesignVariables;
    int mNumElemXDirection;
    int mNumElemYDirection;

    double mPenalty;
    double mFilterRadius;
    double mPoissonRatio;
    double mElasticModulus;
    double mVolumeFraction;
    double mStiffnessMatrixOneNorm;

    bool mIsForceSet;
    bool mIsStateCached;
    bool mIsDirichletConditionsSet;

    Epetra_SerialDenseSolver mSolver;

    Epetra_SerialDenseVector mForce;
    Epetra_SerialDenseVector mFixedDOFs;
    Epetra_SerialDenseVector mWorkVector;
    Epetra_SerialDenseVector mElemDisplacements;
    Epetra_SerialDenseVector mTrialDisplacements;
    Epetra_SerialDenseVector mCurrentDisplacements;
    Epetra_SerialDenseVector mElemStiffTimesElemDisplacements;

    Epetra_SerialDenseMatrix mStiffnessMatrix;
    Epetra_SerialDenseMatrix mElementStiffnessMatrix;

private:
    StructuralTopologyOptimization(const Plato::StructuralTopologyOptimization & aRhs);
    Plato::StructuralTopologyOptimization & operator=(const Plato::StructuralTopologyOptimization & aRhs);
};

} //namespace Plato

#endif /* PLATO_STRUCTURALTOPOLOGYOPTIMIZATION_HPP_ */
