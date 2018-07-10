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
