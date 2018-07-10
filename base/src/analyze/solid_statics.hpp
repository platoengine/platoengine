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

#ifndef SOLID_STATICS
#define SOLID_STATICS

#include "Intrepid_Basis.hpp"
#include "Intrepid_FieldContainer.hpp"
#include "Shards_CellTopology.hpp"
#include "Plato_Parser.hpp"

#include "bc.hpp"
#include "material_container.hpp"

class SystemContainer;
class LightMP;
class DistributedCrsMatrix;
class DistributedVector;

namespace Plato { class PenaltyModel; }

/******************************************************************************/
class SolidStatics
/******************************************************************************/
{
  public:
    SolidStatics(SystemContainer& sys, LightMP& ren);

    void buildStiffnessMatrix( DistributedCrsMatrix& K, 
                               const DistributedVector& topology,
                               Plato::PenaltyModel* p);

    void applyConstraints( DistributedCrsMatrix& stiffMatrix, 
                           DistributedVector& forcingVector, 
                           Real time=0.0 );

    void updateMaterialState( Real time );
    void lagrangianUpdate();

    void computeInternalForces( DistributedVector& forcingVector, Real time );
    void computeExternalForces( DistributedVector& forcingVector, Real time );

    void updateDisplacement( DistributedVector& x, 
                             DistributedVector& B,
                             DistributedCrsMatrix& A, Real time );
    
    void computeInternalEnergy( DistributedVector* topology, 
                                Plato::PenaltyModel* penaltyModel,
                                double* ie, DistributedVector* iegradient);
    
  private:
    void Parse( pugi::xml_node& input, LightMP& ren );

    SystemContainer *mySystem;
    int nHourglassModes;
    Intrepid::FieldContainer<double>* hourglassModes;

    VarIndex STRESS, STRAIN_INCREMENT;

    VarIndex DISPX, DISPY, DISPZ;
    VarIndex TOTAL_DX, TOTAL_DY, TOTAL_DZ;

    vector< BoundaryCondition<Real>* > essentialBCs;
    vector< BoundaryCondition<Real>* > naturalBCs;

    DataContainer* myDataContainer;
    MaterialContainer* myMaterialContainer;
    DataMesh* myDataMesh;

    pugi::xml_node solverspec;
};
/******************************************************************************/
#endif

