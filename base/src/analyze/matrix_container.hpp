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

#ifndef MATRIX_CONTAINER_HPP
#define MATRIX_CONTAINER_HPP

#include "Epetra_CrsMatrix.h"
#include "Epetra_Vector.h"
#include "Epetra_MpiComm.h"
#include "Epetra_Map.h"
#include "Epetra_Export.h"
#include "Epetra_Import.h"

#include "Intrepid_FieldContainer.hpp"

#include "types.hpp"
#include "Plato_Parser.hpp"

#include <vector>
using std::vector;

class DataMesh;
class DistributedCrsMatrix;
class DistributedVector;
class LightMP;

template<typename Type>
class PeriodicBoundaryCondition;

/*!
  The SystemContainer class manages the node map, dof map, and communication.
*/

class SystemContainer
{
  public:

    //! SystemContainer constructor.
    /*!
      \param mesh The DataMesh on which the maps and graph are based.
      \param dofsPerNode Number of degrees of freedom per node.
    */
    SystemContainer(DataMesh* mesh, int dofsPerNode, pugi::xml_node& input);
    ~SystemContainer();

    int getDofsPerNode(){ return dofsPerNode; }
    DataMesh* getMesh(){ return myMesh; }

    friend class DistributedCrsMatrix;
    friend class DistributedVector;

    Epetra_Map* getRowMap(){ return RowMap; }
    Epetra_Map* getOverlapRowMap(){ return OverlapRowMap; }

    Epetra_Map* getNodeRowMap(){ return NodeRowMap; }
    Epetra_Map* getNodeOverlapRowMap(){ return NodeOverlapRowMap; }
    
  protected:

    void Initialize(DataMesh* mesh, int dofsPerNode);
    void zeroSet();

    Epetra_Map          *RowMap;        // degree of freedom map for local-node-list dofs
    Epetra_Map          *OverlapRowMap; // degree of freedom map for total-node-list dofs
    Epetra_Map          *NodeRowMap;        // node map for local-node-list dofs
    Epetra_Map          *NodeOverlapRowMap; // node map for total-node-list dofs
    Epetra_Comm         *Comm;
    Epetra_CrsGraph     *k_graph;
    Epetra_CrsGraph     *k_overlap_graph;
    Epetra_Export*      assemblyExporter;
    Epetra_Import       *myImporter;
    Epetra_Import       *myNodeImporter;

    DataMesh            *myMesh;

    int dofsPerNode;

};

/******************************************************************************/
class DistributedEntity
/******************************************************************************/
{
  public:
    DistributedEntity(SystemContainer *sys) { mySystem = sys; }

  protected:
    DistributedEntity(){};
    SystemContainer    *mySystem;
};




/******************************************************************************/
class DistributedCrsMatrix : public DistributedEntity
/******************************************************************************/
{
  public:
    //! DistributedCrsMatrix constructor.
    /*!
      \param sys SystemContainer pointer with the DofMap and graph.
    */
    DistributedCrsMatrix(SystemContainer *sys);
    void Assemble( Intrepid::FieldContainer<double>& localMatrix, int* elemConnect, int npe );
    Epetra_CrsMatrix* getEpetraCrsMatrix(){ return globalMatrix; }
    Epetra_CrsMatrix* getAssemblyEpetraCrsMatrix() const { return assemblyMatrix; }
    void replaceGlobalValue(int node1_plid, int node2_plid, int dof_id, Real val);
    void getDiagonal(DistributedVector &diag);
    void replaceDiagonal(DistributedVector &diag);
    void FillComplete();
    void Print(std::ostream& fout){ globalMatrix->Print(fout); }

    void PutScalar(Real val){ globalMatrix->PutScalar(val),
                              assemblyMatrix->PutScalar(val); }


  private:
    DistributedCrsMatrix(){}
    Epetra_CrsMatrix   *globalMatrix;
    Epetra_CrsMatrix   *assemblyMatrix;

    Real* Values;
    int*  Indices;
};




/******************************************************************************/
class DistributedVector : public DistributedEntity
/******************************************************************************/
{
  public:

    friend class DistributedCrsMatrix;

    DistributedVector(SystemContainer *sys);
    DistributedVector(SystemContainer *sys, vector<VarIndex> vars);
    DistributedVector(SystemContainer *sys, VarIndex var);
    ~DistributedVector();
    void Assemble( Real value, int nodeId, int dofId);
    void LocalAssemble( Real value, int nodeId, int dofId);
    void Assemble( Intrepid::FieldContainer<double>& localMatrix, int* elemConnect, int npe );
    void DisAssemble();
    Epetra_Vector* getEpetraVector(){ return globalVector; }
    Epetra_Vector* getAssemblyEpetraVector() const { return assemblyVector; }
    void Import();
    void Export();
    void LocalExport();

    void PutScalar(Real val){ globalVector->PutScalar(val),
                              assemblyVector->PutScalar(val); }

    void ExtractView(Real** data){ assemblyVector->ExtractView(data); }

    int MyLength() const { return assemblyVector->MyLength(); }

    const vector<VarIndex>& getDataIndices() const { return plotIndices; }
 
  private:
    DistributedVector();
    Epetra_Vector   *globalVector;
    Epetra_Vector   *assemblyVector;

    vector<VarIndex> plotIndices;
};


#endif

