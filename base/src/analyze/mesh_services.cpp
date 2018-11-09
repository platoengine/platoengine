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

#include "mesh_services.hpp"
#include "data_mesh.hpp"
#include "data_container.hpp"
#include "topological_element.hpp"
#include "communicator.hpp"
#include "Plato_PenaltyModel.hpp"

#include <stdlib.h>
#include <iostream>

/******************************************************************************/
double MeshServices::getTotalVolume()
/******************************************************************************/
{
    double totalVolume = 0.0;

    using Intrepid::FieldContainer;
    using Intrepid::CellTools;

    DataMesh& myMesh = *myDataMesh;
    int nblocks = myMesh.getNumElemBlks();
    for(int ib=0; ib<nblocks; ib++){
      Topological::Element& elblock = *(myMesh.getElemBlk(ib));

      // not all blocks will be present on all processors
      if( elblock.getNumElem() == 0 ) continue;

      shards::CellTopology& topo = elblock.getTopology();
      int numNodesPerElem = elblock.getNnpe();
      int spaceDim = elblock.getDim();
      int numCubPoints = elblock.getNumIntPoints();

      FieldContainer<double>& cubPoints = elblock.getCubaturePoints();
      FieldContainer<double>& cubWeights = elblock.getCubatureWeights();
      auto uniformCubature = elblock.cubatureIsUniform();

      // Evaluate basis values and gradients at cubature points
      int numFieldsG = elblock.getBasis().getCardinality();
      FieldContainer<double> Gvals(numFieldsG, numCubPoints); 
      elblock.getBasis().getValues(Gvals, cubPoints, Intrepid::OPERATOR_VALUE);

      // Settings and data structures for mass and stiffness matrices
      typedef CellTools<double>  CellTools;
      typedef Intrepid::FunctionSpaceTools fst;
      int numCells = 1; 

      // Container for nodes
      FieldContainer<double> Nodes(numCells, numNodesPerElem, spaceDim);
      // Containers for Jacobian
      FieldContainer<double> Jacobian(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobDet(numCells, numCubPoints);
      FieldContainer<double> weightedMeasure(numCells, numCubPoints);

      int numElemsThisBlock = elblock.getNumElem();

      // Physical cell coordinates
      std::vector<Real*> coords(spaceDim);
      coords[0] = myMesh.getX();
      if(spaceDim > 1) coords[1] = myMesh.getY();
      if(spaceDim > 2) coords[2] = myMesh.getZ();

      FieldContainer<double> cWeights(cubWeights);

      // *** Element loop ***
      for (int iel=0; iel<numElemsThisBlock; iel++) {

        int* elemConnect = elblock.Connect(iel);

        for (int inode=0; inode<numNodesPerElem; inode++) {
          for (int idim=0; idim<spaceDim; idim++) {
            Nodes(0,inode,idim) = coords[idim][elemConnect[inode]];
          }
        }

        // Compute cell Jacobians, their inverses and their determinants
        CellTools::setJacobian(Jacobian, cubPoints, Nodes, topo);
        CellTools::setJacobianDet(JacobDet, Jacobian );

        if (!uniformCubature) {
          elblock.getCubatureWeights(cWeights, Nodes);
          weightedMeasure = cWeights;
        } else {
          fst::computeCellMeasure<double>(weightedMeasure, JacobDet, cWeights);
        }

        for(int cubPoint=0; cubPoint<numCubPoints; cubPoint++){
          totalVolume += weightedMeasure(0, cubPoint);
        }
      } // *** end element loop ***
    } // *** end block loop ***

    totalVolume = WorldComm.globalSum(totalVolume);

    return totalVolume;
}

/******************************************************************************/
void
MeshServices::getCurrentVolume(
  const DistributedVector& topologyField,
  double& totalVolume, 
  DistributedVector& gradientVector,
  Plato::PenaltyModel* penaltyModel)
/******************************************************************************/
{
    DataContainer& dc = *(myDataMesh->getDataContainer());
    VarIndex topoIndex = topologyField.getDataIndices()[0];
    Real* topoField; dc.getVariable(topoIndex, topoField);

    totalVolume = 0.0;
    gradientVector.PutScalar( 0.0 );

    using Intrepid::FieldContainer;
    using Intrepid::CellTools;

    DataMesh& myMesh = *myDataMesh;
    int nblocks = myMesh.getNumElemBlks();
    for(int ib=0; ib<nblocks; ib++){
      Topological::Element& elblock = *(myMesh.getElemBlk(ib));

      // not all blocks will be present on all processors
      if( elblock.getNumElem() == 0 ) continue;

      shards::CellTopology& topo = elblock.getTopology();
      int numNodesPerElem = elblock.getNnpe();
      int spaceDim = elblock.getDim();
      int numCubPoints = elblock.getNumIntPoints();

      FieldContainer<double>& cubPoints = elblock.getCubaturePoints();
      FieldContainer<double>& cubWeights = elblock.getCubatureWeights();
      auto uniformCubature = elblock.cubatureIsUniform();

      // Evaluate basis values and gradients at cubature points
      int numFieldsG = elblock.getBasis().getCardinality();
      FieldContainer<double> Gvals(numFieldsG, numCubPoints); 
      elblock.getBasis().getValues(Gvals, cubPoints, Intrepid::OPERATOR_VALUE);

      // Settings and data structures for mass and stiffness matrices
      typedef CellTools<double>  CellTools;
      typedef Intrepid::FunctionSpaceTools fst;
      int numCells = 1; 

      // Container for nodes
      FieldContainer<double> Nodes(numCells, numNodesPerElem, spaceDim);
      // Containers for Jacobian
      FieldContainer<double> Jacobian(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobDet(numCells, numCubPoints);
      FieldContainer<double> weightedMeasure(numCells, numCubPoints);

      FieldContainer<double> localGradient(numNodesPerElem, /*numTopos=*/ 1);

      int numElemsThisBlock = elblock.getNumElem();

      std::vector<Real*> coords(spaceDim);
      coords[0] = myMesh.getX();
      if(spaceDim > 1) coords[1] = myMesh.getY();
      if(spaceDim > 2) coords[2] = myMesh.getZ();

      FieldContainer<double> cWeights(cubWeights);

      // *** Element loop ***
      for (int iel=0; iel<numElemsThisBlock; iel++) {

        localGradient.initialize(0);

        // Physical cell coordinates
        int* elemConnect = elblock.Connect(iel);
        for (int inode=0; inode<numNodesPerElem; inode++) {
          for (int idim=0; idim<spaceDim; idim++) {
            Nodes(0,inode,idim) = coords[idim][elemConnect[inode]];
          }
        }

        // Compute cell Jacobians, their inverses and their determinants
        CellTools::setJacobian(Jacobian, cubPoints, Nodes, topo);
        CellTools::setJacobianDet(JacobDet, Jacobian );

        if (!uniformCubature) {
          elblock.getCubatureWeights(cWeights, Nodes);
          weightedMeasure = cWeights;
        } else {
          fst::computeCellMeasure<double>(weightedMeasure, JacobDet, cWeights);
        }

        for(int iQP=0; iQP<numCubPoints; iQP++){
          double topoVal=0.0;
          for(int iNode=0; iNode<numNodesPerElem; iNode++){
            topoVal += Gvals(iNode,iQP)*topoField[elemConnect[iNode]];
          }
          if(penaltyModel) topoVal = penaltyModel->eval(topoVal);

          totalVolume += topoVal*weightedMeasure(0, iQP);

          double topoGrad = 1.0;
          if(penaltyModel) topoGrad = penaltyModel->grad(topoVal);
          for(int iNode=0; iNode<numNodesPerElem; iNode++){
            localGradient(iNode, 0) += topoGrad*Gvals(iNode,iQP)*weightedMeasure(0,iQP);
          }
        }

        gradientVector.Assemble( localGradient, elemConnect, numNodesPerElem );
      } // *** end element loop ***
    } // *** end block loop ***

    totalVolume = WorldComm.globalSum(totalVolume);
    gradientVector.Export();
    gradientVector.DisAssemble();

    return;
}

/******************************************************************************/
void
MeshServices::updateUpperBoundsForFixedNodesets(double *aUpperBoundVector, const std::vector<int> &aFixedNodesets,
                                                const double &aFixedNodesetValue)
/******************************************************************************/
{
    DataMesh& tMyMesh = *myDataMesh;
    int tNumNodesets = tMyMesh.getNumNodeSets();
    for(int ib=0; ib<tNumNodesets; ib++)
    {
        DMNodeSet *nsi = tMyMesh.getNodeSet(ib);
        if(std::find(aFixedNodesets.begin(), aFixedNodesets.end(), nsi->id) == aFixedNodesets.end())
            continue;
        int num_nodes = nsi->numNodes;
        if(num_nodes)
        {
            int* nodes;
            tMyMesh.getDataContainer()->getVariable(nsi->NODE_LIST, nodes);
            for (int inode=0; inode<num_nodes; inode++)
            {
                aUpperBoundVector[nodes[inode]] = aFixedNodesetValue;
            }
        }
    } // *** end block loop ***
}

/******************************************************************************/
void
MeshServices::updateLowerBoundsForFixedNodesets(double *aLowerBoundVector, const std::vector<int> &aFixedNodesets,
                                                const double &aFixedNodesetValue)
/******************************************************************************/
{
    DataMesh& tMyMesh = *myDataMesh;
    int tNumNodesets = tMyMesh.getNumNodeSets();
    for(int ib=0; ib<tNumNodesets; ib++)
    {
        DMNodeSet *nsi = tMyMesh.getNodeSet(ib);
        if(std::find(aFixedNodesets.begin(), aFixedNodesets.end(), nsi->id) == aFixedNodesets.end())
            continue;
        int num_nodes = nsi->numNodes;
        if(num_nodes)
        {
            int* nodes;
            tMyMesh.getDataContainer()->getVariable(nsi->NODE_LIST, nodes);
            for (int inode=0; inode<num_nodes; inode++)
            {
                aLowerBoundVector[nodes[inode]] = aFixedNodesetValue;
            }
        }
    } // *** end block loop ***
}

/******************************************************************************/
void
MeshServices::updateLowerBoundsForFixedSidesets(double *aLowerBoundVector, const std::vector<int> &aFixedSidesets,
                                                const double &aFixedSidesetValue)
/******************************************************************************/
{
    DataMesh& tMyMesh = *myDataMesh;
    int tNumSidesets = tMyMesh.getNumSideSets();
    for(int ib=0; ib<tNumSidesets; ib++)
    {
        DMSideSet *ssi = tMyMesh.getSideSet(ib);
        if(std::find(aFixedSidesets.begin(), aFixedSidesets.end(), ssi->id) == aFixedSidesets.end())
            continue;
        int num_nodes = ssi->numNodes;
        if(num_nodes)
        {
            int* nodes;
            tMyMesh.getDataContainer()->getVariable(ssi->FACE_NODE_LIST, nodes);
            for (int inode=0; inode<num_nodes; inode++)
            {
                aLowerBoundVector[nodes[inode]] = aFixedSidesetValue;
            }
        }
    } // *** end block loop ***
}

/******************************************************************************/
void
MeshServices::updateUpperBoundsForFixedSidesets(double *aUpperBoundVector, const std::vector<int> &aFixedSidesets,
                                                const double &aFixedSidesetValue)
/******************************************************************************/
{
    DataMesh& tMyMesh = *myDataMesh;
    int tNumSidesets = tMyMesh.getNumSideSets();
    for(int ib=0; ib<tNumSidesets; ib++)
    {
        DMSideSet *ssi = tMyMesh.getSideSet(ib);
        if(std::find(aFixedSidesets.begin(), aFixedSidesets.end(), ssi->id) == aFixedSidesets.end())
            continue;
        int num_nodes = ssi->numNodes;
        if(num_nodes)
        {
            int* nodes;
            tMyMesh.getDataContainer()->getVariable(ssi->FACE_NODE_LIST, nodes);
            for (int inode=0; inode<num_nodes; inode++)
            {
                aUpperBoundVector[nodes[inode]] = aFixedSidesetValue;
            }
        }
    } // *** end block loop ***
}

/******************************************************************************/
void
MeshServices::updateLowerBoundsForFixedBlocks(double *aLowerBoundVector,
                                              const std::vector<int> &aFixedBlocks,
                                              const double& aOptimizationBlockInternalValue,
                                              const double& aFixedBlockBoundaryValue,
                                              const double& aFixedBlockInternalValue,
                                              DistributedVector& aDistributed)
/******************************************************************************/
{
    DataMesh& tMyMesh = *myDataMesh;
    const int tNumNodes = tMyMesh.getNumNodes();

    std::vector<int> tNodeTypeEnum(tNumNodes, 0);
    // 0 not yet set
    // 1 purely optimization
    // 2 between optimization and fixed
    // 3 purely fixed

    // for each block
    const int tNumBlocks = tMyMesh.getNumElemBlks();
    for(int ib = 0; ib < tNumBlocks; ib++)
    {
        Topological::Element& tCurBlock = *(tMyMesh.getElemBlk(ib));
        int tCurBlockId = tCurBlock.getBlockId();

        // not all blocks will be present on all processors
        int tNumElemsThisBlock = tCurBlock.getNumElem();
        if(tNumElemsThisBlock == 0)
        {
            continue;
        }

        // determine if this block is within list
        const bool tIsThisBlockFixed = (std::find(aFixedBlocks.begin(), aFixedBlocks.end(), tCurBlockId) != aFixedBlocks.end());

        // for each element
        for(int iel = 0; iel < tNumElemsThisBlock; iel++)
        {
            // for each node in this element
            int* tElemConnect = tCurBlock.Connect(iel);
            int tNumNodesPerElem = tCurBlock.getNnpe();
            for(int inode = 0; inode < tNumNodesPerElem; inode++)
            {
                const int tNodeId = tElemConnect[inode];
                switch(tNodeTypeEnum[tNodeId])
                {
                    case 0:
                    {
                        if(tIsThisBlockFixed)
                        {
                            // before: not yet set, after: purely fixed
                            tNodeTypeEnum[tNodeId] = 3;
                        }
                        else
                        {
                            // before: not yet set, after: purely optimization
                            tNodeTypeEnum[tNodeId] = 1;
                        }
                        break;
                    }
                    case 1:
                    {
                        if(tIsThisBlockFixed)
                        {
                            // before: purely optimization, after: between
                            tNodeTypeEnum[tNodeId] = 2;
                        }
                        break;
                    }
                    case 2:
                    {
                        break;
                    }
                    case 3:
                    {
                        if(!tIsThisBlockFixed)
                        {
                            // before: purely fixed, after: between
                            tNodeTypeEnum[tNodeId] = 2;
                        }
                        break;
                    }
                    default:
                    {
                        std::cout << "Plato Mesh Services Fatal Error: Unmatched node enumeration value." << std::endl
                                  << "Possible memory corruption" << std::endl;
                        std::abort();
                        break;
                    }
                }
            }
        }
    }

    // make parallel optimize
    aDistributed.PutScalar(0.);
    // for each node
    for(int inode = 0; inode < tNumNodes; inode++)
    {
        const bool tIsPurelyOptimize = (tNodeTypeEnum[inode] == 1);
        const bool tIsBetween = (tNodeTypeEnum[inode] == 2);
        if(tIsPurelyOptimize || tIsBetween)
        {
            aDistributed.Assemble(1., inode, 0);
        }
    }
    aDistributed.Export();
    aDistributed.DisAssemble();
    double* tParallelIsOptimize = NULL;
    aDistributed.ExtractView(&tParallelIsOptimize);
    // for each node
    for(int inode = 0; inode < tNumNodes; inode++)
    {
        const bool tIsAtleastPartialOptimize = (0 < tParallelIsOptimize[inode]);
        if(!tIsAtleastPartialOptimize)
        {
            continue;
        }

        switch(tNodeTypeEnum[inode])
        {
            case 0:
            {
                // before: not yet set, after: purely optimization
                tNodeTypeEnum[inode] = 1;
                break;
            }
            case 1:
            {
                break;
            }
            case 2:
            {
                break;
            }
            case 3:
            {
                // before: purely fixed, after: between
                tNodeTypeEnum[inode] = 2;
                break;
            }
            default:
            {
                std::cout << "Plato Mesh Services Fatal Error: Unmatched node enumeration value." << std::endl
                          << "Possible memory corruption" << std::endl;
                std::abort();
                break;
            }
        }
    }

    // make parallel fixed
    aDistributed.PutScalar(0.);
    // for each node
    for(int inode = 0; inode < tNumNodes; inode++)
    {
        const bool tIsBetween = (tNodeTypeEnum[inode] == 2);
        const bool tIsPurelyFixed = (tNodeTypeEnum[inode] == 3);
        if(tIsPurelyFixed || tIsBetween)
        {
            aDistributed.Assemble(1., inode, 0);
        }
    }
    aDistributed.Export();
    aDistributed.DisAssemble();
    double* tParallelIsFixed = NULL;
    aDistributed.ExtractView(&tParallelIsFixed);
    // for each node
    for(int inode = 0; inode < tNumNodes; inode++)
    {
        const bool tIsAtleastPartialFixed = (0 < tParallelIsFixed[inode]);
        if(!tIsAtleastPartialFixed)
        {
            continue;
        }

        switch(tNodeTypeEnum[inode])
        {
            case 0:
            {
                // before: not yet set, after: purely fixed
                tNodeTypeEnum[inode] = 3;
                break;
            }
            case 1:
            {
                // before: purely optimize, after: between
                tNodeTypeEnum[inode] = 2;
                break;
            }
            case 2:
            {
                break;
            }
            case 3:
            {
                break;
            }
            default:
            {
                std::cout << "Plato Mesh Services Fatal Error: Unmatched node enumeration value." << std::endl
                          << "Possible memory corruption" << std::endl;
                std::abort();
                break;
            }
        }
    }

    // for each node
    for(int inode = 0; inode < tNumNodes; inode++)
    {
        switch(tNodeTypeEnum[inode])
        {
            case 1:
            {
                // 1 purely optimization
                aLowerBoundVector[inode] = aOptimizationBlockInternalValue;
                break;
            }
            case 2:
            {
                // 2 between optimization and fixed
                aLowerBoundVector[inode] = aFixedBlockBoundaryValue;
                break;
            }
            case 3:
            {
                // 3 purely fixed
                aLowerBoundVector[inode] = aFixedBlockInternalValue;
                break;
            }
            case 0:
            {
                std::cout << "Plato Mesh Services Fatal Error: Not yet set node enumeration value." << std::endl
                          << "Possible node not connected to mesh elements." << std::endl;
                std::abort();
                break;
            }
            default:
            {
                std::cout << "Plato Mesh Services Fatal Error: Unmatched node enumeration value." << std::endl
                          << "Possible memory corruption" << std::endl;
                std::abort();
                break;
            }
        }
    }
}

/******************************************************************************/
void
MeshServices::updateUpperBoundsForFixedBlocks(double *aUpperBoundVector, const std::vector<int> &aFixedBlocks,
                                              const double &aFixedBlockValue)
/******************************************************************************/
{
    DataMesh& tMyMesh = *myDataMesh;
    int tNumBlocks = tMyMesh.getNumElemBlks();
    for(int ib=0; ib<tNumBlocks; ib++)
    {
        Topological::Element& tCurBlock = *(tMyMesh.getElemBlk(ib));
        int tCurBlockId = tCurBlock.getBlockId();

        if(std::find(aFixedBlocks.begin(), aFixedBlocks.end(), tCurBlockId) == aFixedBlocks.end())
            continue;

        // not all blocks will be present on all processors
        int tNumElemsThisBlock = tCurBlock.getNumElem();
        if( tNumElemsThisBlock == 0 )
            continue;

        int tNumNodesPerElem = tCurBlock.getNnpe();
        // *** Element loop ***
        for (int iel=0; iel<tNumElemsThisBlock; iel++)
        {
            int* tElemConnect = tCurBlock.Connect(iel);
            for (int inode=0; inode<tNumNodesPerElem; inode++)
            {
                aUpperBoundVector[tElemConnect[inode]] = aFixedBlockValue;
            }
        } // *** end element loop ***
    } // *** end block loop ***
}

/******************************************************************************/
void
MeshServices::getRoughness(
  const DistributedVector& topologyField,
  double& roughness, 
  DistributedVector& gradientVector,
  Plato::PenaltyModel* penaltyModel)
/******************************************************************************/
{
    DataContainer& dc = *(myDataMesh->getDataContainer());
    VarIndex topoIndex = topologyField.getDataIndices()[0];
    Real* topoField; dc.getVariable(topoIndex, topoField);

    roughness = 0.0;
    gradientVector.PutScalar( 0.0 );

    using Intrepid::FieldContainer;
    using Intrepid::CellTools;

    DataMesh& myMesh = *myDataMesh;
    int nblocks = myMesh.getNumElemBlks();
    for(int ib=0; ib<nblocks; ib++){
      Topological::Element& elblock = *(myMesh.getElemBlk(ib));

      // not all blocks will be present on all processors
      if( elblock.getNumElem() == 0 ) continue;

      shards::CellTopology& topo = elblock.getTopology();
      int numNodesPerElem = elblock.getNnpe();
      int spaceDim = elblock.getDim();
      int numCubPoints = elblock.getNumIntPoints();

      FieldContainer<double>& cubPoints = elblock.getCubaturePoints();
      FieldContainer<double>& cubWeights = elblock.getCubatureWeights();

      // Evaluate basis values and gradients at cubature points
      int numFieldsG = elblock.getBasis().getCardinality();
      FieldContainer<double> Grads(numFieldsG, numCubPoints, spaceDim); 
      elblock.getBasis().getValues(Grads, cubPoints, Intrepid::OPERATOR_GRAD);

      // Settings and data structures for mass and stiffness matrices
      typedef CellTools<double>  CellTools;
      typedef Intrepid::FunctionSpaceTools fst;
      int numCells = 1, iCell = 0;

      // Container for nodes
      FieldContainer<double> Nodes(numCells, numNodesPerElem, spaceDim);
      FieldContainer<double> Topos(numCells, numNodesPerElem);
      FieldContainer<double> divTopo(numCells, spaceDim);
      // Containers for Jacobian
      FieldContainer<double> Jacobian(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobInv(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobDet(numCells, numCubPoints);

      FieldContainer<double> weightedMeasure(numCells, numCubPoints);
      FieldContainer<double> GradsTransformed(numCells, numFieldsG, numCubPoints, spaceDim);

      FieldContainer<double> localGradient(numNodesPerElem, /*numTopos=*/ 1);

      int numElemsThisBlock = elblock.getNumElem();

      std::vector<Real*> coords(spaceDim);
      coords[0] = myMesh.getX();
      if(spaceDim > 1) coords[1] = myMesh.getY();
      if(spaceDim > 2) coords[2] = myMesh.getZ();

      // *** Element loop ***
      for (int iel=0; iel<numElemsThisBlock; iel++) {

        localGradient.initialize(0);

        // Physical cell coordinates
        int* elemConnect = elblock.Connect(iel);
        for (int inode=0; inode<numNodesPerElem; inode++) {
          for (int idim=0; idim<spaceDim; idim++) {
            Nodes(0,inode,idim) = coords[idim][elemConnect[inode]];
          }
          Topos(iCell,inode)   = topoField[elemConnect[inode]];
        }

        // Compute cell Jacobians, their inverses and their determinants
        CellTools::setJacobian(Jacobian, cubPoints, Nodes, topo);
        CellTools::setJacobianInv(JacobInv, Jacobian );
        CellTools::setJacobianDet(JacobDet, Jacobian );

        // transform to physical coordinates 
        fst::HGRADtransformGRAD<double>(GradsTransformed, JacobInv, Grads);

        // compute weighted measure
        fst::computeCellMeasure<double>(weightedMeasure, JacobDet, cubWeights);

        for(int iQP=0; iQP<numCubPoints; iQP++){
          divTopo.initialize(0.0);
          for(int iNode=0; iNode<numNodesPerElem; iNode++){
            for(int iDim=0; iDim<spaceDim; iDim++){
              divTopo(iCell,iDim) += GradsTransformed(iCell, iNode, iQP, iDim)*Topos(iCell, iNode);
            }
          }

          for(int iDim=0; iDim<spaceDim; iDim++){
            roughness += divTopo(iCell,iDim)*divTopo(iCell,iDim)*weightedMeasure(iCell, iQP)/2.0;
          }
          
          for(int iNode=0; iNode<numNodesPerElem; iNode++){
            for(int iDim=0; iDim<spaceDim; iDim++){
              localGradient(iNode, 0) 
                += GradsTransformed(iCell, iNode, iQP, iDim)*divTopo(iCell,iDim)*weightedMeasure(iCell, iQP);
            }
          }
        }

        gradientVector.Assemble( localGradient, elemConnect, numNodesPerElem );
      } // *** end element loop ***
    } // *** end block loop ***

    WorldComm.globalSum(roughness);
    gradientVector.Export();
    gradientVector.DisAssemble();

    return;
}
