#include "lightmp.hpp"
#include "bc.hpp"
#include "Plato_PenaltyModel.hpp"

#define HAVE_CONFIG_H
#include "matrix_container.hpp"
#include "data_mesh.hpp"
#include "data_container.hpp"
#include "communicator.hpp"


// TODO: unkloodge the natural boundary condition implementation!


/******************************************************************************/
void SolidStatics::Parse( pugi::xml_node& input, LightMP& ren )
/******************************************************************************/
{
  pugi::xml_node config = input.child( "solid_statics" );

  solverspec = config.child( "solver" );

  pugi::xml_node bcspecs = config.child( "boundary_conditions" );
  if( bcspecs ){
    // parse displacements
    for(pugi::xml_node cur_disp = bcspecs.child("displacement"); cur_disp;
            cur_disp = cur_disp.next_sibling("displacement")){
      ConstantValueBC<Real>* newBC = new ConstantValueBC<Real>(cur_disp);
      Real value = Plato::Parse::getDouble( cur_disp, "value" );
      newBC->setValue(value);
      newBC->findNodeSet( ren.getMesh()->getNodeSets() );
      essentialBCs.push_back(newBC);
    }
    // parse traction
    for(pugi::xml_node cur_trac = bcspecs.child("traction"); cur_trac;
            cur_trac = cur_trac.next_sibling("traction")){
      ConstantValueBC<Real>* newBC = new ConstantValueBC<Real>(cur_trac);
      Real value = Plato::Parse::getDouble( cur_trac, "value" );
      newBC->setValue(value);
      newBC->findNodeSet( ren.getMesh()->getNodeSets() );
      naturalBCs.push_back(newBC);
    }

  }
}

/******************************************************************************/
SolidStatics::SolidStatics(SystemContainer& sys, LightMP& ren)
/******************************************************************************/
{
   myMaterialContainer = ren.getMaterialContainer();
   myDataContainer = ren.getDataContainer();
   myDataMesh = ren.getMesh();

   // parse
   pugi::xml_node input = ren.getInput();
   pugi::xml_node physicsSpec = input.child("physics");

   Parse( physicsSpec, ren );

   nHourglassModes = 0;
   mySystem = &sys;

   // Variables are stored in the data container and can be accessed by name
   // (slow) or by index (faster).  The SolidStatics object doesn't own any
   // data, but requests STRESS and STRAIN_INCREMENT from the data container.
   // If STRESS and STRAIN_INCREMENT have been registered (i.e., by a material
   // model or material models) then setup proceeds and assumes that a useable
   // tangent will be provided.

   DataContainer& dc = *myDataContainer;
   
   bool plottable = true;
   int numStates = 2;
   STRESS = dc.registerVariable(SymTensorType, "cauchy stress", 
                                MATPNT, plottable, numStates);

   STRAIN_INCREMENT = dc.registerVariable(SymTensorType, "strain increment", 
                                          MATPNT, plottable);

   TOTAL_DX = dc.registerVariable( RealType, "total dx", NODE, plottable);
   TOTAL_DY = dc.registerVariable( RealType, "total dy", NODE, plottable);
   TOTAL_DZ = dc.registerVariable( RealType, "total dz", NODE, plottable);
   Real* total_dx; dc.getVariable( TOTAL_DX, total_dx );
   Real* total_dy; dc.getVariable( TOTAL_DY, total_dy );
   Real* total_dz; dc.getVariable( TOTAL_DZ, total_dz );
   int numNodes = myDataMesh->getNumNodes();
   for(int i=0; i<numNodes; i++){
     total_dx[i] = 0.0;
     total_dy[i] = 0.0;
     total_dz[i] = 0.0;
   }
    

   dc.getVariable( "dispx", DISPX );
   dc.getVariable( "dispy", DISPY );
   dc.getVariable( "dispz", DISPZ );

}

/******************************************************************************/
void SolidStatics::
computeInternalEnergy( DistributedVector* topology,  
                       Plato::PenaltyModel* penaltyModel,
                       double* ie, DistributedVector* iegradient)
/******************************************************************************/
{
    double& internalEnergy = ie[0];
    internalEnergy = 0.0;

    MaterialContainer& mc = *myMaterialContainer;
    DataContainer& dc = *myDataContainer;
    DataMesh& myMesh = *myDataMesh;

    VarIndex topoIndex = topology->getDataIndices()[0];
    Real* topoField; dc.getVariable(topoIndex, topoField);

    SymTensor* stress;     dc.getVariable( STRESS, stress, UPDATED );
    SymTensor* strain_inc; dc.getVariable( STRAIN_INCREMENT, strain_inc );

    Real* X = myMesh.getX();
    Real* Y = myMesh.getY();
    Real* Z = myMesh.getZ();

    // tvmap:  map from tensor indices to voigt index
    const int tvmap[3][3] = {{0,5,4},{5,1,3},{4,3,2}};
    //const int nVoigt = 6;


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
      elblock.getBasis().getValues(Grads, cubPoints, OPERATOR_GRAD);

      FieldContainer<double> Gvals(numFieldsG, numCubPoints);
      elblock.getBasis().getValues(Gvals, cubPoints, OPERATOR_VALUE);

      // Settings and data structures for mass and stiffness matrices
      typedef CellTools<double>  CellTools;
      typedef FunctionSpaceTools fst;
      int numCells = 1; 

      // Container for nodes
      FieldContainer<double> Nodes(numCells, numNodesPerElem, spaceDim);
      // Containers for Jacobian
      FieldContainer<double> Jacobian(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobInv(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobDet(numCells, numCubPoints);

      // Containers for element HGRAD stiffness matrix
      FieldContainer<double> weightedMeasure(numCells, numCubPoints);
      FieldContainer<double> GradsTransformed(numCells, numFieldsG, numCubPoints, spaceDim);

      FieldContainer<double> localGradient(numFieldsG, 1);

      int numElemsThisBlock = elblock.getNumElem();

      // *** Element loop ***
      for (int iel=0; iel<numElemsThisBlock; iel++) {

        // Physical cell coordinates and displacements
        int* elemConnect = elblock.Connect(iel);
        for (int inode=0; inode<numNodesPerElem; inode++) {
          Nodes(0,inode,0) = X[elemConnect[inode]];
          Nodes(0,inode,1) = Y[elemConnect[inode]];
          Nodes(0,inode,2) = Z[elemConnect[inode]];
        }

        // Compute cell Jacobians, their inverses and their determinants
        CellTools::setJacobian(Jacobian, cubPoints, Nodes, topo);
        CellTools::setJacobianInv(JacobInv, Jacobian );
        CellTools::setJacobianDet(JacobDet, Jacobian );

        // transform to physical coordinates 
        fst::HGRADtransformGRAD<double>(GradsTransformed, JacobInv, Grads);
      
        // compute weighted measure
        fst::computeCellMeasure<double>(weightedMeasure, JacobDet, cubWeights);

        // compute stress divergence
        int icell = 0;
        localGradient.initialize(0);
        for(int ipoint=0; ipoint<numCubPoints; ipoint++){

          double topoVal=0.0;
          for(int iNode=0; iNode<numFieldsG; iNode++){
            topoVal += Gvals(iNode,ipoint)*topoField[elemConnect[iNode]];
          }
          if(penaltyModel) topoVal = penaltyModel->eval(topoVal);

          int dataIndex =  mc.getDataIndex(ib, iel, ipoint);

          Real *elStress;     stress[dataIndex].extractData(elStress);
          Real *elStrain_inc; strain_inc[dataIndex].extractData(elStrain_inc);

          double localIE=0.0;
          for(int idim=0; idim<spaceDim; idim++)
            for(int jdim=0; jdim<spaceDim; jdim++)
              localIE+=elStress[tvmap[idim][jdim]]*elStrain_inc[tvmap[idim][jdim]];
          localIE*=weightedMeasure(icell,ipoint);

          internalEnergy += topoVal*localIE;

          double topoGrad = 1.0;
          if(penaltyModel) topoGrad = penaltyModel->grad(topoVal);
          for(int iNode=0; iNode<numFieldsG; iNode++){
            // negative makes this a total derivative
            localGradient(iNode, 0) -= topoGrad*Gvals(iNode,ipoint)*localIE;
          }
        }
        iegradient->Assemble( localGradient, elemConnect, numFieldsG );
      } // *** end element loop ***
    } // *** end block loop ***

    internalEnergy = WorldComm.globalSum(internalEnergy);

    iegradient->Export();
    iegradient->DisAssemble();

    return;
}

/******************************************************************************/
void SolidStatics::
buildStiffnessMatrix( DistributedCrsMatrix&    stiffMatrix,
                      const DistributedVector& topology, 
                      Plato::PenaltyModel*     penaltyModel )
/******************************************************************************/
{
    stiffMatrix.PutScalar(0.0);

    MaterialContainer& mc = *myMaterialContainer;
    DataContainer& dc = *(myDataContainer);
    DataMesh& myMesh = *myDataMesh;

    VarIndex topoIndex = topology.getDataIndices()[0];
    Real* topoField; dc.getVariable(topoIndex, topoField);

    const int dofsPerNode = mySystem->getDofsPerNode();
    const int voigtSize = 6;

    int nblocks = myMesh.getNumElemBlks();
    for(int ib=0; ib<nblocks; ib++){
      Topological::Element& elblock = *(myMesh.getElemBlk(ib));

      p0cout << "block " << ib+1 << endl;
  
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
      elblock.getBasis().getValues(Grads, cubPoints, OPERATOR_GRAD);

      FieldContainer<double> Gvals(numFieldsG, numCubPoints);
      elblock.getBasis().getValues(Gvals, cubPoints, OPERATOR_VALUE);


      // ******** LOOP OVER ELEMENTS TO CREATE LOCAL STIFFNESS MATRIX *************

      // Settings and data structures for mass and stiffness matrices
      typedef CellTools<double>  CellTools;
      typedef FunctionSpaceTools fst;
      int numCells = 1; 

      // Container for nodes
      FieldContainer<double> Nodes(numCells, numNodesPerElem, spaceDim);
      // Containers for Jacobian
      FieldContainer<double> Jacobian(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobInv(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobDet(numCells, numCubPoints);
      // Containers for element HGRAD stiffness matrix
      FieldContainer<double> weightedMeasure(numCells, numCubPoints);
      FieldContainer<double> GradsTransformed(numCells, numFieldsG, numCubPoints, spaceDim);
      FieldContainer<double> GradsTransformedWeighted(numCells, numFieldsG, numCubPoints, spaceDim);
      // Container for cubature points in physical space

      FieldContainer<double> B(numCells, numFieldsG*dofsPerNode, numCubPoints, voigtSize); // 1 x 24 x 8 x 6  
      FieldContainer<double> CB(numCells, numFieldsG*dofsPerNode, numCubPoints, voigtSize); // 1 x 24 x 8 x 6
      FieldContainer<double> CBWeighted(numCells, numFieldsG*dofsPerNode, numCubPoints, voigtSize); // 1 x 24 x 8 x 6
      FieldContainer<double> BCB(numCells, numFieldsG*dofsPerNode, numCubPoints, numFieldsG*dofsPerNode); // 1 x 24 x 8 x 24
      FieldContainer<double> BCBWeighted(numCells, numFieldsG*dofsPerNode, numCubPoints, numFieldsG*dofsPerNode); // 1 x 24 x 8 x 24
      FieldContainer<double> localStiffMatrix(numCells, numFieldsG, dofsPerNode, numFieldsG, dofsPerNode); // 1 x 24 x 24     element stiffness matrix (dof x dof)

      int numElemsThisBlock = elblock.getNumElem();

      // *** Element loop ***
      int mileStone = numElemsThisBlock/10;
      int percentComplete = -10;
      for (int iel=0; iel<numElemsThisBlock; iel++) {

        if(mileStone > 0){
          if(iel%mileStone == 0) {
                percentComplete += 10;
                p0cout << " " << std::setw(5) << percentComplete << "% complete" << endl;
          }
        }

        // Physical cell coordinates
        int* elemConnect = elblock.Connect(iel);
        Real* X = myMesh.getX();
        Real* Y = myMesh.getY();
        Real* Z = myMesh.getZ();
        for (int inode=0; inode<numNodesPerElem; inode++) {
          Nodes(0,inode,0) = X[elemConnect[inode]];
          Nodes(0,inode,1) = Y[elemConnect[inode]];
          Nodes(0,inode,2) = Z[elemConnect[inode]];
        }

        // Compute cell Jacobians, their inverses and their determinants
        CellTools::setJacobian(Jacobian, cubPoints, Nodes, topo);
        CellTools::setJacobianInv(JacobInv, Jacobian );
        CellTools::setJacobianDet(JacobDet, Jacobian );

        // transform to physical coordinates 
        fst::HGRADtransformGRAD<double>(GradsTransformed, JacobInv, Grads);
      
        // compute weighted measure
        fst::computeCellMeasure<double>(weightedMeasure, JacobDet, cubWeights);

        for(int cubPoint=0; cubPoint<numCubPoints; cubPoint++){
          for(int shape=0; shape<numFieldsG; shape++){

            B(0, shape*3, cubPoint, 0) = GradsTransformed(0,shape, cubPoint, 0);
            B(0, shape*3, cubPoint, 1) = 0.0;
            B(0, shape*3, cubPoint, 2) = 0.0;
            B(0, shape*3, cubPoint, 3) = 0.0;
            B(0, shape*3, cubPoint, 4) = GradsTransformed(0,shape, cubPoint, 2);
            B(0, shape*3, cubPoint, 5) = GradsTransformed(0,shape, cubPoint, 1);

            B(0, shape*3+1, cubPoint, 0) = 0.0;
            B(0, shape*3+1, cubPoint, 1) = GradsTransformed(0,shape, cubPoint, 1);
            B(0, shape*3+1, cubPoint, 2) = 0.0;
            B(0, shape*3+1, cubPoint, 3) = GradsTransformed(0,shape, cubPoint, 2);
            B(0, shape*3+1, cubPoint, 4) = 0.0;
            B(0, shape*3+1, cubPoint, 5) = GradsTransformed(0,shape, cubPoint, 0);

            B(0, shape*3+2, cubPoint, 0) = 0.0;
            B(0, shape*3+2, cubPoint, 1) = 0.0;
            B(0, shape*3+2, cubPoint, 2) = GradsTransformed(0,shape, cubPoint, 2);
            B(0, shape*3+2, cubPoint, 3) = GradsTransformed(0,shape, cubPoint, 1);
            B(0, shape*3+2, cubPoint, 4) = GradsTransformed(0,shape, cubPoint, 0);
            B(0, shape*3+2, cubPoint, 5) = 0.0;

          }
        }

        FieldContainer<double>* C;

        // compute C:B
        CB.initialize(0.0);
        for(int cubPoint=0; cubPoint<numCubPoints; cubPoint++){
 
          mc.getCurrentTangent(ib, iel, cubPoint, C, STRESS, STRAIN_INCREMENT);
          FieldContainer<double>& c = *C;
              
          // determine if the cubature point is within the level-set function
          for(int i=0; i<voigtSize; i++){
            for(int j=0; j<numFieldsG*dofsPerNode; j++){
              for(int k=0; k<voigtSize; k++){
                CB(0, j, cubPoint, i) += c(i, k) * B(0, j, cubPoint, k);
              }
            }
          }

        }


        // compute B_transpose:C:B
        BCB.initialize(0.0);
        for(int cubPoint=0; cubPoint<numCubPoints; cubPoint++){ 
          for(int i=0; i<numFieldsG*dofsPerNode; i++){
            for(int j=0; j<numFieldsG*dofsPerNode; j++){
              for(int k=0; k<voigtSize; k++){
                BCB(0, j, cubPoint, i) += B(0, i, cubPoint, k) * CB(0, j, cubPoint, k);
              }
            }
          }
        }

        // multiply values with weighted measure
        fst::multiplyMeasure<double>(BCBWeighted, weightedMeasure, BCB);

        // integrate to compute element stiffness matrix
        localStiffMatrix.initialize(0.0);
        for(int cubPoint=0; cubPoint<numCubPoints; cubPoint++){

          double topoVal=0.0;
          for(int iNode=0; iNode<numFieldsG; iNode++){
            topoVal += Gvals(iNode,cubPoint)*topoField[elemConnect[iNode]];
          }
          if(penaltyModel) topoVal = penaltyModel->eval(topoVal);

          for (int iNode = 0; iNode < numFieldsG; iNode++){
            for (int jNode = 0; jNode < numFieldsG; jNode++){
              for (int iDof = 0; iDof < dofsPerNode; iDof++){
                for (int jDof = 0; jDof < dofsPerNode; jDof++){
                  localStiffMatrix(0, iNode, iDof, jNode, jDof) += topoVal*BCBWeighted(0, iNode*dofsPerNode+iDof, cubPoint, jNode*dofsPerNode+jDof);
                }
              }
            }
          }
        }

        if( nHourglassModes ){
          int ipoint = 0;  // assumes the first (and likely only) integration point
          double hgCoef = 1.0;
          hgCoef *= 1.0/8.0*JacobDet(0,ipoint);
          FieldContainer<double>& hg = *hourglassModes;
          for( int imode=0; imode<nHourglassModes; imode++) {
            FieldContainer<double> gamma(numFieldsG);
            
            // node location: Nodes(0,inode,idim)
            // gradient matrix: GradsTransformed(0, inode, ipoint, idim)
            double* projection = new double[3];
            for( int idim=0; idim<spaceDim; idim++){
              double& proj = projection[idim];
              proj = 0.0;
              for( int inode=0; inode<numFieldsG; inode++)
                proj += hg(imode, inode) * Nodes(0, inode, idim);
            }
            for( int inode=0; inode<numFieldsG; inode++){
              double gammaVal = hg(inode);
              for( int idim=0; idim<spaceDim; idim++){
                gammaVal -= projection[idim]*GradsTransformed(0, inode, ipoint, idim);
              }
              gamma(inode) = gammaVal;
            }

            // gamma = 1/8 ( mode - (mode x)Bx - (mode y)By - (mode z)Bz)
            // Kstabilize = gamma^T gamma 
            for (int iNode = 0; iNode < numFieldsG; iNode++){
              for (int jNode = 0; jNode < numFieldsG; jNode++){
                for (int iDof = 0; iDof < dofsPerNode; iDof++){
                  localStiffMatrix(0, iNode, iDof, jNode, iDof) += hgCoef*gamma(iNode)*gamma(jNode);
                }
              }
            }
          }
          // Klocal += Kstabilize
        }

        stiffMatrix.Assemble( localStiffMatrix, elemConnect, numNodesPerElem);

      } // *** end element loop ***
    } // *** end block loop ***

    stiffMatrix.FillComplete();

    return;
}
/******************************************************************************/
void SolidStatics::computeExternalForces( DistributedVector& forcingVector, 
                                          Real time )
/******************************************************************************/
{
    DataContainer& dc = *(myDataContainer);

    int nbcs = naturalBCs.size();
    for( int ibc=0; ibc<nbcs; ibc++ ){
      BoundaryCondition<Real>& bc = *(naturalBCs[ibc]);
      const DMNodeSet& ns = bc.getNodeSet();
      int numNodes_ns = ns.numNodes;
      if( numNodes_ns > 0 ){
        int *nodes; dc.getVariable(ns.NODE_LIST, nodes);
        for(int inode=0; inode<numNodes_ns; inode++){
          int idof = bc.getDofIndex();
          Real value = bc.Value( time );
          forcingVector.LocalAssemble( value, nodes[inode], idof );
        }
      }
    }
}
/******************************************************************************/
void SolidStatics::computeInternalForces( DistributedVector& forcingVector, 
                                          Real time )
/******************************************************************************/
{

    MaterialContainer& mc = *myMaterialContainer;
    DataContainer& dc = *myDataContainer;
    DataMesh& myMesh = *myDataMesh;

    SymTensor* stress; dc.getVariable( STRESS, stress, UPDATED );

    Real* X = myMesh.getX();
    Real* Y = myMesh.getY();
    Real* Z = myMesh.getZ();

    // tvmap:  map from tensor indices to voigt index
    const int tvmap[3][3] = {{0,5,4},{5,1,3},{4,3,2}};

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
      elblock.getBasis().getValues(Grads, cubPoints, OPERATOR_GRAD);


      // Settings and data structures for mass and stiffness matrices
      typedef CellTools<double>  CellTools;
      typedef FunctionSpaceTools fst;
      int numCells = 1; 

      // Container for nodes
      FieldContainer<double> Nodes(numCells, numNodesPerElem, spaceDim);
      // Containers for Jacobian
      FieldContainer<double> Jacobian(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobInv(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobDet(numCells, numCubPoints);

      // Containers for element HGRAD stiffness matrix
      FieldContainer<double> weightedMeasure(numCells, numCubPoints);
      FieldContainer<double> GradsTransformed(numCells, numFieldsG, numCubPoints, spaceDim);

      FieldContainer<double> localForce(numFieldsG, spaceDim);

      int numElemsThisBlock = elblock.getNumElem();

      // *** Element loop ***
      for (int iel=0; iel<numElemsThisBlock; iel++) {

        // Physical cell coordinates and displacements
        int* elemConnect = elblock.Connect(iel);
        for (int inode=0; inode<numNodesPerElem; inode++) {
          Nodes(0,inode,0) = X[elemConnect[inode]];
          Nodes(0,inode,1) = Y[elemConnect[inode]];
          Nodes(0,inode,2) = Z[elemConnect[inode]];
        }

        // Compute cell Jacobians, their inverses and their determinants
        CellTools::setJacobian(Jacobian, cubPoints, Nodes, topo);
        CellTools::setJacobianInv(JacobInv, Jacobian );
        CellTools::setJacobianDet(JacobDet, Jacobian );

        // transform to physical coordinates 
        fst::HGRADtransformGRAD<double>(GradsTransformed, JacobInv, Grads);
      
        // compute weighted measure
        fst::computeCellMeasure<double>(weightedMeasure, JacobDet, cubWeights);

        // compute stress divergence
        int icell = 0;
        localForce.initialize(0);
        for(int ipoint=0; ipoint<numCubPoints; ipoint++){

          int dataIndex =  mc.getDataIndex(ib, iel, ipoint);
          Real *elStress; stress[dataIndex].extractData(elStress);
          
          for(int inode=0; inode<numFieldsG; inode++){
            for(int idim=0; idim<spaceDim; idim++)
              for(int jdim=0; jdim<spaceDim; jdim++)
                localForce(inode, idim) 
                  -= GradsTransformed(icell, inode, ipoint, jdim) * elStress[tvmap[idim][jdim]] * weightedMeasure(icell, ipoint);
          }
        }
        forcingVector.Assemble( localForce, elemConnect, numFieldsG );
      } // *** end element loop ***
    } // *** end block loop ***

    forcingVector.Export();

    return;
}

/******************************************************************************/
void SolidStatics::applyConstraints( DistributedCrsMatrix& stiffMatrix, 
                                     DistributedVector& forcingVector, 
                                     Real time )
/******************************************************************************/
{
    //This function applies both the essential and periodic boundary conditions
    //This function also calls FillComplete on the stiffness matrix because that
    //...needs to be called after we are done editing the stiffness matrix and
    //...and the periodic boundary condition requires editing of off-diagonal elements

    const Real PENALTY_NUMBER = 1e16;

    DataContainer& dc = *(myDataContainer);

    DistributedVector diagonal(mySystem);
    stiffMatrix.getDiagonal(diagonal);

    //apply essential boundary conditions
    int nbcs = essentialBCs.size();
    for( int ibc=0; ibc<nbcs; ibc++ ){
      BoundaryCondition<Real>& bc = *(essentialBCs[ibc]);
      const DMNodeSet& ns = bc.getNodeSet();
      int numNodes_ns = ns.numNodes;
      if( numNodes_ns > 0 ){
        int *nodes; dc.getVariable(ns.NODE_LIST, nodes);

        for(int inode=0; inode<numNodes_ns; inode++){
          int idof = bc.getDofIndex();
          Real value = bc.Value( time ) * PENALTY_NUMBER;
          diagonal.LocalAssemble( PENALTY_NUMBER, nodes[inode], idof );
          forcingVector.LocalAssemble( value, nodes[inode], idof );
        }
      }
    }

    stiffMatrix.replaceDiagonal(diagonal);

    return;
}


/******************************************************************************/
void SolidStatics::lagrangianUpdate()
/******************************************************************************/
{
  Real* X = myDataMesh->getX();
  Real* Y = myDataMesh->getY();
  Real* Z = myDataMesh->getZ();

  Real* total_dx; myDataContainer->getVariable( TOTAL_DX, total_dx );
  Real* total_dy; myDataContainer->getVariable( TOTAL_DY, total_dy );
  Real* total_dz; myDataContainer->getVariable( TOTAL_DZ, total_dz );
  Real* dx; myDataContainer->getVariable( DISPX, dx );
  Real* dy; myDataContainer->getVariable( DISPY, dy );
  Real* dz; myDataContainer->getVariable( DISPZ, dz );
  int numNodes = myDataMesh->getNumNodes();
  for(int i=0; i<numNodes; i++){
    total_dx[i] += dx[i];
    total_dy[i] += dy[i];
    total_dz[i] += dz[i];
    X[i] += dx[i];
    Y[i] += dy[i];
    Z[i] += dz[i];
    dx[i] = 0.0;
    dy[i] = 0.0;
    dz[i] = 0.0;
  }
}

/******************************************************************************/
void SolidStatics::updateMaterialState( Real time )
/******************************************************************************/
{

    MaterialContainer& mc = *myMaterialContainer;
    DataContainer& dc = *myDataContainer;
    DataMesh& myMesh = *myDataMesh;

    SymTensor* strain_inc; dc.getVariable( STRAIN_INCREMENT, strain_inc );

    Real* X = myMesh.getX();
    Real* Y = myMesh.getY();
    Real* Z = myMesh.getZ();

    Real *dx, *dy, *dz;
    dc.getVariable( DISPX, dx );
    dc.getVariable( DISPY, dy );
    dc.getVariable( DISPZ, dz );

    const int nVoigt = 6;
    int voigtStride[nVoigt][2] = {{0,0},{1,1},{2,2},{1,2},{0,2},{0,1}};

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
      elblock.getBasis().getValues(Grads, cubPoints, OPERATOR_GRAD);

      // Settings and data structures for mass and stiffness matrices
      typedef CellTools<double>  CellTools;
      typedef FunctionSpaceTools fst;
      int numCells = 1; 

      // Container for nodes
      FieldContainer<double> Nodes(numCells, numNodesPerElem, spaceDim);
      FieldContainer<double> Disps(numCells, numNodesPerElem, spaceDim);
      // Containers for Jacobian
      FieldContainer<double> Jacobian(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobInv(numCells, numCubPoints, spaceDim, spaceDim);
      FieldContainer<double> JacobDet(numCells, numCubPoints);

      // Containers for element HGRAD stiffness matrix
      FieldContainer<double> weightedMeasure(numCells, numCubPoints);
      FieldContainer<double> GradsTransformed(numCells, numFieldsG, numCubPoints, spaceDim);

      FieldContainer<double> fc_strain_inc(numCells, numFieldsG, spaceDim, spaceDim);

      int numElemsThisBlock = elblock.getNumElem();

      // *** Element loop ***
      for (int iel=0; iel<numElemsThisBlock; iel++) {

        // Physical cell coordinates and displacements
        int* elemConnect = elblock.Connect(iel);
        for (int inode=0; inode<numNodesPerElem; inode++) {
          Nodes(0,inode,0) = X[elemConnect[inode]];
          Nodes(0,inode,1) = Y[elemConnect[inode]];
          Nodes(0,inode,2) = Z[elemConnect[inode]];
          Disps(0,inode,0) = dx[elemConnect[inode]];
          Disps(0,inode,1) = dy[elemConnect[inode]];
          Disps(0,inode,2) = dz[elemConnect[inode]];
        }

        // Compute cell Jacobians, their inverses and their determinants
        CellTools::setJacobian(Jacobian, cubPoints, Nodes, topo);
        CellTools::setJacobianInv(JacobInv, Jacobian );
        CellTools::setJacobianDet(JacobDet, Jacobian );

        // transform to physical coordinates 
        fst::HGRADtransformGRAD<double>(GradsTransformed, JacobInv, Grads);
      
        // compute weighted measure
        fst::computeCellMeasure<double>(weightedMeasure, JacobDet, cubWeights);

        // compute strain
        int icell = 0;
        fc_strain_inc.initialize(0);
        for(int ipoint=0; ipoint<numCubPoints; ipoint++){
          for(int inode=0; inode<numFieldsG; inode++){
            for( int ivgt=0; ivgt<nVoigt; ivgt++){
              int idim = voigtStride[ivgt][0], jdim = voigtStride[ivgt][1];
              fc_strain_inc(icell, ipoint, idim, jdim ) += 
                0.5*(GradsTransformed(icell, inode, ipoint, idim)*Disps(icell, inode, jdim)
                    +GradsTransformed(icell, inode, ipoint, jdim)*Disps(icell, inode, idim));
            }
          }
        }
         
        for(int ipoint=0; ipoint<numCubPoints; ipoint++){
 
          // update strain measure
          int dataIndex =  mc.getDataIndex(ib, iel, ipoint);
          Real *elStrain_inc; strain_inc[dataIndex].extractData(elStrain_inc);
          for( int ivgt=0; ivgt<nVoigt; ivgt++){
            int idim = voigtStride[ivgt][0];
            int jdim = voigtStride[ivgt][1];
            elStrain_inc[ivgt] = fc_strain_inc(icell, ipoint, idim, jdim );
          }

          // call material model to update state
          mc.updateMaterialState(ib, iel, ipoint);
        }
      } // *** end element loop ***
    } // *** end block loop ***

    return;
}


void SolidStatics::updateDisplacement( DistributedVector& x,
                                       DistributedVector& B,
                                       DistributedCrsMatrix& A, Real time )
{

  // create problem and solver
  Epetra_LinearProblem problem( A.getEpetraCrsMatrix(),
                                x.getEpetraVector(),
                                B.getEpetraVector());
  AztecOO solver(problem);

  // setup
  int niters; Real tolerance;
  setupSolver( solverspec, solver, niters, tolerance, &A );

  // solve
  solver.Iterate( niters, tolerance );

  // exchange boundary data 
  x.Import();
  x.DisAssemble();

}

