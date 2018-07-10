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

