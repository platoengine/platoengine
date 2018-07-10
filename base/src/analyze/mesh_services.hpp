#ifndef MESH_SERVICES
#define MESH_SERVICES

#include "Intrepid_Basis.hpp"
#include "Intrepid_FieldContainer.hpp"
#include "Shards_CellTopology.hpp"
#include "matrix_container.hpp"

class DataMesh;
namespace Plato { class PenaltyModel; }

/******************************************************************************/
class MeshServices
/******************************************************************************/
{
  public:
    MeshServices(DataMesh* dataMesh) : myDataMesh(dataMesh){}

    double getTotalVolume();
    void getCurrentVolume(const DistributedVector& topo,
                          double& volume, DistributedVector& volumeGradient,
                          Plato::PenaltyModel* pModel=nullptr);

    void getRoughness(const DistributedVector& topo,
                      double& roughness, DistributedVector& roughnessGradient,
                      Plato::PenaltyModel* pModel=nullptr);
    void updateLowerBoundsForFixedBlocks(double *toData, const std::vector<int> &aFixedBlocks, const double &aFixedBlockValue);
    void updateLowerBoundsForFixedSidesets(double *toData, const std::vector<int> &aFixedSidesets, const double &aFixedSidesetsValue);
    void updateLowerBoundsForFixedNodesets(double *toData, const std::vector<int> &aFixedNodesets, const double &aFixedNodesetsValue);
    void updateUpperBoundsForFixedBlocks(double *toData, const std::vector<int> &mFixedBlocks, const double &aFixedBlockValue);
    void updateUpperBoundsForFixedSidesets(double *toData, const std::vector<int> &aFixedSidesets, const double &aFixedSidesetsValue);
    void updateUpperBoundsForFixedNodesets(double *toData, const std::vector<int> &aFixedNodesets, const double &aFixedNodesetsValue);

  private:
    DataMesh* myDataMesh;
    //shards::CellTopology **blockTopology;
    //Intrepid::Basis<double, Intrepid::FieldContainer<double> > **blockBasis;
};
/******************************************************************************/
#endif
