#ifndef MATERIAL_CONTAINER
#define MATERIAL_CONTAINER

// To add a native material model:
// 1.  create model (see any other model for an example)
// 2.  add the model name and call back to material_container.cpp

#include "types.hpp"
#include "Intrepid_FieldContainer.hpp"
#include "Plato_Parser.hpp"
#include "math_library.hpp"
#include <string>
using namespace std;

class Model;
class Material;
class MaterialModel;
class DataContainer;
class DataMesh;

enum IndexOrdering { COrdering=0, FortranOrdering };

typedef struct{ 
 bool operator() (MaterialModel* a, MaterialModel* b);
} ModelComparator;


typedef MaterialModel* (*NewModelFP)(pugi::xml_node& node);

/******************************************************************************/
/******************************************************************************/
class MaterialModel {
public:
  MaterialModel(pugi::xml_node& node){}
  virtual ~MaterialModel(){}
  virtual bool SetUp(DataContainer*, Tensor& basis){return false;}
  virtual bool Initialize(int dataIndex, DataContainer*) = 0;

  virtual bool Tangent(int dataIndex, DataContainer* mc, 
                       Intrepid::FieldContainer<double>*& C){return false;}

  virtual bool UpdateMaterialState( int dataIndex,
                                   DataContainer* dc) = 0;

  VarIndex getDependentVariable() { return Findex; }
  VarIndex getIndependentVariable() { return Xindex; }

  protected:
    double *param;
    VarIndex Findex, Xindex;
};
/******************************************************************************/
/******************************************************************************/


/******************************************************************************/
/******************************************************************************/
/*! This class owns all the available models.  This added layer between the
   caller and the materials is used to determine the relavent material given
   the block, element, and material point.  Consequently, this class has 
   access to the material topology information.
*/
class MaterialContainer {
  public:
    MaterialContainer(){}
    MaterialContainer(pugi::xml_node& config);

    virtual ~MaterialContainer(){}

    virtual void setMaterialTopology(DataMesh* mesh, 
                                     DataContainer* dc,
                                     pugi::xml_node& node);

    virtual void initializeMaterialTopology()=0;
    virtual void initializeDataMap();


    virtual void getCurrentTangent(int iblock, int ielement, int ipoint, 
                                   Intrepid::FieldContainer<double>*& C,
                                   VarIndex f, VarIndex x);

    virtual void updateMaterialState(int iblock, int ielement, int ipoint);
    virtual void updateMaterialState(int iblock, int ielement, int ipoint, VarIndex var);

    virtual void SetUp(DataMesh* mesh, DataContainer* dc, pugi::xml_node& node);
    virtual void Initialize(DataMesh* mesh, DataContainer* dc);

    void addMaterial(Material* newMat){ myMaterials.push_back(newMat); }
    int getDataIndex(int iblock, int ielem, int ipoint )
        { return myDataMap[iblock](ielem, ipoint); }
    Intrepid::FieldContainer<int>& getBlockDataMap( int iblock )
        { return myDataMap[iblock]; }
 
  protected:
    vector<Material*> myMaterials;
    Intrepid::FieldContainer<int>* myMaterialTopology;
    Intrepid::FieldContainer<int>* myDataMap;
    DataContainer* dataContainer;
    DataMesh* dataMesh;
    int numMaterialPoints;

    VarIndex MATERIAL_ID;
};
/******************************************************************************/
/******************************************************************************/

class MCFactory {
  public:
    MaterialContainer* create(pugi::xml_node& xml_data);
};


/******************************************************************************/
/******************************************************************************/
class DefaultMaterialContainer : public MaterialContainer
{
  public:
    DefaultMaterialContainer(pugi::xml_node& xml_data);
    void initializeMaterialTopology();
  private:
    vector<int> blockToMaterialMap;
};
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/******************************************************************************/
/*!  This class owns one or more MaterialModels.  It is responsible for 
   extracting the necessary data from the data container, putting in the needed
   form, and calling the submodels.  
*/
class Material {
  public:
    virtual ~Material(){}
    Material( pugi::xml_node& matspec, int id );
    virtual void SetUp(DataContainer* dc);
    virtual void Initialize(int dataIndex, DataContainer* dc);

    virtual void getCurrentTangent(int dataIndex, DataContainer* dc,
                                   Intrepid::FieldContainer<double>*& C,
                                   VarIndex f, VarIndex x);
    virtual void updateMaterialState(int dataIndex, DataContainer* dc);
    virtual void updateMaterialState(int dataIndex, DataContainer* dc, VarIndex var);

    int getMyId(){ return myId; }

  private:
    int myId;
    Tensor crystalBasis;
    vector<MaterialModel*> materialModels;
};
/******************************************************************************/
/******************************************************************************/




#endif
