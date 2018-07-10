#include "material_container.hpp"
#include "data_mesh.hpp"
#include "data_container.hpp"
#include "exception_handling.hpp"
#include "topological_element.hpp"
#include "communicator.hpp"


/******************************************************************************/
DefaultMaterialContainer::DefaultMaterialContainer(pugi::xml_node& xml_data) 
                         : MaterialContainer(xml_data)
/******************************************************************************/
{
  // loop through block specs and build blockMap : block_index --> domain_topo_index
  pugi::xml_node meshspec = xml_data.child("mesh");
  int nblockspecs = Plato::Parse::numChildren( meshspec, "block" );
  blockToMaterialMap.resize(nblockspecs);
  int ib=0;
  for(pugi::xml_node bspec = meshspec.child("block"); bspec;
        bspec = bspec.next_sibling("block"), ++ib)
  {
    int matId = Plato::Parse::getInt( bspec, "material" );
    if( matId != -1 ){
      // user requested a material by id.  find the index.
      int nmats = myMaterials.size();
      bool found = false;
      for(int imat=0; imat<nmats; imat++){
        if( matId == myMaterials[imat]->getMyId() ){
          blockToMaterialMap[ib] = imat;
          found = true;
          break;
        }
      }
      if( !found ){
        stringstream err;
        err << "material " << matId << " not found.";
        throw ParsingException(err.str() );
      }
    }
  }
}

/******************************************************************************/
void MaterialContainer::initializeDataMap()
/******************************************************************************/
{

  bool plottable = true;
  MATERIAL_ID = dataContainer->registerVariable( RealType, "material id", 
                                                 MATPNT, plottable );
  Real* matdata;
  dataContainer->getVariable(MATERIAL_ID, matdata);
  
  int dataIndex = 0;
  int numBlocks = dataMesh->getNumElemBlks();
  for(int iblock=0; iblock<numBlocks; iblock++){
    Topological::Element& elblock = *(dataMesh->getElemBlk(iblock));
    int numElements = elblock.getNumElem();
    int numPoints = elblock.getNumIntPoints();
    Intrepid::FieldContainer<int>& dmap = myDataMap[iblock];
    Intrepid::FieldContainer<int>& topo = myMaterialTopology[iblock];
    for(int ielem=0; ielem<numElements; ielem++)
      for(int ipoint=0; ipoint<numPoints; ipoint++){
        matdata[dataIndex] = (double)(topo(ielem, ipoint));
        dmap(ielem, ipoint) = dataIndex++;
      }
  }
}   

/******************************************************************************/
void DefaultMaterialContainer::initializeMaterialTopology()
/******************************************************************************/
{
  int numBlocks = dataMesh->getNumElemBlks();
  for(int iblock=0; iblock<numBlocks; iblock++){
    Topological::Element& elblock = *(dataMesh->getElemBlk(iblock));
    int numElements = elblock.getNumElem();
    int numPoints = elblock.getNumIntPoints();
    int matIndex = blockToMaterialMap[iblock];
    Intrepid::FieldContainer<int>& topo = myMaterialTopology[iblock];
    for(int ielem=0; ielem<numElements; ielem++)
      for(int ipoint=0; ipoint<numPoints; ipoint++){
        topo(ielem, ipoint) = matIndex;
      }
  }
}   


/******************************************************************************/
MaterialContainer::MaterialContainer(pugi::xml_node& xml_data)
/******************************************************************************/
{

  for(pugi::xml_node matspec = xml_data.child("material"); matspec;

      matspec = matspec.next_sibling("material")){

      int id = Plato::Parse::getInt( matspec, "id" );
      auto newMat = new Material( matspec, id );

      addMaterial( newMat );
  }
}

/******************************************************************************/
MaterialContainer* MCFactory::create(pugi::xml_node& xml_data)
/******************************************************************************/
{
  return new DefaultMaterialContainer(xml_data);
}

/******************************************************************************/
void MaterialContainer::setMaterialTopology( DataMesh* mesh, 
                                             DataContainer* dc,
                                             pugi::xml_node& node)
/******************************************************************************/
{
  dataContainer = dc;
  dataMesh = mesh;

  int numBlocks = mesh->getNumElemBlks();
  myMaterialTopology = new Intrepid::FieldContainer<int>[numBlocks];
  myDataMap = new Intrepid::FieldContainer<int>[numBlocks];
  numMaterialPoints = 0;
  for(int iblock=0; iblock<numBlocks; iblock++){
    Topological::Element& elblock = *(mesh->getElemBlk(iblock));
    int numElems = elblock.getNumElem();
    int numPoints = elblock.getNumIntPoints();
    myMaterialTopology[iblock].resize(numElems, numPoints);
    myDataMap[iblock].resize(numElems, numPoints);
    numMaterialPoints += numElems*numPoints;
  }

  // material topology is now set, so inform data container of num mat points
  // before setting up material models
  dc->setNumMatPoints( numMaterialPoints );

  // TODO: initialize myMaterialTopology to point to correct material
  initializeMaterialTopology();

  // TODO: initialize myDataMap to point to data
  initializeDataMap();
}

/******************************************************************************/
void MaterialContainer::getCurrentTangent(int iblock, int ielement, int ipoint, 
                                          Intrepid::FieldContainer<double>*& C,
                                          VarIndex f, VarIndex x)
/******************************************************************************/
{
  int matIndex = myMaterialTopology[iblock](ielement, ipoint);
  int dataIndex = myDataMap[iblock](ielement, ipoint);
  myMaterials[matIndex]->getCurrentTangent(dataIndex, dataContainer, C, f, x);
  
}
/******************************************************************************/
void MaterialContainer::updateMaterialState(int iblock, int ielement, int ipoint)
/******************************************************************************/
{
  int matIndex = myMaterialTopology[iblock](ielement, ipoint);
  int dataIndex = myDataMap[iblock](ielement, ipoint);
  myMaterials[matIndex]->updateMaterialState(dataIndex, dataContainer);
}
/******************************************************************************/
void MaterialContainer::updateMaterialState(int iblock, int ielement, int ipoint, 
                                            VarIndex var)
/******************************************************************************/
{
  int matIndex = myMaterialTopology[iblock](ielement, ipoint);
  int dataIndex = myDataMap[iblock](ielement, ipoint);
  myMaterials[matIndex]->updateMaterialState( dataIndex, dataContainer, var);
}

/******************************************************************************/
void MaterialContainer::Initialize(DataMesh* mesh, DataContainer* dc)
/******************************************************************************/
{
  int nblocks = mesh->getNumElemBlks();
  for(int ib=0; ib<nblocks; ib++){
    Topological::Element& elblock = *(mesh->getElemBlk(ib));
    int nelems = elblock.getNumElem();
    for(int iel=0; iel<nelems; iel++){
      int numCubPoints = elblock.getNumIntPoints();
      for(int ip=0; ip<numCubPoints; ip++){
        int matIndex = myMaterialTopology[ib](iel, ip);
        int dataIndex = myDataMap[ib](iel, ip);
        myMaterials[matIndex]->Initialize(dataIndex, dc);
      }
    }
  }
}

/******************************************************************************/
void MaterialContainer::SetUp(DataMesh* mesh, DataContainer* dc, pugi::xml_node& node )
/******************************************************************************/
{

  setMaterialTopology( mesh, dc, node);

  int nmats = myMaterials.size();
  for(int imat=0; imat<nmats; imat++){
    myMaterials[imat]->SetUp(dc);
  }
}

#include "linear_elastic.hpp"

typedef struct{ string name; NewModelFP model; } ModelPair;

/******************************************************************************/
Material::Material( pugi::xml_node& matspec, int id )
/******************************************************************************/
{
  // add models here
  vector<ModelPair> models;
  models.push_back( (ModelPair){"linear_elastic",            NewIsotropicElastic3D} );

  // parse crystal basis
  pugi::xml_node basisSpec = matspec.child( "crystal_basis" );
  if( basisSpec ){
    int nangles = Plato::Parse::numChildren( basisSpec, "euler_angles" );
    if( nangles != 3 ){
        stringstream err;
        err << "Three euler angles required. " << nangles << " given.";
        throw ParsingException(err.str() );
    }
    Tensor xrot, yrot, zrot;
    Real angle;
    pugi::xml_node cur_angle = matspec.child("euler_angles");
    angle = Plato::Parse::getDouble( cur_angle, "euler_angle" );
    xrot[TENSOR::XX] =  1.0;
    xrot[TENSOR::YY] =  cos(angle);
    xrot[TENSOR::YZ] = -sin(angle);
    xrot[TENSOR::ZY] =  sin(angle);
    xrot[TENSOR::ZZ] =  cos(angle);
    cur_angle = cur_angle.next_sibling("euler_angles");
    angle = Plato::Parse::getDouble( cur_angle, "euler_angle" );
    yrot[TENSOR::XX] =  cos(angle);
    yrot[TENSOR::XZ] =  sin(angle);
    yrot[TENSOR::YY] =  1.0;
    yrot[TENSOR::ZX] = -sin(angle);
    yrot[TENSOR::ZZ] =  cos(angle);
    cur_angle = cur_angle.next_sibling("euler_angles");
    angle = Plato::Parse::getDouble( cur_angle, "euler_angle" );
    zrot[TENSOR::XX] =  cos(angle);
    zrot[TENSOR::XY] = -sin(angle);
    zrot[TENSOR::YX] =  sin(angle);
    zrot[TENSOR::YY] =  cos(angle);
    zrot[TENSOR::ZZ] =  1.0;
    Tensor yzrot = yrot*zrot;
    crystalBasis = xrot*yzrot;
  } else {
    crystalBasis[TENSOR::XX] = 1.0;
    crystalBasis[TENSOR::YY] = 1.0;
    crystalBasis[TENSOR::ZZ] = 1.0;
  }
  

  int ntypemodels = models.size();
  for(int imat=0; imat<ntypemodels; imat++){
    pugi::xml_node matl = matspec.child(models[imat].name.c_str());
    if( matl )
      materialModels.push_back(models[imat].model(matl));
  }

  myId = id;
}

bool ModelComparator::operator()(MaterialModel* a, MaterialModel* b)
{
  return a->getDependentVariable() == b->getIndependentVariable();
}

/******************************************************************************/
void Material::Initialize(int dataIndex, DataContainer* dc)
/******************************************************************************/
{
  int nmods = materialModels.size();
  for(int imod=0; imod<nmods; imod++){
    materialModels[imod]->Initialize(dataIndex, dc);
  }
}

/******************************************************************************/
void Material::SetUp(DataContainer* dc)
/******************************************************************************/
{
  int nmods = materialModels.size();
  for(int imod=0; imod<nmods; imod++){
    materialModels[imod]->SetUp(dc, crystalBasis);
  }

  // sort models by dependent and independent variables
  ModelComparator modelOrder;
  sort( materialModels.begin(), materialModels.end(), modelOrder );

}


/******************************************************************************/
void Material::getCurrentTangent(int dataIndex, DataContainer* dc,
                                 Intrepid::FieldContainer<double>*& C,
                                 VarIndex f, VarIndex x)
/******************************************************************************/
{
  int nmodels = materialModels.size();
  for( int imat=0; imat<nmodels; imat++ ){
    MaterialModel& model = *(materialModels[imat]);
    if( model.getDependentVariable() == f && model.getIndependentVariable() == x ){
      materialModels[imat]->Tangent(dataIndex, dc, C);
    }
  }
}
/******************************************************************************/
void Material::updateMaterialState(int dataIndex, DataContainer* dc)
/******************************************************************************/
{
  int nmodels = materialModels.size();
  for( int imat=0; imat<nmodels; imat++ )
    materialModels[imat]->UpdateMaterialState(dataIndex, dc);
}

/******************************************************************************/
void Material::updateMaterialState(int dataIndex, DataContainer* dc, VarIndex var)
/******************************************************************************/
{
  int nmodels = materialModels.size();
  for( int imat=0; imat<nmodels; imat++ ){
    MaterialModel& model = *(materialModels[imat]);
    if( model.getDependentVariable() == var )
      materialModels[imat]->UpdateMaterialState(dataIndex, dc);
  }
}
