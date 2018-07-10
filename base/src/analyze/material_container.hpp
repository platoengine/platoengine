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
