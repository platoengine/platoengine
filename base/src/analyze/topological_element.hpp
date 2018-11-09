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

// Intrepid includes
#include "Intrepid_FunctionSpaceTools.hpp"
#include "Intrepid_FieldContainer.hpp"
#include "Intrepid_CellTools.hpp"
#include "Intrepid_ArrayTools.hpp"
#include "Intrepid_HGRAD_HEX_C1_FEM.hpp"
#include "Intrepid_RealSpaceTools.hpp"
#include "Intrepid_DefaultCubatureFactory.hpp"
#include "Intrepid_Utils.hpp"

#include "types.hpp"
#include "Plato_Parser.hpp"
#include "core/Cogent_Integrator.hpp"
#include <cassert>

#include <string>
using std::string;

#include <vector>
using std::vector;

class DataContainer;
namespace Topological {

class ElementIntegration{
  public: 
    ElementIntegration(bool uniform = false) : uniformCubature(uniform) {}
    virtual ~ElementIntegration();
    Intrepid::FieldContainer<Real>& getCubaturePoints() { return *cubPoints; }
    Intrepid::FieldContainer<Real>& getCubatureWeights() { return *cubWeights; }
    virtual void getCubatureWeights(Intrepid::FieldContainer<double>& cubWeights, 
                                    const Intrepid::FieldContainer<double>& nodes){}
    bool cubatureIsUniform(){ return uniformCubature; }
    int getNumIntPoints(){ return cubPoints->dimension(0); }
  protected:
    bool uniformCubature;
    Intrepid::FieldContainer<double> *cubPoints;
    Intrepid::FieldContainer<double> *cubWeights;
};

class IntrepidIntegration : public ElementIntegration
{
  public: 
    IntrepidIntegration( pugi::xml_node& node, Teuchos::RCP<shards::CellTopology> blockTopology );
    virtual ~IntrepidIntegration() {}
  private:
    Teuchos::RCP<Intrepid::Cubature<double> > cubature;
};

class CustomIntegration : public ElementIntegration
{
  public: 
    CustomIntegration( pugi::xml_node& node, int myDim );
    virtual ~CustomIntegration() {}
};

class CogentIntegration : public ElementIntegration
{
  public: 
    CogentIntegration( pugi::xml_node& node, Teuchos::RCP<shards::CellTopology> blockTopology );
    virtual ~CogentIntegration() {}
    void getCubatureWeights(Intrepid::FieldContainer<double>& cubWeights, 
                      const Intrepid::FieldContainer<double>& nodes);
  private:
    int mNumNodes, mNumDims, mNumPts;
    Teuchos::RCP<Cogent::Integrator> mCubature;
    Kokkos::DynRankView<Real, Kokkos::Serial> mCoordVals;
    Kokkos::DynRankView<Real, Kokkos::Serial> mWeights;
};

class Element {
  
public:
  Element() {zeroset();}
  Element( int number, int nattr=0 );
  virtual ~Element();
  
public:
  virtual void setDataContainer(DataContainer* dc);
  virtual void registerData() = 0;
  virtual void connectNodes(int, int, int*);
  virtual int* getNodeConnect();
  virtual double* getAttributes() {return attributes;}
  virtual int  getNnpe() {return myNnpe;}
  virtual int  getNattr() {return myNattr;}
  virtual int  getNnps() {return myNnps;}
  virtual int  getDim() {return myDim;}
  virtual int  getNumElem() {return myNel;}
  virtual const char* getType() { return myType.c_str(); }
  virtual int  getBlockId() { return groupID; }
  virtual void setBlockId(int bid) { groupID = bid; }

  void Connect(int* gid, int lid);
  int* Connect(int lid);

  virtual void setIntegrationMethod(pugi::xml_node& node);

  int getNumIntPoints(){ return elementIntegration->getNumIntPoints(); }
  Intrepid::FieldContainer<Real>& getCubaturePoints() { return elementIntegration->getCubaturePoints(); }
  Intrepid::FieldContainer<Real>& getCubatureWeights() { return elementIntegration->getCubatureWeights(); }
  void getCubatureWeights(Intrepid::FieldContainer<Real>& weights, 
                    const Intrepid::FieldContainer<Real>& nodes )
  {
    elementIntegration->getCubatureWeights(weights, nodes);
  }
  bool cubatureIsUniform(){ return elementIntegration->cubatureIsUniform(); }

  Intrepid::Basis<double, Intrepid::FieldContainer<double> >& getBasis(){ return *blockBasis; }
  shards::CellTopology& getTopology() { return *blockTopology; }

  virtual void CurrentCoordinates(int* node_gid_list, Real** X, Real* curcoor) = 0;

protected:
  void zeroset();

  DataContainer* myData;
  VarIndex NODECONNECT;
  VarIndex GLOBALID;

  // element attributes
  VarIndex ATTRIBUTES;

  // if there are node or element data, store here
  VarIndex* INPUTDATA;

  int myNnpe;
  int myNnps;
  int myNel;
  int myNattr;
  int myDim;
  string myType;
  int* nodeConnect;
  int* globalID;
  double* attributes;
  int groupID; //! Also blockid in exodus-lingo

  ElementIntegration* elementIntegration;
  Teuchos::RCP<shards::CellTopology> blockTopology;
  Intrepid::Basis<double, Intrepid::FieldContainer<double> > *blockBasis;

private:
  Element(const Element&);
  Element& operator=(const Element&);
};


class Beam : public Element
{
public:
    Beam( int number, int nattr=0 ): Element( number, nattr ){ init(); }
    Beam( int number, pugi::xml_node& node): Element( number ){ init(); setIntegrationMethod(node); }
    virtual ~Beam();
    virtual void registerData();
    virtual void CurrentCoordinates(int* node_gid_list, Real** X, Real* curcoor);

private:
    void init();
    Beam(const Beam&);
    Beam& operator=(const Beam&);
};

class Tri3 : public Element
{
public:
  Tri3( int number, int nattr=0 ): Element( number, nattr ){ init(); }
  Tri3( int number, pugi::xml_node& node): Element( number ){ init(); setIntegrationMethod(node); }
  virtual ~Tri3();
  virtual void registerData();
  virtual void CurrentCoordinates(int* node_gid_list, Real** X, Real* curcoor);

private:
  void init();
  Tri3(const Tri3&);
  Tri3& operator=(const Tri3&);
};

class Quad4 : public Element
{
public:
  Quad4( int number, int nattr=0 ): Element( number, nattr ){ init(); }
  Quad4( int number, pugi::xml_node& node): Element( number ){ init(); setIntegrationMethod(node); }
  virtual ~Quad4();
  virtual void registerData();
  virtual void CurrentCoordinates(int* node_gid_list, Real** X, Real* curcoor);

private:
  void init();
  Quad4(const Quad4&);
  Quad4& operator=(const Quad4&);
};

class Quad8 : public Element
{
public:
  Quad8( int number, int nattr=0 ): Element( number, nattr ){ init(); }
  Quad8( int number, pugi::xml_node& node): Element( number ){ init(); setIntegrationMethod(node); }
  virtual ~Quad8();
  virtual void registerData();
  virtual void CurrentCoordinates(int* node_gid_list, Real** X, Real* curcoor);

private:
  void init();
  Quad8(const Quad8&);
  Quad8& operator=(const Quad8&);
};

class Hex8 : public Element
{
public:
  Hex8( int number, int nattr=0 ): Element( number, nattr ){ init(); }
  Hex8( int number, pugi::xml_node& node ): Element( number ){ init(); setIntegrationMethod(node); }
  virtual ~Hex8();
  virtual void registerData();
  virtual void CurrentCoordinates(int* node_gid_list, Real** X, Real* curcoor);

private:
  void init();
  Hex8(const Hex8&);
  Hex8& operator=(const Hex8&);
};

class Hex20 : public Element
{
public:
  Hex20( int number, int nattr=0 ): Element( number, nattr ){ init(); }
  Hex20( int number, pugi::xml_node& node): Element( number ){ init(); setIntegrationMethod(node); }
  virtual ~Hex20();
  virtual void registerData();
  virtual void CurrentCoordinates(int* node_gid_list, Real** X, Real* curcoor);

private:
  void init();
  Hex20(const Hex20&);
  Hex20& operator=(const Hex20&);
};

class Tet4 : public Element
{
public:
  Tet4( int number, int nattr=0 ): Element( number, nattr ){ init(); }
  Tet4( int number, pugi::xml_node& node): Element( number ){ init(); setIntegrationMethod(node); }
  virtual ~Tet4();
  virtual void registerData();
  virtual void CurrentCoordinates(int* node_gid_list, Real** X, Real* curcoor);

private:
  void init();
  Tet4(const Tet4&);
  Tet4& operator=(const Tet4&);
};

class NullElement : public Element
{
public:
  NullElement( int number, int nattr=0 ): Element( number, nattr ){ zeroset(); }
  NullElement( int number, pugi::xml_node& node): Element( number ){ zeroset(); setIntegrationMethod(node); }
  virtual ~NullElement(){};
  virtual void registerData();
  virtual void CurrentCoordinates(int* node_gid_list, Real** X, Real* curcoor);
private:
  NullElement(const NullElement&);
  NullElement& operator=(const NullElement&);
};


} //namespace TOPOLOGICAL

