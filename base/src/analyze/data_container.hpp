#ifndef AXSIS_DATA_CONTAINER_H
#define AXSIS_DATA_CONTAINER_H

#include "types.hpp"
#include "math_library.hpp"
#include "communicator.hpp"
#include "exception_handling.hpp"

#include <string>
#include <vector>
#include <assert.h>
using std::string;
using std::endl;
using std::vector;

enum DataType { IntType, RawType, RealType, VectorType, SymTensorType, TensorType, ArrayType, };
enum DataCentering { UNSET, NODE, ELEM, MATPNT, MISC, NUM_CENTERINGS };
enum StateIndex { CURRENT=0, UPDATED };

class AbstractData {
  public:
    AbstractData( int arrayLength, string name, DataCentering cntr, DataType datatype, bool plot )
      { varLength = arrayLength; varName = name; varCntr = cntr; myType = datatype; varPlot = plot; }
    virtual ~AbstractData(){}

    // this is clunky, but the derived template only instantiates the correct accessor.
    // functions that aren't derived are superfluous, and should error out by default.
    virtual void getData( int*& data     , StateIndex state=CURRENT ) { pXcout << "request for integer from non-integer data" << endl; assert(0); }
    virtual void getData( int i, int*& data, StateIndex state=CURRENT ) { pXcout << "request for integer from non-integer data" << endl; assert(0); }
    virtual void getData( int i, int& data , StateIndex state=CURRENT ) { pXcout << "request for integer from non-integer data" << endl; assert(0); }
    virtual void setData( int i, int data  , StateIndex state=CURRENT ) { pXcout << "request for integer from non-integer data" << endl; assert(0); }

    virtual void getData( Real*& data     , StateIndex state=CURRENT ) { pXcout << "request for Real from non-Real data" << endl; assert(0); }
    virtual void getData( int i, Real*& data, StateIndex state=CURRENT ) { pXcout << "request for Real from non-Real data" << endl; assert(0); }
    virtual void getData( int i, Real& data , StateIndex state=CURRENT ) { pXcout << "request for Real from non-Real data" << endl; assert(0); }
    virtual void setData( int i, Real data  , StateIndex state=CURRENT ) { pXcout << "request for Real from non-Real data" << endl; assert(0); }

    virtual void getData( Vector*& data     , StateIndex state=CURRENT ) { pXcout << "request for Vector from non-Vector data" << endl; assert(0); }
    virtual void getData( int i, Vector*& data, StateIndex state=CURRENT ) { pXcout << "request for Vector from non-Vector data" << endl; assert(0); }
    virtual void getData( int i, Vector& data , StateIndex state) { pXcout << "request for Vector from non-Vector data" << endl; assert(0); }
    virtual void setData( int i, Vector data  , StateIndex state=CURRENT ) { pXcout << "request for Vector from non-Vector data" << endl; assert(0); }

    virtual void getData( Tensor*& data     , StateIndex state=CURRENT ) { pXcout << "request for Tensor from non-Tensor data" << endl; assert(0); }
    virtual void getData( int i, Tensor*& data, StateIndex state=CURRENT ) { pXcout << "request for Tensor from non-Tensor data" << endl; assert(0); }
    virtual void getData( int i, Tensor& data , StateIndex state=CURRENT ) { pXcout << "request for Tensor from non-Tensor data" << endl; assert(0); }
    virtual void setData( int i, Tensor data  , StateIndex state=CURRENT ) { pXcout << "request for Tensor from non-Tensor data" << endl; assert(0); }

    virtual void getData( SymTensor*& data     , StateIndex state=CURRENT ) { pXcout << "request for SymTensor from non-SymTensor data" << endl; assert(0); }
    virtual void getData( int i, SymTensor*& data, StateIndex state=CURRENT ) { pXcout << "request for SymTensor from non-SymTensor data" << endl; assert(0); }
    virtual void getData( int i, SymTensor& data , StateIndex state=CURRENT ) { pXcout << "request for SymTensor from non-SymTensor data" << endl; assert(0); }
    virtual void setData( int i, SymTensor data  , StateIndex state=CURRENT ) { pXcout << "request for SymTensor from non-SymTensor data" << endl; assert(0); }

    void dump();

    string getName(){ return varName; }
    DataType getDataType(){ return myType; }
    DataCentering getCentering(){ return varCntr; }
    Real getLength(){ return varLength; }
    bool isPlottable(){ return varPlot; }
    bool isExternal(){ return varExternal; }
    int getNumStates(){ return varNumStates; }

    virtual void updateExternalData(){ /* this is a no-op by default */ }

    virtual void advanceState(){}
  
  protected:
    DataCentering varCntr;       /*! centering of data */
    DataType      myType;
    string        varName;       /*! name of data */
    int           varLength;
    bool          varPlot;       /*! data plottable or not */
    bool          varExternal;   /*! locally owned? */
    int           varNumStates;
};


/******************************************************************************/
/* This is the implementation for standard types, both internal and external  */
/******************************************************************************/
template <class dataType>
class DataEntity : public AbstractData {
  public:
    // constructor for new local data
    DataEntity( int arrayLength, string name, 
                DataCentering cntr, int numStates, DataType datatype, bool plot ) 
              : AbstractData( arrayLength, name, cntr, datatype, plot )
      { zeroset(); 
        varNumStates = numStates;
        varPntr = new dataType*[varNumStates];
        for(int i=0; i<varNumStates; i++){
          varPntr[i] = new dataType [ arrayLength ]; 
          for(int j=0; j<arrayLength; j++) varPntr[i][j] = 0;
        }
      }

    // constructor for external data
    DataEntity( dataType* data, string name, int numData,
                DataCentering cntr, int numStates, DataType datatype, bool plot ) 
              : AbstractData( numData, name, cntr, datatype, plot )
      { zeroset(); 
        varNumStates = numStates;
        varPntr = new dataType*[numStates];
        varPntr[0] = data; 
        varExternal = true; }

    // constructor for external mapped data
    DataEntity( dataType* data, string name, vector<int>& map, 
                DataCentering cntr, int numStates, DataType datatype, bool plot ) 
              : AbstractData( map.size(), name, cntr, datatype, plot ) { 
        zeroset();

        varNumStates = numStates;
        varExternal = true;
        varMap = &map;
        varExtData = data;
        int ndata = map.size();
        varPntr = new dataType*[varNumStates];
        for(int i=0; i<varNumStates; i++) varPntr[i] = new dataType[ndata];
        dataType* initialState = varPntr[0];
        for(int i=0; i<ndata; i++)
          initialState[i] = data[map[i]];
      }

    virtual ~DataEntity() {
        if( !varExternal ){
          for(int i=0; i<varNumStates; i++) 
            delete [] varPntr[i];
          delete [] varPntr;
        } else if( varMap ){
          vector<int>& map = *varMap;
          int ndata = map.size();
          for(int i=0; i<ndata; i++) varExtData[map[i]] = varPntr[0][i];
          for(int i=0; i<varNumStates; i++)
            delete [] varPntr[i];
          delete [] varPntr;
        }
    }

    void advanceState() {
      if( varNumStates == 1 ) return;
      if( varNumStates == 2 ) {
        memcpy( varPntr[CURRENT], varPntr[UPDATED], varLength*sizeof(dataType) );
      } else {
        dataType* back = varPntr[varNumStates-1];
        for( int i=3; i<varNumStates; i++){
          varPntr[i] = varPntr[i-1];
        }
        varPntr[2] = varPntr[CURRENT];
        varPntr[CURRENT] = varPntr[UPDATED];
        varPntr[UPDATED] = back;
        memcpy( varPntr[UPDATED], varPntr[CURRENT], varLength*sizeof(dataType) );
      }
    }

    using AbstractData::getData;
    using AbstractData::setData;
    void getData( dataType*& data       , StateIndex state=CURRENT){ data = varPntr[state]; }
    void getData( int i, dataType*& data, StateIndex state=CURRENT){ data = varPntr[state]+i; }
    void getData( int i, dataType& data , StateIndex state=CURRENT){ data = varPntr[state][i]; }
    void setData( int i, dataType data  , StateIndex state=CURRENT){ varPntr[state][i] = data; }

  private:
    void zeroset(){ varExternal = false; varPntr = NULL; varMap = NULL; varExtData = NULL; }
    dataType**    varPntr;
    vector<int>*  varMap;
    dataType*     varExtData;
};
/******************************************************************************/


/******************************************************************************/
/* specialization for external vectors, tensors and symtensors                */
/******************************************************************************/
class ExternalVector : public AbstractData {
  public:
    ExternalVector( Real** data, string name, int numData,      DataCentering cntr, int numStates, bool plot );
    ExternalVector( Real** data, string name, vector<int>& map, DataCentering cntr, int numStates, bool plot );
    virtual ~ExternalVector();

    using AbstractData::getData;
    using AbstractData::setData;
    virtual void getData( Vector*& data       , StateIndex state=CURRENT);
    virtual void getData( int i, Vector*& data, StateIndex state=CURRENT);
    virtual void getData( int i, Vector& data , StateIndex state=CURRENT);
    virtual void setData( int i, Vector data  , StateIndex state=CURRENT);

    virtual void advanceState();

  private:
    Real* varPntr[3];
    Vector** varData;
    vector<int>* varMap;
};
class ExternalTensor : public AbstractData {
  public:
    ExternalTensor( Real** data, string name, int numData,      DataCentering cntr, int numStates, bool plot );
    ExternalTensor( Real** data, string name, vector<int>& map, DataCentering cntr, int numStates, bool plot );
    virtual ~ExternalTensor();

    using AbstractData::getData;
    using AbstractData::setData;
    virtual void getData( Tensor*& data       , StateIndex state=CURRENT);
    virtual void getData( int i, Tensor*& data, StateIndex state=CURRENT);
    virtual void getData( int i, Tensor& data , StateIndex state=CURRENT);
    virtual void setData( int i, Tensor data  , StateIndex state=CURRENT);

    virtual void advanceState();

  private:
    Real* varPntr[9];
    Tensor** varData;
    vector<int>* varMap;
};
class ExternalSymTensor : public AbstractData {
  public:
    ExternalSymTensor(Real** data, string name, int numData, DataCentering cntr, int numStates, bool plot);
    ExternalSymTensor(Real** data, string name, vector<int>& map, DataCentering cntr, int numStates, bool plot);
    virtual ~ExternalSymTensor();

    using AbstractData::getData;
    using AbstractData::setData;
    virtual void getData(SymTensor*& data, StateIndex state = CURRENT);
    virtual void getData(int i, SymTensor*& data, StateIndex state = CURRENT);
    virtual void getData(int i, SymTensor& data, StateIndex state = CURRENT);
    virtual void setData(int i, SymTensor data, StateIndex state = CURRENT);

    virtual void advanceState();

  private:
    Real* varPntr[6];
    SymTensor** varData;
    vector<int>* varMap;
};
/******************************************************************************/
 
class DataContainer {
  
public:
  
  DataContainer( );
  DataContainer( int nnodes, int nelements, int nmatpoints );
  DataContainer( string myname );
  ~DataContainer();
 
  /***** VARIABLE REGISTRATION FUNCTIONS *****/

  /*! variable registration for data that already exists.  */
  template <class myType>
  VarIndex registerVariable( DataType datatype,
                             myType* ptr,                     /*! ptr to existing data */
                             int numData,                     /*! length of inner array */
                             string,                          /*! name of data */
                             DataCentering centering=UNSET,   /*! centering of data */
                             int numStates=1,                 /*! name of data */
                             bool plot=false);                /*! plottable (true) or not */

  /*! variable registration for data that already exists.  */
  /** use this function for external data that has a non-trivial
      map between local and external data.  map must be provided */
  template <class myType>
  VarIndex registerVariable( DataType datatype,             /*! data rank */
                             myType* ptr,                   /*! ptr array to existing data */
                             vector<int>& map,              /*! local to external index map */
                             string,                        /*! name of data */
                             DataCentering centering=UNSET, /*! centering of data */
                             int numStates=1,               /*! name of data */
                             bool plot=false);              /*! plottable (true) or not */


  /*! variable registration for data that already exists.  */
  VarIndex registerVariable( DataType datatype,               /*! data rank */
                             Real** ptr,                      /*! ptr array to existing data */
                             int numData,                     /*! length of inner array */
                           string,                          /*! name of data */
                                   DataCentering centering=UNSET,   /*! centering of data */
                                   int numStates=1,                 /*! name of data */
                                   bool plot=false);                /*! plottable (true) or not */

  /*! variable registration for data that already exists.  */
  /** use this function for external data that has a non-trivial
      map between local and external data.  map must be provided */
  VarIndex registerVariable( DataType datatype,               /*! data rank */
                             Real** ptr,                      /*! ptr array to existing data */
                             vector<int>& map,              /*! local to external index map */
                             string,                          /*! name of data */
                             DataCentering centering=UNSET,   /*! centering of data */
                             int numStates=1,                 /*! name of data */
                             bool plot=false);                /*! plottable (true) or not */


  /*! variable registration for new data */
  VarIndex registerVariable( DataType datatype,         /*! data rank */
                             string,                    /*! name of data */
                             DataCentering centering,   /*! centering of data */
                             int num,                   /*! number of items */
                             bool plot=false,           /*! plottable (true) or not */
                             int numStates=1,           /*! name of data */
                             bool varNew=false);        /*! must be new (true) */

  /*! variable registration for new data */
  VarIndex registerVariable( DataType datatype,         /*! data rank */
                             string,                    /*! name of data */
                             DataCentering centering,   /*! centering of data */
                             bool plot=false,           /*! plottable (true) or not */
                             int numStates=1,           /*! name of data */
                             bool varNew=false);        /*! must be new (true) */

  void advanceState();
  void advanceState(VarIndex varindex);


  /***** VARIABLE ACCESS FUNCTIONS *****/
  template <class myType> void getVariable( VarIndex, myType*& d, StateIndex=CURRENT );
  template <class myType> void getVariable( VarIndex, myType*& d, int i, StateIndex=CURRENT );
  template <class myType> void getVariable( VarIndex, myType& d, int i, StateIndex=CURRENT );
  template <class myType> void setVariable( VarIndex, myType& d, int i, StateIndex=CURRENT );

  template <class myType> void getVariable( string name, myType*& d, StateIndex );

  void getVariable( string name, VarIndex& var );

  void  setName( string myname ) { myName = myname; }
  void  setNumNodes(int nn) { numnodes = nn; }       /*! set total number of nodes (ghosted+owned) */
  void  setNumElems(int ne) { numelements = ne; }    /*! set total number of elements (owned) */
  void  setNumMatPoints(int nm) { nummatpoints = nm;}/*! set total number of mat points (owned) */
  int  getNumNodes() { return numnodes; }       /*! set total number of nodes (ghosted+owned) */
  int  getNumElems() { return numelements; }    /*! set total number of elements (owned) */
  int  getNumMatPoints() { return nummatpoints;}/*! set total number of mat points (owned) */
  string getVariableName( VarIndex );
  int   getNumVariables(){return dataEntities.size();}
  bool  isPlotVariable( VarIndex );
  bool  isNodalVariable( VarIndex );
  bool  isElementVariable( VarIndex );
  void  dumpAll();
  void  deleteAll();
  int   getCentering( VarIndex );
  void updateExternalData();

  AbstractData* getAbstractVariable(int i){ return dataEntities[i]; }

  vector<int>* getEmptyDataMap()
  {
     vector<int>* newmap = new vector<int>; 
     dataMaps.push_back(newmap); 
     return dataMaps.back();
  }

private:
  vector< AbstractData* > dataEntities;

  string myName;             /*! name of data container */
  int numnodes;                     /*! if num isn't provided, ... */
  int numelements;             /*! if num isn't provided, ... */
  int nummatpoints;          /*! if num isn't provided, ... */

  vector< vector<int>* > dataMaps;
  
  bool variableExists( string name, int& index );

  DataContainer(const DataContainer&);
  DataContainer& operator=(const DataContainer&);
  
  void setUp();
  VarIndex addnew( int num, string name, DataCentering centering,
                   int numStates, DataType datatype, bool plot);


};

template <class myType>
void DataContainer::getVariable( VarIndex varIndex, myType*& d, StateIndex state )
{
  int ndata = dataEntities.size();
  if( (varIndex >= 0) && (varIndex < ndata) ) {
    AbstractData*& ad = dataEntities[varIndex];
    if( state < ad->getNumStates() ){
      ad->getData( d, state );
      return;
    } else {
      throw RunTimeError( "State requested that doesn't exist" );
    }
  }
  // not found...
  pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
       << "\tInvalid variable index = " << varIndex << std::endl;
  return;
}

template <class myType>
void DataContainer::getVariable( VarIndex varIndex, myType*& d, int i, StateIndex state )
{
  int ndata = dataEntities.size();
  if( (varIndex >= 0) && (varIndex < ndata) ) {
    AbstractData*& ad = dataEntities[varIndex];
    if( state < ad->getNumStates() ){
      ad->getData( i, d, state );
      return;
    } else {
      throw RunTimeError( "State requested that doesn't exist" );
    }
  }
  // not found...
  pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
       << "\tInvalid variable index = " << varIndex << std::endl;
  return;
}

template <class myType>
void DataContainer::getVariable( VarIndex varIndex, myType& d, int i, StateIndex state )
{
  int ndata = dataEntities.size();
  if( (varIndex >= 0) && (varIndex < ndata) ) {
    AbstractData*& ad = dataEntities[varIndex];
    if( state < ad->getNumStates() ){
      ad->getData( i, d, state );
      return;
    } else {
      throw RunTimeError( "State requested that doesn't exist" );
    }
  }
  // not found...
  pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
       << "\tInvalid variable index = " << varIndex << std::endl;
  return;
}

template <class myType>
void DataContainer::setVariable( VarIndex varIndex, myType& d, int i, StateIndex state )
{
  int ndata = dataEntities.size();
  if( (varIndex >= 0) && (varIndex < ndata) ) {
    AbstractData*& ad = dataEntities[varIndex];
    if( state < ad->getNumStates() ){
      ad->setData( i, d, state );
      return;
    } else {
      throw RunTimeError( "State requested that doesn't exist" );
    }
  }
  // not found...
  pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
       << "\tInvalid variable index = " << varIndex << std::endl;
  return;
}

template <class myType>
void DataContainer::getVariable( string name, myType*& d, StateIndex state )
{
  int ndata = dataEntities.size();
  for( int i=0; i<ndata; i++){
    AbstractData*& ad = dataEntities[i];
    if(name == ad->getName()){
      if( state < ad->getNumStates() ){
        ad->getData( d, state );
        return;
      } else {
        throw RunTimeError( "State requested that doesn't exist" );
      }
    }
  }
  // not found...
  pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
       << "\tInvalid variable name = " << name << std::endl;
  return;
}

template <class myType>
VarIndex DataContainer::registerVariable( DataType datatype, 
                                          myType* pntr,
                                          int numData,
                                          string name,
                                          DataCentering centering,
                                          int numStates,
                                          bool plot)
{
  VarIndex varIndex;
  if( variableExists(name, varIndex) ){
    pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
           << "\tVariable (" << name << ") already exists" << std::endl;
    pXcout << "Attempted to overwrite data in DataContainer" << std::endl;
    return UNSET_VAR_INDEX;
  } else {

    AbstractData* newdata = new DataEntity<myType>( pntr, name, numData, centering, numStates, datatype, plot );
    int index = dataEntities.size();
    dataEntities.push_back( newdata );
    return ( index );
  }
}

template <class myType>
VarIndex DataContainer::registerVariable( DataType datatype, 
                                          myType* pntr,
                                          vector<int>& map,
                                          string name,
                                          DataCentering centering,
                                          int numStates,
                                          bool plot)
{
  VarIndex varIndex;
  if( variableExists(name, varIndex) ){
    pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
           << "\tVariable (" << name << ") already exists" << std::endl;
    pXcout << "Attempted to overwrite data in DataContainer" << std::endl;
    return UNSET_VAR_INDEX;
  } else {

    AbstractData* newdata = new DataEntity<myType>( pntr, name, map, centering, numStates, datatype, plot );
    int index = dataEntities.size();
    dataEntities.push_back( newdata );
    return ( index );
  }
}


#endif //AXSIS_DATA_CONTAINER_H
