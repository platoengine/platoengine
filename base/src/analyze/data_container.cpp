#include "types.hpp"

#include "communicator.hpp"
#include "data_container.hpp"

#include <iostream>
#include <string>

using std::endl;
using std::string;

/******************************************************************************/
ExternalVector::ExternalVector( Real** data, string name, vector<int>& map, 
                                DataCentering cntr, int numStates, bool plot )
                              : AbstractData( map.size(), name, cntr, VectorType, plot )
{
  varExternal = true;
  varNumStates = numStates;
  varMap = &map;
  for(int i=0; i<3; i++) varPntr[i] = data[i];
  int ndata = map.size();
  varData = new Vector*[varNumStates];
  for( int i=0; i<varNumStates; i++ ) 
    varData[i] = new Vector [ndata];
  Vector* initialState = varData[0];
  for( int i=0; i<ndata; i++){
    int imp = map[i];
    initialState[i] = Vector( varPntr[0][imp], varPntr[1][imp], varPntr[2][imp] );
  }
}

ExternalVector::ExternalVector( Real** data, string name, int numData,
                                DataCentering cntr, int numStates, bool plot )
                              : AbstractData( numData, name, cntr, VectorType, plot )
{
  varExternal = true;
  varNumStates = numStates;
  varMap = NULL;
  for(int i=0; i<3; i++) varPntr[i] = data[i];
  varData = new Vector*[varNumStates];
  for( int i=0; i<varNumStates; i++ ) 
    varData[i] = new Vector [varLength];
  Vector* initialState = varData[0];
  for( int i=0; i<varLength; i++){
    initialState[i] = Vector( varPntr[0][i], varPntr[1][i], varPntr[2][i]);
  }
}

void ExternalVector::getData(Vector*& data, StateIndex state)
{
    data = varData[state];
}
void ExternalVector::getData(int i, Vector*& data, StateIndex state)
{
    data = varData[state] + i;
}
void ExternalVector::getData(int i, Vector& data, StateIndex state)
{
    data = varData[state][i];
}
void ExternalVector::setData(int i, Vector data, StateIndex state)
{
    varData[state][i] = data;
}

void ExternalVector::advanceState(){
  if( varNumStates == 1 ) return;
  if( varNumStates == 2 ) {
    memcpy( varData[CURRENT], varData[UPDATED], varLength*sizeof(Vector) );
  } else {
    Vector* back = varData[varNumStates-1];
    for( int i=3; i<varNumStates; i++){
      varData[i] = varData[i-1];
    }
    varData[2] = varData[CURRENT];
    varData[CURRENT] = varData[UPDATED];
    varData[UPDATED] = back;
    memcpy( varData[UPDATED], varData[CURRENT], varLength*sizeof(Vector) );
  }
}

ExternalVector::~ExternalVector()
{
  if( varMap ){
    vector<int>& map = *varMap;
    int ndata = map.size();
    Vector* initialState = varData[0];
    for(int i=0; i<ndata; i++){
      int imp = map[i];
      Real* data; initialState[i].extractData( data );
      for(int j=0; j<3; j++) varPntr[j][imp] = data[j];
    }
  } else {
    Vector* initialState = varData[0];
    for(int i=0; i<varLength; i++){
      Real* data; initialState[i].extractData( data );
      for(int j=0; j<3; j++) varPntr[j][i] = data[j];
    }
  }
  for(int i=0; i<varNumStates; i++)
    delete [] varData[i];
  delete [] varData;
}
/******************************************************************************/

 
/******************************************************************************/
ExternalTensor::ExternalTensor( Real** data, string name, vector<int>& map, 
                                DataCentering cntr, int numStates, bool plot )
                              : AbstractData( map.size(), name, cntr, TensorType, plot )
{
  varExternal = true;
  varNumStates = numStates;
  varMap = &map;
  for(int i=0; i<9; i++) varPntr[i] = data[i];
  int ndata = map.size();
  varData = new Tensor*[varNumStates];
  for( int i=0; i<varNumStates; i++ ) 
    varData[i] = new Tensor [ndata];
  Tensor* initialState = varData[0];
  for( int i=0; i<ndata; i++){
    int imp = map[i];
    initialState[i] = Tensor( varPntr[0][imp], varPntr[1][imp], varPntr[2][imp],
                         varPntr[3][imp], varPntr[4][imp], varPntr[5][imp],
                         varPntr[6][imp], varPntr[7][imp], varPntr[8][imp] );
  }
}

ExternalTensor::ExternalTensor( Real** data, string name, int numData,
                                DataCentering cntr, int numStates, bool plot )
                              : AbstractData( numData, name, cntr, TensorType, plot )
{
  varExternal = true;
  varNumStates = numStates;
  varMap = NULL;
  for(int i=0; i<9; i++) varPntr[i] = data[i];
  varData = new Tensor*[varNumStates];
  for( int i=0; i<varNumStates; i++ ) 
    varData[i] = new Tensor [varLength];
  Tensor* initialState = varData[0];
  for( int i=0; i<varLength; i++){
    initialState[i] = Tensor( varPntr[0][i], varPntr[1][i], varPntr[2][i],
                         varPntr[3][i], varPntr[4][i], varPntr[5][i],
                         varPntr[6][i], varPntr[7][i], varPntr[8][i] );
  }
}

void ExternalTensor::getData(Tensor*& data, StateIndex state)
{
    data = varData[state];
}
void ExternalTensor::getData(int i, Tensor*& data, StateIndex state)
{
    data = varData[state] + i;
}
void ExternalTensor::getData(int i, Tensor& data, StateIndex state)
{
    data = varData[state][i];
}
void ExternalTensor::setData(int i, Tensor data, StateIndex state)
{
    varData[state][i] = data;
}

void ExternalTensor::advanceState()
{
    if(varNumStates == 1)
        return;
    if(varNumStates == 2)
    {
        memcpy(varData[CURRENT], varData[UPDATED], varLength * sizeof(Tensor));
    }
    else
    {
    Tensor* back = varData[varNumStates-1];
    for( int i=3; i<varNumStates; i++){
      varData[i] = varData[i-1];
    }
    varData[2] = varData[CURRENT];
    varData[CURRENT] = varData[UPDATED];
    varData[UPDATED] = back;
    memcpy( varData[UPDATED], varData[CURRENT], varLength*sizeof(Tensor) );
  }
}


ExternalTensor::~ExternalTensor()
{
  if( varMap ){
    vector<int>& map = *varMap;
    int ndata = map.size();
    Tensor* initialState = varData[0];
    for(int i=0; i<ndata; i++){
      int imp = map[i];
      Real* data; initialState[i].extractData( data );
      for(int j=0; j<9; j++) varPntr[j][imp] = data[j];
    }
  } else {
    Tensor* initialState = varData[0];
    for(int i=0; i<varLength; i++){
      Real* data; initialState[i].extractData( data );
      for(int j=0; j<9; j++) varPntr[j][i] = data[j];
    }
  }
  for(int i=0; i<varNumStates; i++)
    delete [] varData[i];
  delete [] varData;
}
/******************************************************************************/



/******************************************************************************/
ExternalSymTensor::ExternalSymTensor( Real** data, string name, vector<int>& map, 
                                DataCentering cntr, int numStates, bool plot )
                              : AbstractData( map.size(), name, cntr, SymTensorType, plot )
{
  varExternal = true;
  varNumStates = numStates;
  varMap = &map;
  for(int i=0; i<6; i++) varPntr[i] = data[i];
  int ndata = map.size();
  varData = new SymTensor*[varNumStates];
  for( int i=0; i<varNumStates; i++ ) 
    varData[i] = new SymTensor [ndata];
  SymTensor* initialState = varData[0];
  for( int i=0; i<ndata; i++){
    int imp = map[i];
    initialState[i] = SymTensor( varPntr[0][imp], varPntr[1][imp], varPntr[2][imp],
                            varPntr[3][imp], varPntr[4][imp], varPntr[5][imp] );
  }
}

ExternalSymTensor::ExternalSymTensor( Real** data, string name, int numData,
                                DataCentering cntr, int numStates, bool plot )
                              : AbstractData( numData, name, cntr, SymTensorType, plot )
{
  varExternal = true;
  varNumStates = numStates;
  varMap = NULL;
  for(int i=0; i<6; i++) varPntr[i] = data[i];
  varData = new SymTensor*[varNumStates];
  for( int i=0; i<varNumStates; i++ ) 
    varData[i] = new SymTensor [varLength];
  SymTensor* initialState = varData[0];
  for( int i=0; i<varLength; i++){
    initialState[i] = SymTensor( varPntr[0][i], varPntr[1][i], varPntr[2][i],
                            varPntr[3][i], varPntr[4][i], varPntr[5][i] );
  }
}

void ExternalSymTensor::advanceState(){
  if( varNumStates == 1 ) return;
  if( varNumStates == 2 ) {
    memcpy( varData[CURRENT], varData[UPDATED], varLength*sizeof(SymTensor) );
  } else {
    SymTensor* back = varData[varNumStates-1];
    for( int i=3; i<varNumStates; i++){
      varData[i] = varData[i-1];
    }
    varData[2] = varData[CURRENT];
    varData[CURRENT] = varData[UPDATED];
    varData[UPDATED] = back;
    memcpy(varData[UPDATED], varData[CURRENT], varLength * sizeof(SymTensor));
    }
}

void ExternalSymTensor::getData(SymTensor*& data, StateIndex state)
{
    data = varData[state];
}
void ExternalSymTensor::getData(int i, SymTensor*& data, StateIndex state)
{
    data = varData[state] + i;
}
void ExternalSymTensor::getData(int i, SymTensor& data, StateIndex state)
{
    data = varData[state][i];
}
void ExternalSymTensor::setData(int i, SymTensor data, StateIndex state)
{
    varData[state][i] = data;
}

ExternalSymTensor::~ExternalSymTensor()
{
  if( varMap ){
    vector<int>& map = *varMap;
    int ndata = map.size();
    SymTensor* initialState = varData[0];
    for(int i=0; i<ndata; i++){
      int imp = map[i];
      Real* data; initialState[i].extractData( data );
      for(int j=0; j<6; j++) varPntr[j][imp] = data[j];
    }
  } else {
    SymTensor* initialState = varData[0];
    for(int i=0; i<varLength; i++){
      Real* data; initialState[i].extractData( data );
      for(int j=0; j<6; j++) varPntr[j][i] = data[j];
    }
  }
  for(int i=0; i<varNumStates; i++)
    delete [] varData[i];
  delete [] varData;
}
/******************************************************************************/

DataContainer::DataContainer( )
{
  myName = "UNSET";
  numnodes = 0;
  numelements = 0;
  nummatpoints = 0;
  setUp();
}

DataContainer::DataContainer( 
                              int nnodes,
                              int nelements,
                              int nmatpoints )
{
  myName = "UNSET";
  numnodes = nnodes;
  numelements = nelements;
  nummatpoints = nmatpoints;
  setUp();
}

DataContainer::DataContainer( string myname )
{
  myName = myname;
  numnodes = 0;
  numelements = 0;
  nummatpoints = 0;
  setUp();
}

DataContainer::~DataContainer()
{
  deleteAll();
}

void
DataContainer::dumpAll()
{
  pXcout << "\n\n";
  pXcout << "*****************************************************************" << endl;
  pXcout << "\n\n DataContainer Report... \n" << endl;

  int numVars = dataEntities.size();
  for( int i=0; i<numVars; i++)
    dataEntities[i]->dump();

  pXcout << "\n\n";
  pXcout << "*****************************************************************";
  pXcout << "\n" << endl;

}


void AbstractData::dump(){
  pXcout << "\tlength (# members): \t"  << varLength  << endl;
  pXcout << "\tname: \t\t"        << varName  << endl;
  pXcout << "\tcentering: \t"     << varCntr << endl;
  pXcout << "\texternal: \t"      << varExternal << endl;
  pXcout << "\tplottable: \t"     << varPlot << endl;
}

int
DataContainer::getCentering(VarIndex var_index)
{
  return dataEntities[var_index]->getCentering();
}

void DataContainer::setUp() { }

/******************************************************************************/
void DataContainer::advanceState(){
/******************************************************************************/
  int numData = dataEntities.size();
  for( int index=0; index<numData; index++ ) {
    AbstractData*& ad = dataEntities[index];
    ad->advanceState();
  }
}

/******************************************************************************/
void DataContainer::advanceState(VarIndex varindex){
/******************************************************************************/
  int ndata = dataEntities.size();
  if( (varindex >= 0) && (varindex < ndata) ) {
    AbstractData*& ad = dataEntities[varindex];
    ad->advanceState();
  }
}
 
bool DataContainer::variableExists( string name, VarIndex& varIndex )
{
  // does a variable by this name already exist?
  int numData = dataEntities.size();
  for( int index=0; index<numData; index++ ) {
    AbstractData*& ad = dataEntities[index];
    if( ad->getName() == name){
      // variable with this name exists
      varIndex = index;
      return true;
    }
  }
  varIndex = UNSET_VAR_INDEX;
  return false;
}

VarIndex 
DataContainer::registerVariable( DataType datatype,
                                 Real** pntr,
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

    AbstractData* newdata;
    if( datatype == VectorType )
      newdata = new ExternalVector( pntr, name, map, centering, numStates, plot );
    else if( datatype == TensorType )
      newdata = new ExternalTensor( pntr, name, map, centering, numStates, plot );
    else if( datatype == SymTensorType )
      newdata = new ExternalSymTensor( pntr, name, map, centering, numStates, plot );
    else newdata = NULL;

    int index = dataEntities.size();
    dataEntities.push_back( newdata );
    return ( index );
  }
}

VarIndex 
DataContainer::registerVariable( DataType datatype,
                                 Real** pntr,
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

    AbstractData* newdata;
    if( datatype == VectorType )
      newdata = new ExternalVector( pntr, name, numData, centering, numStates, plot );
    else if( datatype == TensorType )
      newdata = new ExternalTensor( pntr, name, numData, centering, numStates, plot );
    else if( datatype == SymTensorType )
      newdata = new ExternalSymTensor( pntr, name, numData, centering, numStates, plot );
    else newdata = NULL;

    int index = dataEntities.size();
    dataEntities.push_back( newdata );
    return ( index );
  }
}


void DataContainer::getVariable( string name, VarIndex& varIndex )
{
  if( variableExists(name, varIndex) ){
    return;
  } else {
      pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
           << "\tVariable (" << name << ") doesn't exist" << endl;
      varIndex = UNSET_VAR_INDEX;
      return;
  }
}

VarIndex
DataContainer::registerVariable( DataType datatype,
                                 string name,
                                 DataCentering centering,
                                 bool plot,
                                 int numStates,
                                 bool varNew ) {
  VarIndex varIndex;
  if( variableExists(name, varIndex) ){
    if( varNew == true ){
      pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
           << "\tVariable (" << name << ") already exists" << endl;
      return UNSET_VAR_INDEX;
    }
    if( dataEntities[varIndex]->getCentering() != centering ){
      pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
           << "\tVariable (" << name << ") already exists, but has different attributes" << endl;
      return UNSET_VAR_INDEX;
    }
    return varIndex;

  } else {

    int num = 0;
    if(centering == NODE) num = numnodes;
    else if(centering == ELEM) num = numelements;
    else if(centering == MATPNT) num = nummatpoints;
    
    return addnew( num, name, centering, numStates, datatype, plot );
  }

}

VarIndex
DataContainer::registerVariable( DataType datatype,
                                 string name,
                                 DataCentering centering,
                                 int num,
                                 bool plot,
                                 int numStates,
                                 bool varNew ) {
  VarIndex varIndex;
  if( variableExists(name, varIndex) ){
    if( varNew == true ){
      pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
           << "\tVariable (" << name << ") already exists" << endl;
      return UNSET_VAR_INDEX;
    }
    if( dataEntities[varIndex]->getCentering() != centering ){
      pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
           << "\tVariable (" << name << ") already exists, but has different attributes" << endl;
      return UNSET_VAR_INDEX;
    }
    return varIndex;
  } else {
    if(num == 0){
      if(centering == NODE) num = numnodes;
      else if(centering == ELEM) num = numelements;
      else if(centering == MATPNT) num = nummatpoints;
    }
    
    return addnew( num, name, centering, numStates, datatype, plot );

  }
}

VarIndex
DataContainer::addnew( int num, string name, DataCentering centering, 
                       int numStates, DataType datatype, bool plot)
{
  AbstractData* newdata;
  if( datatype == IntType )
    newdata = new DataEntity< int >( num, name, centering, numStates, datatype, plot );
  else if( datatype == RealType )
    newdata = new DataEntity< Real >( num, name, centering, numStates, datatype, plot );
  else if( datatype == VectorType ) 
    newdata = new DataEntity< Vector >( num, name, centering, numStates, datatype, plot );
  else if( datatype == TensorType ) 
    newdata = new DataEntity< Tensor >( num, name, centering, numStates, datatype, plot );
  else if( datatype == SymTensorType ) 
    newdata = new DataEntity< SymTensor>( num, name, centering, numStates, datatype, plot );
  else newdata = NULL;

  int index = dataEntities.size();
  dataEntities.push_back( newdata );
  return ( index );

}

bool DataContainer::isPlotVariable( VarIndex varindex )
{
  int ndata = dataEntities.size();
  if( (varindex >= 0) && (varindex < ndata) ) {
    return (dataEntities[varindex]->isPlottable());
  }
  pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
       << "\tInvalid variable index " << varindex
       << "\tReturning false" << endl;
  
  return( false );
}

bool DataContainer::isNodalVariable( VarIndex varindex )
{
  int ndata = dataEntities.size();
  if( (varindex >= 0) && (varindex < ndata) ) {
    return (dataEntities[varindex]->getCentering() == NODE);
  }
  pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
       << "\tInvalid variable index " << varindex
       << "\tReturning false" << endl;
  
  return( false );
}

bool DataContainer::isElementVariable( VarIndex varindex )
{
  int ndata = dataEntities.size();
  if( (varindex >= 0) && (varindex < ndata) ) {
    return (dataEntities[varindex]->getCentering() == ELEM);
  }
  pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
       << "\tInvalid variable index " << varindex
       << "\tReturning false" << endl;
  
  return( false );
  
}

string DataContainer::getVariableName( VarIndex varindex )
{
  int ndata = dataEntities.size();
  if( (varindex >= 0) && (varindex < ndata) ) {
    return (dataEntities[varindex]->getName());
  }
  pXcout << "!!! WARNING: \n" << "\t" << __GIBLET_FUNCTION_NAMER__ << "\n"
       << "\tInvalid variable index " << varindex
       << "\tReturning NULL pointer" << endl;
  
  return( NULL );
  
}

void DataContainer::deleteAll(){
  int numVars = dataEntities.size();
  for( int i=0; i<numVars; i++) {
    AbstractData*& ad = dataEntities[i];
    if( ad ) delete ad;
  }
}
