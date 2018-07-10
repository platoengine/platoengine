#ifndef _COMMUNICATOR_H_
#define _COMMUNICATOR_H_

#include "types.hpp"
#include "math_library.hpp"
#include <string>
#include <vector>
#include <iostream>

#define pXcout std::cout << "P" << WorldComm.getPID() << ": "
#define pXcerr std::cerr << "P" << WorldComm.getPID() << ": "
#define p0cout if (WorldComm.getPID() == 0) std::cout
#define p0cerr if (WorldComm.getPID() == 0) std::cerr

#define AXSIS_IO_PROC 0

using std::string;
using std::vector;

class DataMesh;

class Communicator
{
public:
  Communicator();
  virtual ~Communicator();

  void init();                           //! Initialize communicator data
 
  void init(MPI_Comm comm, 
            bool preConstructed=false);  //! Initialize communicator data
 
  void cartCreate();                     //! create cartesian communicator

  void dumpComm();                       //! Code to dump communicator info

  void addVectorType();

  int getPID() { return myPID; };        //! set proc id
//  int getProcDomain(int dim){ if( dim>=0 && dim<3 ) return procDomain[dim]; else return 0;}
//  int getCartCoords(int dim){ if( dim>=0 && dim<3 ) return myIndices[dim]; else return 0; }
  void getElementDomain( int dim, int numElem, int& ibegin, int& iend);
  void setElementDomain( int dim, int& ibegin, int& iend);

  MPI_Comm getComm() { return myComm; }

  int globalSum(int value);            //! global sum for ints

  Real globalSum(Real value);           //! global sum for reals

  void globalArraySum(Real* values, int len ); 

  int globalMax(int *value, int size);  //! global max for ints
  int globalMax(int value); //! global max for int
  int globalMin(int value); //! global max for int

  Real globalMax(Real *value, int size); //! global max for reals
  Real globalMax(Real value); //! global max for reals

  Real globalMin(Real value); //! global max for reals

  int Broadcast(int &value);
  int Broadcast(Real &value);
  int Broadcast(Real *value, int size);
  int Broadcast(Vector *value, int size);

  int BroadcastFromRank(int fromRank, Real *data, int numData);

  int Import(vector<int>*, DataMesh*);
  int Import(vector<double>*, DataMesh*);

  void sync();                          //! synchronize procs

  void blockingRecv(int srce, int* data, int size=1, int tag=0);
  void blockingSend(int dest, int* data, int size=1, int tag=0);
  void pAcout(const char*);
  void BeginProcStaging();
  void EndProcStaging();
  void BeginBlockForIO();
  void EndBlockForIO();
  int  GetPID() const { return myPID; }
  int  GetSize() const { return mySize; }

public:
  const int rootPID;

private:
  MPI_Comm myComm;
  int      mySize; 
  int      myPID;
 
  // cartesian decomp data
  bool     preConstructed;
  int      procDomain[3];
  int     *myIndices;
  int      begin[3], end[3];

private:
  void setComm();
  void setComm(MPI_Comm comm);
  void setSize();
  void setPID();

private:
  Communicator(const Communicator&);
  Communicator& operator=(const Communicator&);
};

extern Communicator WorldComm;

#endif //_COMMUNICATOR_H_
