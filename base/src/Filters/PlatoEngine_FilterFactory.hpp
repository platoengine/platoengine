#pragma once

#include <mpi.h>

class DataMesh;
namespace Plato
{
class AbstractFilter;
class InputData;

AbstractFilter* build_filter(Plato::InputData aInputData, MPI_Comm& aLocalComm, DataMesh* mesh);
}
