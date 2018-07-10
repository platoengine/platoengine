#pragma once

#include <mpi.h>
#include <cstddef>

class DataMesh;
namespace Plato
{
class InputData;

class AbstractFilter
{
public:
    AbstractFilter();
    virtual ~AbstractFilter();

    virtual void build(InputData aInputData, MPI_Comm& aLocalComm, DataMesh* aMesh) = 0;
    virtual void apply_on_field(size_t length, double* field_data) = 0;
    virtual void apply_on_gradient(size_t length, double* gradient_data) = 0;

private:

};
}
