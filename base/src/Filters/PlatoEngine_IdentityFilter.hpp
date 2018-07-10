#pragma once

#include "PlatoEngine_AbstractFilter.hpp"
#include "Plato_InputData.hpp"
#include <cstddef>

class DataMesh;
namespace Plato
{

class IdentityFilter : public AbstractFilter
{
public:
    IdentityFilter();
    virtual ~IdentityFilter();

    virtual void build(InputData aInputData, MPI_Comm& aLocalComm, DataMesh* aMesh);
    virtual void apply_on_field(size_t length, double* field_data);
    virtual void apply_on_gradient(size_t length, double* gradient_data);

protected:
};

}
