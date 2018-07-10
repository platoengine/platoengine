#include "PlatoEngine_IdentityFilter.hpp"

namespace Plato
{

IdentityFilter::IdentityFilter() :
        AbstractFilter()
{
}
IdentityFilter::~IdentityFilter()
{
}

void IdentityFilter::build(InputData aInputData, MPI_Comm& aLocalComm, DataMesh* aMesh)
{
}
void IdentityFilter::apply_on_field(size_t length, double* field_data)
{
}
void IdentityFilter::apply_on_gradient(size_t length, double* gradient_data)
{
}

}

