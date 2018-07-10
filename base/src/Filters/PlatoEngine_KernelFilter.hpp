#pragma once

#include <mpi.h>
#include <cstddef>
#include "PlatoEngine_AbstractFilter.hpp"

namespace PlatoSubproblemLibrary
{
class KernelFilter;
class ParameterData;
class AbstractAuthority;
namespace AbstractInterface
{
class PointCloud;
class ParallelExchanger;
}
}
class DataMesh;

namespace Plato
{
class InputData;

class KernelFilter : public AbstractFilter
{
public:
    KernelFilter();
    virtual ~KernelFilter();

    virtual void build(InputData aInputData, MPI_Comm& aLocalComm, DataMesh* aMesh);
    virtual void apply_on_field(size_t length, double* field_data);
    virtual void apply_on_gradient(size_t length, double* gradient_data);

private:

    void build_input_data(InputData interface);
    void build_points(DataMesh* mesh);
    void build_parallel_exchanger(DataMesh* mesh);

    MPI_Comm m_comm;
    PlatoSubproblemLibrary::KernelFilter* m_kernel;
    PlatoSubproblemLibrary::AbstractAuthority* m_authority;
    PlatoSubproblemLibrary::ParameterData* m_input_data;
    PlatoSubproblemLibrary::AbstractInterface::PointCloud* m_points;
    PlatoSubproblemLibrary::AbstractInterface::ParallelExchanger* m_parallel_exchanger;
    double m_maximum_absolute_parallel_error_tolerance;
    bool m_validate_interface;

};

}
