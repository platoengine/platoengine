// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_Kernel_StructParameterDataBuilder.hpp"

#include "PSL_Implementation_Kernel_StructParameterData.hpp"
#include "PSL_Abstract_ParameterDataBuilder.hpp"
#include "PSL_ParameterData.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_Kernel_StructParameterDataBuilder::Interface_Kernel_StructParameterDataBuilder() :
        AbstractInterface::ParameterDataBuilder(),
        m_data(NULL)
{
}

Interface_Kernel_StructParameterDataBuilder::~Interface_Kernel_StructParameterDataBuilder()
{
    m_data = NULL;
}

void Interface_Kernel_StructParameterDataBuilder::set_data(example::Kernel_StructParameterData* data)
{
    m_data = data;
}

#define PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(_input_data, _output_data, _field) \
    if(_input_data->has_##_field) { \
        _output_data->set_##_field(_input_data->_field); \
    }

ParameterData* Interface_Kernel_StructParameterDataBuilder::build()
{
    if (m_data == NULL) {
        return NULL;
    }

    // allocate
    ParameterData* result = new ParameterData;

    // fill
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, scale)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, absolute)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, iterations)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, symPlane_X)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, symPlane_Y)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, symPlane_Z)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, penalty)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, node_resolution_tolerance)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, spatial_searcher)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, normalization)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, reproduction)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, matrix_assembly_agent)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, symmetry_plane_agent)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, mesh_scale_agent)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, point_ghosting_agent)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, bounded_support_function)

    return result;
}

}
}
