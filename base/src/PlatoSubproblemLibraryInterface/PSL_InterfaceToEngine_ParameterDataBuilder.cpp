#include "PSL_InterfaceToEngine_ParameterDataBuilder.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_ParameterData.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Parser.hpp"

namespace Plato
{

InterfaceToEngine_ParameterDataBuilder::InterfaceToEngine_ParameterDataBuilder(InputData aInputData) :
        PlatoSubproblemLibrary::AbstractInterface::ParameterDataBuilder(),
        m_inputData(aInputData)
{
}

InterfaceToEngine_ParameterDataBuilder::~InterfaceToEngine_ParameterDataBuilder()
{
}

PlatoSubproblemLibrary::ParameterData* InterfaceToEngine_ParameterDataBuilder::build()
{
    // allocate result
    PlatoSubproblemLibrary::ParameterData* result = new PlatoSubproblemLibrary::ParameterData;

    // set scale and/or absolute
    double absolute=-1.0;
    double scale=-1.0;
    double power=-1.0;
    if( m_inputData.size<Plato::InputData>("Filter") )
    {
        auto tFilterNode = m_inputData.get<Plato::InputData>("Filter");
        if(tFilterNode.size<std::string>("Absolute") > 0) {
            absolute = Plato::Get::Double(tFilterNode, "Absolute");
        }
        if(tFilterNode.size<std::string>("Scale") > 0) {
            scale = Plato::Get::Double(tFilterNode, "Scale");
        }
        if(tFilterNode.size<std::string>("Power") > 0) {
            power = Plato::Get::Double(tFilterNode, "Power");
        }
    }
    const bool meaningful_absolute = (absolute >= 0.);
    const bool meaningful_scale = (scale >= 0.);
    const bool meaningful_power = (power > 0);
    if(meaningful_absolute)
    {
        result->set_absolute(absolute);
    }
    if(meaningful_scale)
    {
        result->set_scale(scale);
    }
    if(!meaningful_absolute && !meaningful_scale)
    {
        const double default_scale = 3.5;
        result->set_scale(default_scale);
    }
    if(meaningful_power)
    {
        result->set_penalty(power);
    }
    else
    {
        result->set_penalty(1.0);
    }
    result->set_iterations(1);

    // defaults
    result->set_spatial_searcher(PlatoSubproblemLibrary::spatial_searcher_t::spatial_searcher_t::recommended);
    result->set_normalization(PlatoSubproblemLibrary::normalization_t::normalization_t::classical_row_normalization);
    result->set_reproduction(PlatoSubproblemLibrary::reproduction_level_t::reproduction_level_t::reproduce_constant);
    result->set_symmetry_plane_agent(PlatoSubproblemLibrary::symmetry_plane_agent_t::by_narrow_clone);
    result->set_matrix_assembly_agent(PlatoSubproblemLibrary::matrix_assembly_agent_t::by_row);
    result->set_mesh_scale_agent(PlatoSubproblemLibrary::mesh_scale_agent_t::by_average_optimized_element_side);
    result->set_matrix_normalization_agent(PlatoSubproblemLibrary::matrix_normalization_agent_t::default_agent);
    result->set_point_ghosting_agent(PlatoSubproblemLibrary::point_ghosting_agent_t::by_narrow_share);
    result->set_bounded_support_function(PlatoSubproblemLibrary::bounded_support_function_t::polynomial_tent_function);

    return result;
}

}

