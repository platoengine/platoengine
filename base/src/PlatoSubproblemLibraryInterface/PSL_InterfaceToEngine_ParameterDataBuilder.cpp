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

    // get scale and/or absolute
    double absolute=-1.0;
    double scale=-1.0;
    double power=-1.0;
    double heaviside_min=-1.;
    double heaviside_update=-1.;
    double heaviside_max=-1;

    if( m_inputData.size<Plato::InputData>("Filter") )
    {
        auto tFilterNode = m_inputData.get<Plato::InputData>("Filter");
        if(tFilterNode.size<std::string>("Absolute") > 0)
        {
            absolute = Plato::Get::Double(tFilterNode, "Absolute");
        }
        if(tFilterNode.size<std::string>("Scale") > 0)
        {
            scale = Plato::Get::Double(tFilterNode, "Scale");
        }
        if(tFilterNode.size<std::string>("Power") > 0)
        {
            power = Plato::Get::Double(tFilterNode, "Power");
        }
        if(tFilterNode.size<std::string>("HeavisideMin") > 0)
        {
            heaviside_min = Plato::Get::Double(tFilterNode, "HeavisideMin");
        }
        if(tFilterNode.size<std::string>("HeavisideUpdate") > 0)
        {
            heaviside_update = Plato::Get::Double(tFilterNode, "HeavisideUpdate");
        }
        if(tFilterNode.size<std::string>("HeavisideMax") > 0)
        {
            heaviside_max = Plato::Get::Double(tFilterNode, "HeavisideMax");
        }
        if(tFilterNode.size<std::string>("SmoothMaxPNorm") > 0)
        {
            result->set_smooth_max_p_norm(Plato::Get::Double(tFilterNode, "SmoothMaxPNorm"));
        }
        if(tFilterNode.size<std::string>("CriticalPrintAngle") > 0)
        {
            result->set_critical_print_angle(Plato::Get::Double(tFilterNode, "CriticalPrintAngle"));
        }
        if(tFilterNode.size<std::string>("BaseLayerSideSetName") > 0)
        {
            result->set_base_layer_side_set_name(Plato::Get::String(tFilterNode, "BaseLayerSideSetName"));
        }
        if(tFilterNode.size<std::string>("BuildDirectionX") > 0)
        {
            result->set_build_direction_x(Plato::Get::Double(tFilterNode, "BuildDirectionX"));
        }
        if(tFilterNode.size<std::string>("BuildDirectionY") > 0)
        {
            result->set_build_direction_y(Plato::Get::Double(tFilterNode, "BuildDirectionY"));
        }
        if(tFilterNode.size<std::string>("BuildDirectionZ") > 0)
        {
            result->set_build_direction_z(Plato::Get::Double(tFilterNode, "BuildDirectionZ"));
        }

    }

    // decide if input was meaningful
    const bool meaningful_absolute = (absolute >= 0.);
    const bool meaningful_scale = (scale >= 0.);
    const bool meaningful_power = (power > 0);
    const bool meaningful_HeavisideMin = (heaviside_min > 0);
    const bool meaningful_HeavisideUpdate = (heaviside_update >= 0);
    const bool meaningful_HeavisideMax = (heaviside_max > 0);

    // if not meaningful, use defaults
    // TODO: This is bad behavior, should either throw an error if the input is not meaningful
    // or do nothing and let the algorithm that uses the data worry about error checking
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
    if(meaningful_HeavisideMin)
    {
        result->set_min_heaviside_parameter(heaviside_min);
    }
    else
    {
        result->set_min_heaviside_parameter(3.);
    }
    if(meaningful_HeavisideUpdate)
    {
        result->set_heaviside_continuation_scale(heaviside_update);
    }
    else
    {
        result->set_heaviside_continuation_scale(.75);
    }
    if(meaningful_HeavisideMax)
    {
        result->set_max_heaviside_parameter(heaviside_max);
    }
    else
    {
        result->set_max_heaviside_parameter(4.2);
    }

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

