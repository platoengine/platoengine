/*
 * Plato_OperationsUtilities.cpp
 *
 *  Created on: Jun 27, 2019
 */

#include "Plato_Parser.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_OperationsUtilities.hpp"

namespace Plato
{

Plato::data::layout_t getLayout(const std::string & aLayoutStr)
{

    Plato::data::layout_t tLayout;
    if(aLayoutStr == "Nodal Field")
    {
        tLayout = Plato::data::layout_t::SCALAR_FIELD;
    }
    else if(aLayoutStr == "Element Field")
    {
        tLayout = Plato::data::layout_t::ELEMENT_FIELD;
    }
    else if(aLayoutStr == "Value" || aLayoutStr == "Global" || aLayoutStr == "Scalar")
    {
        tLayout = Plato::data::layout_t::SCALAR;
    }
    else
    {
        std::stringstream tError;
        tError << std::endl << " PlatoApp: " << std::endl;
        tError << "   Unknown layout specified: '" << aLayoutStr << "'" << std::endl;
        Plato::ParsingException tParsingException(tError.str());
        throw tParsingException;
    }
    return (tLayout);
}

Plato::data::layout_t getLayout(Plato::InputData& aNode, Plato::data::layout_t aDefaultLayout)
{
    auto tLayoutString = Plato::Get::String(aNode, "Layout");
    Plato::data::layout_t tLayout = aDefaultLayout;
    if(!tLayoutString.empty())
    {
        tLayout = Plato::getLayout(tLayoutString);
    }
    return (tLayout);
}

Plato::data::layout_t getLayout(Plato::InputData& aNode)
{
    auto tLayoutString = Plato::Get::String(aNode, "Layout");
    if(tLayoutString.empty())
    {
        Plato::ParsingException tParsingException(" PlatoApp: required 'Layout' not specified");
        throw tParsingException;
    }
    return (Plato::getLayout(tLayoutString));
}

}
// namespace Plato
