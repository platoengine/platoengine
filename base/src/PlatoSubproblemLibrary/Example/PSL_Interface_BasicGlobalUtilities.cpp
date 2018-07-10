// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_BasicGlobalUtilities.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"

#include <iostream>
#include <ostream>
#include <stdlib.h>
#include <string>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_BasicGlobalUtilities::Interface_BasicGlobalUtilities() :
        AbstractInterface::GlobalUtilities(),
        m_never_abort(false),
        m_ignored_abort(false)
{
}

Interface_BasicGlobalUtilities::~Interface_BasicGlobalUtilities()
{
}

void Interface_BasicGlobalUtilities::print(const std::string& message)
{
    std::cout << message;
}

void Interface_BasicGlobalUtilities::fatal_error(const std::string& message)
{
    print(message);
    // flush
    std::flush(std::cout);

    // should abort?
    if(m_never_abort)
    {
        m_ignored_abort = true;
        return;
    }
    else
    {
        // do abort
        abort();
    }
}

void Interface_BasicGlobalUtilities::set_never_abort(bool never_abort)
{
    m_never_abort = never_abort;
}

bool Interface_BasicGlobalUtilities::get_ignored_abort() const
{
    return m_ignored_abort;
}

void Interface_BasicGlobalUtilities::reset_ignored_abort()
{
    m_ignored_abort = false;
}

}
}
