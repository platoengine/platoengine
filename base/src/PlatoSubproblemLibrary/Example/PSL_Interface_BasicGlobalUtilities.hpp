// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_GlobalUtilities.hpp"

#include <string>

namespace PlatoSubproblemLibrary
{
namespace example
{

class Interface_BasicGlobalUtilities : public AbstractInterface::GlobalUtilities
{
public:

    Interface_BasicGlobalUtilities();
    virtual ~Interface_BasicGlobalUtilities();

    virtual void print(const std::string& message);
    virtual void fatal_error(const std::string& message);

    void set_never_abort(bool never_abort);
    bool get_ignored_abort() const;
    void reset_ignored_abort();

protected:

    bool m_never_abort;
    bool m_ignored_abort;
};

}
}
