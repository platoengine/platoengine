/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

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
