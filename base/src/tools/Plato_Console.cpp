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

/*
 * Plato_Console.cpp
 *
 *  Created on: Nov 7, 2019
 *
 */

#include "Plato_Console.hpp"
#include "Plato_Parser.hpp"

#include <sstream>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

namespace Plato
{

std::streambuf* Console::mStreamBufferCout = nullptr;
std::fstream*   Console::mConsoleFile = nullptr;
bool Console::mRedirectable = false;
bool Console::mVerbose = false;
int Console::mMyRank = -1;
int Console::m_stdout_fd = 0;
int Console::m_stderr_fd = 0;
int Console::m_redir_fd = 0;

void Console::Alert(std::string aAlertMessage)
{
   if (mMyRank == 0)
   {
      restore();
      std::cout << aAlertMessage << std::endl;
      redirect();
   }
}

void Console::Status(std::string aStatusMessage)
{
   if (mVerbose) {
     Alert(aStatusMessage);
   }
}


Console::Console(const std::string & aPerformerName, int aPerformerID, InputData aInputData, MPI_Comm& aLocalComm):
  mPerformerName ( aPerformerName ),
  mPerformerID   ( aPerformerID   )
{
   MPI_Comm_rank(aLocalComm, &mMyRank);
   mVerbose = Get::Bool(aInputData, "Verbose", false);

   if( mStreamBufferCout == nullptr )
   {
      mStreamBufferCout = std::cout.rdbuf();
   }

   if( mConsoleFile == nullptr )
   {
      std::stringstream tNameStream;
      tNameStream << mPerformerName << "_" << mPerformerID;

      mConsoleFile = new std::fstream;
      mConsoleFile->open(tNameStream.str(), std::ios::out);
      redirect_cout();

      m_redir_fd = open(tNameStream.str().c_str(), O_WRONLY);
      m_stdout_fd = dup(STDOUT_FILENO);
      m_stderr_fd = dup(STDERR_FILENO);
      redirect_printf();

      mRedirectable = true;
   }
}

Console::~Console()
{
   restore();

   if (mConsoleFile != nullptr)
   {
      mConsoleFile->close();
      delete mConsoleFile;
   }

   if (mRedirectable )
   {
      close(m_redir_fd);
      close(m_stdout_fd);
      close(m_stderr_fd);
   }
}

void Console::restore()
{
   if (mRedirectable)
   {
      restore_cout();
      restore_printf();
   }
}

void Console::redirect()
{
   if (mRedirectable)
   {
      redirect_cout();
      redirect_printf();
   }
}

void Console::redirect_printf()
{
   fflush(stdout);
   fflush(stderr);
   dup2(m_redir_fd, STDOUT_FILENO);
   dup2(m_redir_fd, STDERR_FILENO);
}

void Console::redirect_cout()
{

   if (mConsoleFile != nullptr)
   {
     std::flush(std::cout);
     std::cout.rdbuf(mConsoleFile->rdbuf());
   }
}

void Console::restore_cout()
{
   if (mStreamBufferCout != nullptr)
   {
     std::flush(std::cout);
     std::cout.rdbuf(mStreamBufferCout);
   }
}

void Console::restore_printf()
{
   fflush(stdout);
   fflush(stderr);
   dup2(m_stdout_fd, STDOUT_FILENO);
   dup2(m_stderr_fd, STDERR_FILENO);
}

}
