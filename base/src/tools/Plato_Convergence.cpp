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

#include "Plato_Convergence.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_Parser.hpp"

/******************************************************************************/
void
ConvergenceTest::initNorm( double f, double pnorm )
/******************************************************************************/
{
    m_test->initNorm( f, pnorm );
}

/******************************************************************************/
bool
ConvergenceTest::isConverged( double delta_f, double delta_p, int iter, int myPID )
/******************************************************************************/
{
    if(iter == 0) return false;

    bool writeToCout = false;
    if(myPID == 0) writeToCout = true;

    // check convergence based on user defined criteria
    if(writeToCout){
      std::cout << "************************************************************************" << std::endl;
      std::cout << "** Optimization Convergence Check **************************************" << std::endl;
    }

    bool converged = m_test->passed(iter, delta_f, delta_p, writeToCout);

    if(writeToCout){
      if(converged){
          std::cout << "Termination criteria met" << std::endl;
      } else
        std::cout << "Not converged." << std::endl;
      std::cout << "************************************************************************" << std::endl;
    }

    return converged;
}

/******************************************************************************/
std::vector<Teuchos::RCP<ConvergenceTest::ConTest>>
ConvergenceTest::parseTests(const Plato::InputData& node)
/******************************************************************************/
{

    std::vector<Teuchos::RCP<ConvergenceTest::ConTest>> conTests;
    for( auto test : node.getByName<Plato::InputData>("Test") )
    {

        if( test.size<Plato::InputData>("Test") )
        {
            auto testTest = test.get<Plato::InputData>("Test");
            conTests.push_back(Teuchos::rcp(new CompoundTest(test)));
        }
        else
        {

            std::string testType = Plato::Get::String(test, "Type",/*toUpper=*/true);
            if(testType == "MAXIMUM ITERATIONS")
            {
                double conValue = Plato::Get::Int(test, "Value");
                conTests.push_back(Teuchos::rcp(new MaxIter(conValue)));
            }
            if(testType == "MINIMUM ITERATIONS")
            {
                double conValue = Plato::Get::Int(test, "Value");
                conTests.push_back(Teuchos::rcp(new MinIter(conValue)));
            }
            if(testType == "RELATIVE TOPOLOGY CHANGE")
            {
                double conValue = Plato::Get::Double(test, "Value");
                conTests.push_back(Teuchos::rcp(new RelDeltaP(conValue)));
            }
            if(testType == "ABSOLUTE TOPOLOGY CHANGE")
            {
                double conValue = Plato::Get::Double(test, "Value");
                conTests.push_back(Teuchos::rcp(new AbsDeltaP(conValue)));
            }
            if(testType == "RELATIVE OBJECTIVE CHANGE")
            {
                double conValue = Plato::Get::Double(test, "Value");
                conTests.push_back(Teuchos::rcp(new RelDeltaF(conValue)));
            }
            if(testType == "ABSOLUTE OBJECTIVE CHANGE")
            {
                double conValue = Plato::Get::Double(test, "Value");
                conTests.push_back(Teuchos::rcp(new AbsDeltaF(conValue)));
            }
            if(testType == "RELATIVE OBJECTIVE RUNNING AVERAGE CHANGE")
            {
                double conValue = Plato::Get::Double(test, "Value");
                conTests.push_back(Teuchos::rcp(new RelRunningDF(conValue)));
            }
            if(testType == "ABSOLUTE OBJECTIVE RUNNING AVERAGE CHANGE")
            {
                double conValue = Plato::Get::Double(test, "Value");
                conTests.push_back(Teuchos::rcp(new AbsRunningDF(conValue)));
            }
        }
    }

    return conTests;
}

/******************************************************************************/
void ConvergenceTest::CompoundTest::initNorm( double f0, double p0)
/******************************************************************************/
{
  for(auto test : tests) test->initNorm(f0, p0);
}

/******************************************************************************/
bool ConvergenceTest::CompoundTest::passed(int iter, double delta_f, double delta_p, bool write)
/******************************************************************************/
{

  if( comboType == AND ){
    bool passed = true;
    for(auto test : tests){
      bool test_passed = test->passed(iter, delta_f, delta_p, write);
      passed = passed && test_passed;
    }
    return passed;
  } else 
  if( comboType == OR ){
    bool passed = false;
    for(auto test : tests){
      bool test_passed = test->passed(iter, delta_f, delta_p, write);
      passed = passed || test_passed;
    }
    return passed;
  } else {
    // TODo throw
    return false;
  }

  
}

/******************************************************************************/
ConvergenceTest::CompoundTest::CompoundTest(const Plato::InputData& checkerNode)
/******************************************************************************/
{
  tests = parseTests(checkerNode);

  std::string cbType = Plato::Get::String(checkerNode,"ComboType",/*toUpper=*/true);

  if(cbType == "OR"){
   comboType = OR;
  } else
  if(cbType == "AND"){
   comboType = AND;
  } else {
    std::stringstream ss;
    ss << "Plato::ConvergenceTest: invalid ComboType '" << cbType << "'. Choose from {'OR','AND'}.";
    throw Plato::ParsingException(ss.str());
  }
}

/******************************************************************************/
ConvergenceTest::ConvergenceTest(const Plato::InputData& checkerNode)
/******************************************************************************/
{
  m_test = Teuchos::rcp(new CompoundTest(checkerNode));
}

/******************************************************************************/
bool ConvergenceTest::MinIter::passed(int iter, double delta_f, double delta_p, bool write)
{ 
  bool status = ( iter < m_minIter );
  if( write )
    std::cout << "Test: Minimum iterations: " << std::endl 
    << "     numIter = " << iter << " < " << m_minIter << ": " 
    << (status ? "true" : "false") << std::endl;
  return status;
}
bool ConvergenceTest::MaxIter::passed(int iter, double delta_f, double delta_p, bool write)
{ 
  bool status = ( iter > m_maxIter );
  if( write )
    std::cout << "Test: Maximum iterations: " << std::endl 
    << "     numIter = " << iter << " > " << m_maxIter << ": " 
    << (status ? "true" : "false") << std::endl;
  return status;
}
bool ConvergenceTest::AbsDeltaP::passed(int iter, double delta_f, double delta_p, bool write)
{ 
  bool status = ( fabs(delta_p) < m_conValue );
  if( write )
    std::cout << "Test: Topology Change (Absolute): " << std::endl 
    << "     abs(dp) = " << fabs(delta_p) << " < " << m_conValue << ": " 
    << (status ? "true" : "false") << std::endl;
  return status;
}
bool ConvergenceTest::AbsDeltaF::passed(int iter, double delta_f, double delta_p, bool write)
{
  bool status = ( fabs(delta_f) < m_conValue );
  if( write )
    std::cout << "Test: Objective Change (Absolute): " << std::endl 
    << "     abs(df) = " << fabs(delta_f) << " < " << m_conValue << ": " 
    << (status ? "true" : "false") << std::endl;
  return status;
}
bool ConvergenceTest::AbsRunningDF::passed(int iter, double delta_f, double delta_p, bool write)
{
  dF.push_back(delta_f);
  runningDF += delta_f;
  if(dF.size()>m_nave) runningDF -= *(dF.end()-m_nave);
  bool status = ( runningDF < m_conValue );
  if( write )
    std::cout << "Test: Objective Change Running Average (Absolute): " << std::endl 
    << "     abs(<df>) = " << runningDF << " < " << m_conValue << ": " 
    << (status ? "true" : "false") << std::endl;
  return status;
}
bool ConvergenceTest::RelDeltaP::passed(int iter, double delta_f, double delta_p, bool write){
  bool status = (p0 != 0.0) ? ( fabs(delta_p/p0) < m_conValue ) : false;
  if( write )
    std::cout << "Test: Topology Change (Relative): " << std::endl 
    << "     abs(dp) = " << fabs(delta_p) << ", fabs(dp/p0) = " << fabs(delta_p/p0) << " < " << m_conValue 
    << ": " << (status ? "true" : "false") << std::endl;
  return status;
}
bool ConvergenceTest::RelDeltaF::passed(int iter, double delta_f, double delta_p, bool write){
  bool status = (f0 != 0.0) ? ( fabs(delta_f/f0) < m_conValue ) : false;
  if( write )
    std::cout << "Test: Objective Change (Relative): " << std::endl 
    << "     abs(df) = " << fabs(delta_f) << ", fabs(df/f0) = " << fabs(delta_f/f0) << " < " << m_conValue 
    << ": " << (status ? "true" : "false") << std::endl;
  return status;
}
bool ConvergenceTest::RelRunningDF::passed(int iter, double delta_f, double delta_p, bool write){
  dF.push_back(delta_f);
  runningDF += delta_f;
  unsigned int nvals = dF.size();
  unsigned int lastVal = nvals-1;
  if(nvals>m_nave){
     runningDF -= dF[lastVal-m_nave];
     nvals = m_nave;
   }
  bool status = (f0 != 0.0) ? ( fabs(runningDF/f0)/nvals < m_conValue ) : false;
  if( write )
    std::cout << "Test: Objective Change Running Average (Relative): " << std::endl 
    << "     abs(<df>) = " << fabs(runningDF)/nvals << ", fabs(<df/f0>) = " << fabs(runningDF/f0)/nvals << " < " << m_conValue 
    << ": " << (status ? "true" : "false") << std::endl;
  return status;
}
/******************************************************************************/

