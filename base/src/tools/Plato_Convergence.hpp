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

#ifndef PLATO_CONVERGENCE_TEST
#define PLATO_CONVERGENCE_TEST

#include <Teuchos_ParameterList.hpp>
#include "Plato_InputData.hpp"
#include "Plato_Parser.hpp"
#include <string>
#include <vector>

class ConvergenceTest {
  public:
    ConvergenceTest(const Plato::InputData& checkerNode);

    bool isConverged( double delta_f, double delta_p, int iter, int myPID = -1);
    void initNorm(double f, double p);

  private:
  
    enum ComboType {AND, OR};

    class ConTest {
      public: 
        virtual ~ConTest(){}
        virtual bool passed(int iter, double delta_f, double delta_p, bool write) = 0;
        virtual void initNorm(double f0, double p0){}
    };

    static std::vector<Teuchos::RCP<ConTest>> parseTests(const Plato::InputData& node);

    class CompoundTest : public ConTest {
      public:
        CompoundTest(const Plato::InputData& node);
        bool passed(int iter, double delta_f, double delta_p, bool write);
        void initNorm(double f0, double p0);
      private:
        std::vector<Teuchos::RCP<ConTest>> tests;
        ComboType comboType;
    };
    Teuchos::RCP<CompoundTest> m_test;

    class MinIter : public ConTest {
      public:
        MinIter(int val) : m_minIter(val){}
        bool passed(int iter, double delta_f, double delta_p, bool write);
      private:
        int m_minIter;
    };
    class MaxIter : public ConTest {
      public:
        MaxIter(int val) : m_maxIter(val){}
        bool passed(int iter, double delta_f, double delta_p, bool write);
      private:
        int m_maxIter;
    };
    class AbsDeltaP : public ConTest {
      public:
        AbsDeltaP(double val) : m_conValue(val){}
        bool passed(int iter, double delta_f, double delta_p, bool write);
      private:
        double m_conValue;
    };
    class RelDeltaP : public ConTest {
      public:
        RelDeltaP(double val) : m_conValue(val),p0(0.0){};
        bool passed(int iter, double delta_f, double delta_p, bool write);
        void initNorm(double f, double p){p0 = p;}
      private:
        double m_conValue;
        double p0;
    };
    class AbsDeltaF : public ConTest {
      public:
        AbsDeltaF(double val) : m_conValue(val){}
        bool passed(int iter, double delta_f, double delta_p, bool write);
      private:
        double m_conValue;
    };
    class RelDeltaF : public ConTest {
      public:
        RelDeltaF(double val) : m_conValue(val),f0(0.0){};
        bool passed(int iter, double delta_f, double delta_p, bool write);
        void initNorm(double f, double p){f0 = f;}
      private:
        double m_conValue;
        double f0;
    };
    class AbsRunningDF : public ConTest {
      public:
        AbsRunningDF(double val) : m_conValue(val),runningDF(0.0), m_nave(10){}
        bool passed(int iter, double delta_f, double delta_p, bool write);
      private:
        double m_conValue;
        std::vector<double> dF;
        double runningDF;
        unsigned int m_nave;
    };
    class RelRunningDF : public ConTest {
      public:
        RelRunningDF(double val) : m_conValue(val),f0(0.0),runningDF(0.0),m_nave(10){};
        bool passed(int iter, double delta_f, double delta_p, bool write);
        void initNorm(double f, double p){f0 = f;}
      private:
        double m_conValue;
        double f0;
        std::vector<double> dF;
        double runningDF;
        unsigned int m_nave;
    };
};

#endif
