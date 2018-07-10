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
