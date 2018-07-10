/*
 * NLopt_Optimizer.hpp
 *
 *  Created on: April 19, 2017
 *
 */

#include <nlopt.h>
#include "Plato_Optimizer.hpp"

namespace Plato {

//!  Concrete class for NLopt
/*!
*/
class NLopt_Optimizer : public Optimizer {
  public:
    NLopt_Optimizer(Interface* interface);
    virtual ~NLopt_Optimizer();
  
    void initialize();
    void optimize();
    void finalize();

  private:

    // private member functions
    double evaluate_backend( unsigned int n, const double* x, double* grad );
    static double evaluate( unsigned int n, const double* x, double* grad, void* data);

    double constraint_backend( unsigned int n, const double* x, double* grad );
    static double constraint( unsigned int n, const double* x, double* grad, void* data);

    // private member data
    enum ResponseType {Measure, Aggregate};

    ResponseType m_objectiveType;
    ResponseType m_constraintType;

    std::vector<double> m_lowerBounds;
    std::vector<double> m_upperBounds;

    double m_objectiveValue;
    double m_objectiveValue_last;
    double m_constraintValue;
    double m_constraintValue_last;

    double *p, *p_last;
    double f, g;
    double* x_ref;

    int m_numOptDofs;

    string m_optMethod;       // MMA, CCSA, SLSQP

    nlopt_opt opt;

    bool isChanged(const double* x);

};


}  // end Plato namespace
