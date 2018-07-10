/*
 * OC_Optimizer.hpp
 *
 *  Created on: April 19, 2017
 *
 */

#include "Plato_Optimizer.hpp"

namespace Plato {

//!  Concrete class for OC
/*!
*/
class OC_Optimizer : public Optimizer {
  public:
    OC_Optimizer(Interface* interface);
    ~OC_Optimizer();

    void initialize();
    void optimize();
    void finalize();

  private:
    void zeroSet();
    void computeUpdatedTopology();
    inline void updateTopology(double* dfdp, double* dmdp, double lambda);
    double *p, *p_last;

    double m_moveLimit;
    double m_stabExponent;
    bool   m_useNewtonSearch;
    int    m_constraintMaxIter;

    int m_numOptDofs;

};

} // end Plato namespace
