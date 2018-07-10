/*
 * Plato_Criterion.hpp
 *
 *  Created on: Oct 6, 2017
 */

#ifndef PLATO_CRITERION_HPP_
#define PLATO_CRITERION_HPP_

#include <memory>

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class MultiVector;

template<typename ScalarType, typename OrdinalType = size_t>
class Criterion
{
public:
    virtual ~Criterion()
    {
    }

    //! Directive to cache any criterion specific data once trial control is accepted.
    virtual void cacheData() = 0;
    /*!
     * Evaluates criterion of type f(\mathbf{u}(\mathbf{z}),\mathbf{z})\colon\mathbb{R}^{n_u}\times\mathbb{R}^{n_z}
     * \rightarrow\mathbb{R}, where u denotes the state and z denotes the control variables. This criterion
     * is typically associated with nonlinear programming optimization problems. For instance, PDE constrasize_t
     * optimization problems.
     *  Parameters:
     *    \param In
     *          aControl: control variables
     *
     *  \return Objective function value
     **/
    virtual ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl) = 0;
    /*!
     * Computes the gradient of a criterion of type f(\mathbf{u}(\mathbf{z}),\mathbf{z})\colon\mathbb{R}^{n_u}
     * \times\mathbb{R}^{n_z}\rightarrow\mathbb{R}, where u denotes the state and z denotes the control variables.
     * This criterion is typically associated with nonlinear programming optimization problems. For instance, PDE
     * constraint optimization problems.
     *  Parameters:
     *    \param In
     *          aControl: control variables
     *    \param Out
     *          aOutput: gradient
     **/
    virtual void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                          Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
    /*!
     * Computes the application of a vector to the Hessian of a criterion of type f(\mathbf{u}(\mathbf{z}),\mathbf{z})
     * \colon\mathbb{R}^{n_u}\times\mathbb{R}^{n_z}\rightarrow\mathbb{R}, where u denotes the state and z denotes the
     * control variables. This criterion is typically associated with nonlinear programming optimization problems.
     * For instance, PDE constraint optimization problems.
     *  Parameters:
     *    \param In
     *          aControl: control variables
     *    \param In
     *          aVector:  direction vector
     *    \param Out
     *          aOutput:  Hessian times direction vector
     **/
    virtual void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                         const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                         Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
};

}

#endif /* PLATO_CRITERION_HPP_ */
