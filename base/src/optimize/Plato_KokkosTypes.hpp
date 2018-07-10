/*
 * Plato_KokkosTypes.hpp
 *
 *  Created on: May 6, 2018
 */

#ifndef PLATO_KOKKOSTYPES_HPP_
#define PLATO_KOKKOSTYPES_HPP_

#include <Kokkos_Core.hpp>
#include <Kokkos_Macros.hpp>

#ifdef KOKKOS_HAVE_CUDA
#define LAMBDA_EXPRESSION [=] __device__
#else
#define LAMBDA_EXPRESSION [=]
#endif

#ifdef KOKKOS_HAVE_CUDA
#define DEVICE_TYPE __device__
#else
#define DEVICE_TYPE
#endif

namespace Plato
{

using Scalar = double;
using ExecSpace = Kokkos::DefaultExecutionSpace;
using MemSpace = typename ExecSpace::memory_space;
using Layout = Kokkos::LayoutRight;

template <typename ScalarType>
using ScalarVectorT = typename Kokkos::View<ScalarType*, MemSpace>;
using ScalarVector  = ScalarVectorT<Scalar>;

}

#endif /* PLATO_KOKKOSTYPES_HPP_ */
