#ifndef _COGENT_TYPES_H
#define _COGENT_TYPES_H

#include <Sacado.hpp>
#include <Kokkos_Core.hpp>
#include <Kokkos_DynRankView.hpp>
#include <Kokkos_ViewFactory.hpp>
#include <MiniTensor.h>

typedef unsigned int uint;
typedef double RealType;
typedef Sacado::Fad::DFad<RealType> FadType;
typedef Sacado::mpl::apply<FadType,RealType>::type DFadType;

namespace Cogent {

  enum struct Axis {X, Y, Z};

  static constexpr int nTriPts { 3 };
  static constexpr int nTetPts { 4 };
  static constexpr int nSpcDms { 3 };

  template<class T> 
  using FContainer = Kokkos::DynRankView<T, Kokkos::Serial>;

  enum struct Sense { Positive, Negative, Mixed, NotSet };

  template <typename N> 
  struct Positive { 
    typedef std::greater_equal<N> Type; 
    Type compare;
    static constexpr Sense sense = Sense::Positive;
  };
  template <typename N> 
  struct Negative { 
    typedef std::less_equal<N> Type; 
    Type compare;
    static constexpr Sense sense = Sense::Negative;
  };

  template <typename N> struct Vector3D { typedef minitensor::Vector<N,nSpcDms> Type; };
  typedef minitensor::Vector<int,nTriPts> Tri;
  typedef minitensor::Vector<int,nTetPts> Tet;

  template <typename V, typename P>
  struct Simplex {
    Simplex(){bodyLS=-1;}
    Simplex(int n) {points.resize(n,typename Vector3D<P>::Type(minitensor::Filler::ZEROS));
      fieldvals.resize(n);
      bodyLS=-1;
      boundaryLS.resize(n,-1);
    }
    std::vector<minitensor::Vector<P,nSpcDms> > points;
    std::vector<std::vector<V> > fieldvals;
    std::vector<Sense> sense;
    std::vector<int> boundaryLS;
    int bodyLS;
  };

  template <typename T>
  struct LocalMatrix {
    public:
      LocalMatrix(int dim0, int dim1) : m_dim0(dim0),m_dim1(dim1),m_data(new T[m_dim0*m_dim1]){}
      ~LocalMatrix(){delete [] m_data;}
      T& operator()(int i, int j) const { return m_data[i*m_dim1+j];}
      T* operator()(int i){ return m_data+i*m_dim1;}
    private:
      const int m_dim0, m_dim1;
      T* m_data;
  };
}


#endif
