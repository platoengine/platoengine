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

/*!
 * Plato_MLS.hpp
 *
 * Created on: Feb 27, 2018
 *
 */

#ifndef PLATO_MLS_HPP_
#define PLATO_MLS_HPP_

#include <Kokkos_Core.hpp>
#include <array>
#include <map>
#include <string>
#include <stdio.h>
#include <math.h>
#include "Plato_Parser.hpp"
#include "Plato_Exceptions.hpp"

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

using MemSpace = Kokkos::DefaultExecutionSpace::memory_space;

namespace Plato {
namespace Geometry {

template<typename ScalarType=double>
using ScalarVector = typename Kokkos::View<ScalarType*, MemSpace>;

template<typename ScalarType=double>
using ScalarArray  = typename Kokkos::View<ScalarType**, Kokkos::LayoutRight, MemSpace>;

using NeighborMap = typename Kokkos::View<int**, Kokkos::LayoutRight, Kokkos::DefaultExecutionSpace::memory_space>;
using NeighborMapHost = typename NeighborMap::HostMirror;

template<int SpaceDim, typename ScalarType>
class NeighborMapper
{
   // for now, this is done on the host
   typename ScalarArray<ScalarType>::HostMirror m_points;

   struct Neighbor{ 
     Neighbor() : index(0), distance(0.0) {}
     int        index;
     ScalarType distance;
     friend bool operator< (const Neighbor & a, const Neighbor & b)
     {
          return a.distance < b.distance;
     }
   };
  
   public:
       NeighborMapper(const typename ScalarArray<ScalarType>::HostMirror & a_points) : m_points( a_points ) {}

       NeighborMap map( int a_numNeighbors )
       {
           int t_numPoints = m_points.extent(0);
           NeighborMap retMap("map", t_numPoints, a_numNeighbors);
           auto hstMap = Kokkos::create_mirror_view(retMap);
  
           // use brute force for now.  need a k-d tree w/ n nearest neighbor search
           for(int i=0; i<t_numPoints; i++)
           {
               ScalarType t_iPoint[SpaceDim];
               for(int k=0; k<SpaceDim; k++)
               {
                   t_iPoint[k] = m_points(i,k);
               }

               std::vector<Neighbor> t_neighbors(t_numPoints);
               for(int j=0; j<t_numPoints; j++)
               {
                   t_neighbors[j].index = j;
                   ScalarType t_square_distance = 0.0;
                   for(int k=0; k<SpaceDim; k++)
                   {
                       t_square_distance += (m_points(j,k)-t_iPoint[k])*(m_points(j,k)-t_iPoint[k]);
                   }
                   t_neighbors[j].distance = t_square_distance;
               }
               std::sort(t_neighbors.begin(), t_neighbors.end());
               for(int j=0; j<a_numNeighbors; j++)
               {
                   hstMap(i,j) = t_neighbors[j].index;
               }
           }
           Kokkos::deep_copy(retMap, hstMap);
           return retMap;
       }
};

template<int SpaceDim=3, typename ScalarType=double>
class PointGrid
{

  ScalarVector<int> m_n;
  ScalarVector<int> m_stride;

  ScalarVector<ScalarType> m_d;
  ScalarVector<ScalarType> m_o;
  int m_numPoints;
  
  public:
    PointGrid(std::array<int,SpaceDim> n, 
              std::array<ScalarType,SpaceDim> d,
              std::array<ScalarType,SpaceDim> o) :
       m_n     ("num grid points",SpaceDim),
       m_stride("grid strides",   SpaceDim),
       m_d     ("grid dimensions",SpaceDim),
       m_o     ("grid offsets",   SpaceDim)
    {
      auto n_host = Kokkos::create_mirror_view(m_n);
      auto d_host = Kokkos::create_mirror_view(m_d);
      auto o_host = Kokkos::create_mirror_view(m_o);

      for( int i=0; i<SpaceDim; i++){
        n_host(i) = n[i];
        d_host(i) = d[i];
        o_host(i) = o[i];
      }
      Kokkos::deep_copy(m_n, n_host);
      Kokkos::deep_copy(m_d, d_host);
      Kokkos::deep_copy(m_o, o_host);

      m_numPoints = 1;
      auto stride_host = Kokkos::create_mirror_view(m_stride);
      for(int i=0; i<SpaceDim; i++){
        stride_host(i) = 1.0;
        m_numPoints *= n[i];
      }
      for(int i=SpaceDim-1; i>=0; i--){
        for(int j=i-1; j>=0; j--){
          stride_host(j) *= n[i];
        }
      }
      Kokkos::deep_copy(m_stride, stride_host);
    }
    
    ScalarArray<ScalarType>
    operator()(){
      // create coordinates
      //
      ScalarArray<ScalarType> coords("coordinates", m_numPoints, SpaceDim);
      auto t_stride = m_stride;
      auto t_o = m_o;
      auto t_d = m_d;
      auto t_numPoints = m_numPoints;
      Kokkos::parallel_for(Kokkos::RangePolicy<int>(0,t_numPoints),LAMBDA_EXPRESSION(int pointIndex)
      {
        int t_i[SpaceDim] = {0};
        for(int i=0; i<SpaceDim; i++){
          int index = pointIndex;
          for(int j=0; j<SpaceDim-1; j++){
            index -= t_i[j]*t_stride(j);
          }
          t_i[i]=index/t_stride(i);
        }
        for( int iDim=0; iDim<SpaceDim; iDim++)
        {
          coords(pointIndex,iDim) = t_o(iDim) + t_i[iDim]*t_d(iDim);
        }
      });
      return coords;
    }
};

template<int SpaceDim=3, typename ScalarType=double>
class SphereArray {

    ScalarType m_radius, m_insideValue, m_outsideValue, m_spacing;
    ScalarArray<ScalarType> m_sphereCoords;

    public:
    /***************************************************************************/
    /*!
     *  /brief Constructor with parsing.
     *  Creates a SphereArray instance from the provided xml input.
     */
    /***************************************************************************/
    SphereArray(const Plato::InputData& a_node, const std::array<ScalarType,SpaceDim>& a_size, const std::array<ScalarType,SpaceDim>& a_offset)
    {
          m_radius       = Plato::Get::Double(a_node, "Radius");
          m_insideValue  = Plato::Get::Double(a_node, "InsideValue");
          m_outsideValue = Plato::Get::Double(a_node, "OutsideValue");
          m_spacing      = Plato::Get::Double(a_node, "Spacing");
 
          // create grid of spheres
          //
          std::array<int,SpaceDim> t_n;
          std::array<ScalarType,SpaceDim> t_d, t_o;
          for(int i=0; i<SpaceDim; i++){
            t_n[i] = a_size[i] / m_spacing + 1; 
            t_d[i] = m_spacing;
            t_o[i] = a_offset[i];
          }
          PointGrid<SpaceDim,ScalarType> gp(t_n, t_d, t_o);
          m_sphereCoords = gp();
     }

     void
     getValues(ScalarArray<ScalarType> a_coords, ScalarVector<ScalarType> a_field){
          // loop on grid points then spheres
          int t_numSpheres = m_sphereCoords.extent(0);
          int t_numPoints  = a_coords.extent(0);
          auto t_radius = m_radius;
          auto t_insideValue = m_insideValue;
          auto t_outsideValue = m_outsideValue;
          auto t_sphereCoords = m_sphereCoords;
          Kokkos::parallel_for(Kokkos::RangePolicy<int>(0,t_numPoints),LAMBDA_EXPRESSION(int pointIndex)
          {
            a_field(pointIndex) = t_outsideValue;
            ScalarType t_pointCoord[SpaceDim];
            for(int iDim=0; iDim<SpaceDim; iDim++){
              t_pointCoord[iDim] = a_coords(pointIndex,iDim);
            }
            for(int iSphere=0; iSphere<t_numSpheres; iSphere++){
              ScalarType t_distance=0.0;
              for(int iDim=0; iDim<SpaceDim; iDim++){
                t_distance += pow(t_pointCoord[iDim] - t_sphereCoords(iSphere,iDim),2);
              }
              ScalarType diff = (t_distance/(t_radius*t_radius) < 1.0) ? t_insideValue - t_outsideValue : 0.0;
              a_field(pointIndex) += diff;
            }
          });
     }
};


template<int SpaceDim=3, typename ScalarType=double>
class MovingLeastSquares {

    ScalarArray<ScalarType> m_coords;
    NeighborMap m_neighborMap;
    std::map<std::string, ScalarVector<ScalarType>> m_fields;

    ScalarType m_radius;

    std::string strint(std::string base, int index)
    {
      std::stringstream out;
      out << base << index;
      return out.str();
    }

  public:

    typename decltype(m_coords)::HostMirror getPointCoords()
    {
        typename decltype(m_coords)::HostMirror m_coords_mirror = Kokkos::create_mirror_view(m_coords);
        Kokkos::deep_copy(m_coords_mirror, m_coords);
        return m_coords_mirror;
    }


    decltype(m_fields) getPointFields(){return m_fields;}

    int getNumPoints(){return m_coords.extent(0); }

    /***************************************************************************/
    /*!
     *  /brief Constructor with parsing.
     *  Creates a MovingLeastSquares instance from the provided xml input.
     */
    /***************************************************************************/
    MovingLeastSquares(const Plato::InputData& a_node)
    {
      std::array<ScalarType,SpaceDim> t_delta, t_size, t_offset;
      std::array<int,SpaceDim> t_num;
      for(int i=0; i<SpaceDim; i++)
      {
        t_num[i] = Plato::Get::Int(a_node,strint("N",i+1));
        t_size[i] = Plato::Get::Double(a_node,strint("Size",i+1));
        t_offset[i] = Plato::Get::Double(a_node,strint("Offset",i+1));
        t_delta[i] = (t_num[i]-1)!=0 ? t_size[i] / (t_num[i] - 1) : 0.0;
      }

      // create coordinates
      //
      PointGrid<SpaceDim,ScalarType> gp(t_num, t_delta, t_offset);
      m_coords = gp();

      auto t_coords_host = getPointCoords();
      NeighborMapper<SpaceDim,ScalarType> tNeighborMapper(t_coords_host);
      
      int t_numNeighbors;
      if( a_node.size<std::string>("NumNeighbors") ) {
        t_numNeighbors = Plato::Get::Int(a_node, "NumNeighbors");
      } else {
        t_numNeighbors = t_coords_host.extent(0);
      }
      m_neighborMap = tNeighborMapper.map( t_numNeighbors );


      m_radius = Plato::Get::Double(a_node, "Radius");

      // create fields
      //
      auto t_fieldNodes = a_node.getByName<Plato::InputData>("Field");
      for(auto t_field=t_fieldNodes.begin(); t_field!=t_fieldNodes.end(); ++t_field)
      {
        // create new point field
        //
        auto t_name = Plato::Get::String(*t_field,"Name");
        if( m_fields.find(t_name) != m_fields.end() ){
          std::stringstream ss;
          ss << "Plato::Geometry::MovingLeastSquares: " << std::endl;
          ss << "  Attempted to create a Field (" << t_name << " ) that already exists: " << std::endl;
          throw Plato::ParsingException(ss.str());
        }
        m_fields[t_name] = ScalarVector<ScalarType>(t_name, getNumPoints());


        // define values of new point field
        //
        auto t_initializer = Plato::Get::String(*t_field,"Initializer");
        auto t_initNode = t_field->get<Plato::InputData>(t_initializer.c_str());
        if( t_initializer == "Uniform" ){
          auto t_value = Plato::Get::Double(t_initNode, "Value");
          Kokkos::deep_copy(m_fields[t_name], t_value);
        } else 
        if( t_initializer == "SphereArray" ){

          SphereArray<SpaceDim,ScalarType> t_sphereArray(t_initNode, t_size, t_offset);
          t_sphereArray.getValues(m_coords, m_fields[t_name]);

        }
      }
    }

    /******************************************************************************/
    /*! 
     *  /brief Compute the MLS function values.
     *  /input a_pointValues Values at the MLS points
     *  /input a_nodeCoords Coordinates (nodeIndex, dimIndex) of nodes for which MLS 
     *         function values will be computed
     *  /output a_nodeValues MLS function values (nodeIndex)
     */
    /******************************************************************************/
    void
    f(ScalarVector<ScalarType> a_pointValues, ScalarArray<ScalarType> a_nodeCoords, ScalarVector<ScalarType> a_nodeValues)
    {
      int t_numNodes = a_nodeValues.size();
      int t_numPoints = a_pointValues.size();
      auto t_coords = m_coords;
      auto t_radius = m_radius;
      Kokkos::parallel_for(Kokkos::RangePolicy<int>(0,t_numNodes),LAMBDA_EXPRESSION(int nodeIndex)
      {
        ScalarType t_nodeCoord[SpaceDim];
        for(int iDim=0; iDim<SpaceDim; iDim++){
          t_nodeCoord[iDim] = a_nodeCoords(nodeIndex,iDim);
        }
        ScalarType numerator=0.0, denominator=0.0;
        for(int iPoint=0; iPoint<t_numPoints; iPoint++){
          ScalarType norm=0.0;
          for(int iDim=0; iDim<SpaceDim; iDim++){
            norm += pow(t_nodeCoord[iDim]-t_coords(iPoint,iDim),2);
          }
          auto t_kernelVal = exp(-norm/(t_radius*t_radius));
          numerator   += a_pointValues(iPoint)*t_kernelVal;
          denominator += t_kernelVal;
        }
        a_nodeValues(nodeIndex) = denominator != 0.0 ? numerator / denominator : 0.0;
      });
    }

    /******************************************************************************/
    /*! 
     *  /brief Map a derivative with respect to MLS values at nodes to a derivative
               with respect to MLS point values.
     *  /input a_nodeCoords Coordinates (nodeIndex, dimIndex) of nodes.
     *  /input a_nodeValues Values (nodeIndex) of a gradient with respect to nodal MLS values
     *  /output a_mappedValues Values (pointIndex) of the gradient with respect to the MLS point values.
     */
    /******************************************************************************/
    void
    mapToPoints( 
      ScalarArray<ScalarType> a_nodeCoords, 
      ScalarVector<ScalarType> a_nodeValues, 
      ScalarVector<ScalarType> a_mappedValues)
    {

      auto t_mappedValuesHost = Kokkos::create_mirror_view(a_mappedValues);
      auto t_neighborMap = m_neighborMap;
      int tNumNeighbors = t_neighborMap.extent(1);

      int tNumPoints = m_coords.extent(0);
      auto t_coords = m_coords;
      auto t_radius = m_radius;
      for(int t_pointIndex=0; t_pointIndex<tNumPoints; t_pointIndex++){
        ScalarType tReturnVal(0.0);
        typedef typename ScalarArray<ScalarType>::size_type size_type;
        Kokkos::parallel_reduce( Kokkos::RangePolicy<>(0,a_nodeValues.size()), LAMBDA_EXPRESSION( const size_type & aNodeIndex, ScalarType& aLocalResult)
        {
           // buffer the node coordinate (X_k)
           ScalarType t_nodeCoord[SpaceDim];
           for(int iDim=0; iDim<SpaceDim; iDim++){
             t_nodeCoord[iDim] = a_nodeCoords(aNodeIndex,iDim);
           }
  
           // compute numerator (e_{kl})
           ScalarType norm=0.0;
           for(int iDim=0; iDim<SpaceDim; iDim++){
             norm += pow(t_nodeCoord[iDim]-t_coords(t_pointIndex,iDim),2);
           }
           ScalarType numerator = exp(-norm/(t_radius*t_radius));
  
           // compute denominator (sum_i e_{ki} )
           ScalarType denominator=0.0;
           for(int jPoint=0; jPoint<tNumNeighbors; jPoint++){
             ScalarType norm=0.0;
             for(int iDim=0; iDim<SpaceDim; iDim++){
               norm += pow(t_nodeCoord[iDim]-t_coords(t_neighborMap(t_pointIndex,jPoint),iDim),2);
             }
             denominator += exp(-norm/(t_radius*t_radius));
           }
           ScalarType tInc = (denominator != 0) ? a_nodeValues(aNodeIndex) * numerator / denominator : 0.0;
           aLocalResult += tInc;
        }, tReturnVal);
        t_mappedValuesHost(t_pointIndex) = tReturnVal;
      }
      Kokkos::deep_copy(a_mappedValues, t_mappedValuesHost);
    }
};

} // end namespace Geometry
} // end namespace Plato


#endif
