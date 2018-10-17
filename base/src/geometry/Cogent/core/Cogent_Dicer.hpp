#ifndef _COGENT_DICER_H
#define _COGENT_DICER_H

#include <Shards_CellTopology.hpp>
#include <Intrepid2_Basis.hpp>

#include "Cogent_Types.hpp"
#include "Cogent_Model.hpp"
#include "Cogent_Utilities.hpp"

namespace Cogent {

struct SimplexStencil {
   SimplexStencil(std::vector<int> cutPts,
                  std::vector<std::vector<int>> newPts,
                  std::vector<std::vector<int>> map,
                  std::vector<Cogent::Sense> sense);
   std::vector<int> m_cutPts;
   std::vector<std::vector<int>> m_newPts;
   std::vector<std::vector<int>> m_subTets;
   std::vector<std::vector<int>> m_topoMap;
   std::vector<Cogent::Sense> m_sense;
};

std::vector<Cogent::SimplexStencil> makeStencils();

class Dicer
/**
 *  This class computes a set of conformal tetrahedra from a collection of
 *  levelsets and constructive solid geometry (CSG) instructions.  A hexahedral
 *  parent element is assumed.
 */
{
  public:
	  /**
	   * Constructor.
	   *
	   * @param celltype The topology of the parent element
	   * @param basis FEM basis on the parent element
	   * @param subDomains List of CSG operations that define the geometry
	   * @param _interfaceValue Interface value of the level surfaces
	   */
    Dicer(
      Teuchos::RCP<const shards::CellTopology> celltype,
      Teuchos::RCP<const Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > basis,
      RealType _interfaceValue);

    /**
     * Given a collection of level set fields and a list of
     * implicit simplexes, compute the explicit simplexes that conform
     * to the underlying geometry.
     *
     * @param geomData (Input)
     * @param implicitSimplexes (Input)
     * @param explicitSimplexes (Output)
     */
    template<typename V, typename P>
    void Cut(const FContainer<P>& geomData, 
                   std::vector<Simplex<V,P> >& implicitSimplexes,  
                   std::vector<Simplex<V,P> >& explicitSimplexes);

    /**
     * Evaluate the parent element basis functions given a set of
     * points (evalPoints) in parent element coordinates.
     *
     * @param Nvals (Output) Basis function values.
     * @param evalPoints (Input) Points in parent element coordinates.
     */
    template<typename V, typename P>
    void getValues(       FContainer<V>& Nvals, 
                    const FContainer<P>& evalPoints);

    template <typename V, typename P>
    V Volume(Simplex<V,P>& simplex);


  private:
//    template <typename V, typename P>
//    bool isMaterial(const Simplex<V,P>& simplex);

    template<typename C, typename V, typename P>
    Sense getSense(
      const Simplex<V,P>& simplex,
      const C& compare, int iTopo);

    template <typename N, typename V, typename P>
    void Project( const FContainer<N>& geomData, 
                  std::vector<Simplex<V,P> >& implicitSimplexes);

    template<typename C, typename V, typename P>
    void CutTet(const std::vector<Simplex<V,P> >& implicitSimplexes, 
                      std::vector<Simplex<V,P> >& explicitSimplexes, const C& comparison, int iTopo);
  
    template<typename V, typename P>
    void CutTet(const std::vector<Simplex<V,P> >& implicitSimplexes, 
                      std::vector<Simplex<V,P> >& explicitSimplexes, int iTopo);
  
    template<typename C, typename V, typename P>
    void CutTri(const std::vector<Simplex<V,P> >& implicitSimplexes, 
                      std::vector<Simplex<V,P> >& explicitSimplexes, const C comparison, int iTopo);

    template<typename V, typename P>
    void Mesh(const std::vector<typename Vector3D<P>::Type >& points,
              std::vector<Simplex<V,P> >& explicitSimplexes, int iTopo);
  
    template<typename P>
    void SortMap(const std::vector<typename Vector3D<P>::Type>& points, std::vector<int>& map);
  
    template<typename P>
    bool areColinear(const std::vector<typename Vector3D<P>::Type>& points);
  
    Teuchos::RCP<Intrepid2::Basis<Kokkos::Serial, DFadType, DFadType > > m_DFadBasis;
    Teuchos::RCP<const Intrepid2::Basis<Kokkos::Serial, RealType, RealType > > m_basis;
    RealType m_interfaceValue;
    uint m_numDims;

    std::vector<Cogent::SimplexStencil> m_cutStencils;

};

} /** end namespace Cogent */

#include "Cogent_Dicer_Def.hpp"
#endif
