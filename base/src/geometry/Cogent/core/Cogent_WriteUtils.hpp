#ifndef _COGENT_WRITEUTILS_H
#define _COGENT_WRITEUTILS_H
#include <vector>
#include <string>

#include "Cogent_Types.hpp"

namespace Cogent {

class Writer {

  public:
    Writer(std::string a_format, std::string a_geoType, std::string a_fileNameBase) : m_format(a_format), m_geoType(a_geoType), m_fileNameBase(a_fileNameBase){}

    template<typename V, typename P>
    void write(const std::vector<Simplex<V,P> >& a_explicitSimplexes);

    void writeSurfaceTris(const std::vector<Simplex<RealType,RealType> >& a_outTets){}

    void close();
    
  private:
    std::string m_format;
    std::string m_geoType;
    std::string m_fileNameBase;
    std::vector<int> m_timeIndices;
};
  bool isEqual(
    const Simplex<RealType,RealType>& t0,
    const Simplex<RealType,RealType>& t1);


  template<typename V, typename P>
  void writeTets(const std::vector<Simplex<V,P> >& explicitSimplexes, std::string format="vtk");

  template<typename V, typename P>
  void writeTetsVTK(const std::vector<Simplex<V,P> >& explicitSimplexes);

  template<typename V, typename P>
  void writeTetsCase(const std::vector<Simplex<V,P> >& explicitSimplexes);

  template<typename V, typename P>
  void writeTrisCase(const std::vector<Simplex<V,P> >& explicitSimplexes);

  void writeSurfaceTris(const std::vector<Simplex<RealType,RealType> >& explicitSimplexes);

  void getSurfaceTris(const std::vector<Simplex<RealType,RealType> >& inTets, 
                      std::vector<Simplex<RealType,RealType> >& outTris);

  template<typename V, typename P>
  void writeTris(const std::vector<Simplex<V,P> >& explicitSimplexes);


} /** end namespace Cogent */

#include "Cogent_WriteUtils_Def.hpp"
#endif
