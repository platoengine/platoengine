#ifndef _COGENT_WRITEUTILS_H
#define _COGENT_WRITEUTILS_H

namespace Cogent {

template<typename V, typename P>
void writeTets(const std::vector<Simplex<V,P> >& explicitSimplexes, std::string format="vtk");

template<typename V, typename P>
void writeTetsVTK(const std::vector<Simplex<V,P> >& explicitSimplexes);

template<typename V, typename P>
void writeTetsCase(const std::vector<Simplex<V,P> >& explicitSimplexes);

template<typename V, typename P>
void writeTris(const std::vector<Simplex<V,P> >& explicitSimplexes);

} /** end namespace Cogent */

#include "Cogent_WriteUtils_Def.hpp"
#endif
