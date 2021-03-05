#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <cassert>
#include <tuple>
#include <set>
#include <map>
#include <PSL_Vector.hpp>


namespace PlatoSubproblemLibrary
{

std::vector<int> computeNumElementsInEachDirection(const Vector& aMaxUVWCoords,
                                                   const Vector& aMinUVWCoords,
                                                   const double& aTargetEdgeLength);

void computeGridXYZCoordinates(const Vector& aUBasisVector,
                                 const Vector& aVBasisVector,
                                 const Vector& aBuildDirection,
                                 const Vector& aMaxUVWCoords,
                                 const Vector& aMinUVWCoords,
                                 const std::vector<int>& aNumElements,
                                 std::vector<Vector>& aXYZCoordinates);

int getSerializedIndex(const std::vector<int>& aNumElementsInEachDirection, const int& i, const int& j, const int& k);
int getSerializedIndex(const std::vector<int>& aNumElementsInEachDirection, const std::vector<int>& aIndex);

}
