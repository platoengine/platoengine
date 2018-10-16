/*
 * Cogent_NonParameterizedModel.hpp
 *
 *  Created on: Dec 6, 2016
 *      Author: jrobbin
 */

#ifndef COGENT_NONPARAMETERIZEDMODEL_HPP_
#define COGENT_NONPARAMETERIZEDMODEL_HPP_

#include "Cogent_Model.hpp"

namespace Cogent {

class NonParameterizedModel: public Model {
public:
  NonParameterizedModel(const Teuchos::ParameterList& params);
  virtual ~NonParameterizedModel();

  bool isMaterial(const std::vector<Sense>& sense);
  bool isEmptyElement();
  bool isPartialElement();
  void getSignedDistance(FContainer<RealType>& topoData);
  void mapToGeometryData(const FContainer<RealType>& dMdT, 
                               FContainer<RealType>& dMdG);

private:

  enum struct Operation { Add, Subtract, NotSet };
  struct SubDomain {
    std::vector<int> indices;
    Operation operation;
  };

  std::vector<SubDomain> m_subDomains;
};

} /* namespace Cogent */

#endif /* COGENT_NONPARAMETERIZEDMODEL_HPP_ */
