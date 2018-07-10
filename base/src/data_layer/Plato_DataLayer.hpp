/*
 * Plato_DataLayer.hpp
 *
 *  Created on: May 1, 2017
 *
 */

#ifndef SRC_DATALAYER_HPP_
#define SRC_DATALAYER_HPP_

#include <map>
#include <vector>
#include <string>

namespace Plato
{

class SharedData;
class SharedDataInfo;
struct CommunicationData;

/******************************************************************************/
//!  Data sharing manager
/*!
 */
/******************************************************************************/
class DataLayer
{
public:
    DataLayer(const Plato::SharedDataInfo & aSharedDataInfo, const Plato::CommunicationData & aCommData);
    ~DataLayer();

    // accessors
    SharedData* getSharedData(const std::string & aName) const;
    const std::vector<SharedData*> & getSharedData() const;

private:
    std::vector<SharedData*> mSharedData;
    std::map<std::string, SharedData*> mSharedDataMap;

private:
    DataLayer(const Plato::DataLayer & aRhs);
    Plato::DataLayer & operator=(const Plato::DataLayer & aRhs);
};

} /* namespace Plato */

#endif /* SRC_INTERFACE_HPP_ */
