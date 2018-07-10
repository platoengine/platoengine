/*
 * Plato_Application.hpp
 *
 *  Created on: April 23, 2017
 *
 */

#ifndef SRC_APPLICATION_HPP_
#define SRC_APPLICATION_HPP_

#include <string>
#include <vector>

#include "Plato_SharedData.hpp"

namespace Plato
{

//!  Application: Base class defining hosted code interface
/*!
 */
class Application
{
public:
    virtual ~Application()
    {
    }

    virtual void finalize() = 0;
    virtual void initialize() = 0;
    virtual void compute(const std::string & aOperationName) = 0;
    virtual void exportData(const std::string & aArgumentName, Plato::SharedData & aExportData) = 0;
    virtual void importData(const std::string & aArgumentName, const Plato::SharedData & aImportData) = 0;
    virtual void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs) = 0;
};

} // End namespace Plato

#endif
