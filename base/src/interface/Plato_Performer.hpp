/*
 * Plato_Performer.hpp
 *
 *  Created on: April 23, 2017
 *
 */

#ifndef SRC_PERFORMER_HPP_
#define SRC_PERFORMER_HPP_

#include <string>

namespace Plato
{

class Application;
class SharedData;

//!  Lightweight handle for storing Applications
/*!
 */
class Performer
{
public:
    Performer(const std::string & aMyName, const int & aCommID);

    void finalize();
    void compute(const std::string & aOperationName);

    void importData(const std::string & aArgumentName, const SharedData & aImportData);
    void exportData(const std::string & aArgumentName, SharedData & aExportData);

    void setApplication(Application* aApplication);

    std::string myName();
    int myCommID();

private:
    Application* mApplication;  // TODO make this a unique pointer
    const std::string mName;
    const int mCommID;

private:
    Performer(const Performer& aRhs);
    Performer& operator=(const Performer& aRhs);
};

} // End namespace Plato

#endif
