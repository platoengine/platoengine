/*
 * Plato_Performer.cpp
 *
 *  Created on: April 23, 2017
 *
 */

#include "Plato_Performer.hpp"
#include "Plato_Application.hpp"

namespace Plato
{

Performer::Performer(const std::string & aMyName, const int & aCommID) :
        mApplication(nullptr),
        mName(aMyName),
        mCommID(aCommID)
{
}

void Performer::finalize()
{
    if(mApplication)
    {
        mApplication->finalize();
    }
}

void Performer::compute(const std::string & aOperationName)
{
    if(mApplication)
    {
        mApplication->compute(aOperationName);
    }
}

void Performer::importData(const std::string & aArgumentName, const SharedData & aImportData)
{
    if(mApplication)
    {
        mApplication->importData(aArgumentName, aImportData);
    }
}

void Performer::exportData(const std::string & aArgumentName, SharedData & aExportData)
{
    if(mApplication)
    {
        mApplication->exportData(aArgumentName, aExportData);
    }
}

void Performer::setApplication(Application* aApplication)
{
    mApplication = aApplication;
}

std::string Performer::myName()
{
    return mName;
}

int Performer::myCommID()
{
    return mCommID;
}

}
