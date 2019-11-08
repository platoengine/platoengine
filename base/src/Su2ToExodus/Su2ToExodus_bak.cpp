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

/*
 * Su2ToExodus.cpp
 *
 *  Created on: Oct 31, 2019
 *
 */

#include "Su2ToExodus.hpp"
#include "exodusII.h"
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <iostream>

namespace Su2ToExodus
{

Su2ToExodus::Su2ToExodus()
{
    mExodusFileID = -1;
    mSu2Data.mNumElementBlocks = 1; // limitation for now
    mSu2Data.mNumSideSets = 0;
}

bool Su2ToExodus::readSu2File(std::string &aFilename)
{
    bool tRet = true;
    std::ifstream tFile(aFilename);
    if (tFile.is_open())
    {
        tRet = readSu2FileFromStream(tFile);
        tFile.close();
    }
    else
        tRet = false;

    return tRet;
}

bool Su2ToExodus::readSu2FileFromStream(std::istream &aStream)
{
    bool tRet = true;
    mSu2Data.mNumDimensions = getNamedIntegerField(aStream, "NDIME");
    mSu2Data.mNumElements = getNamedIntegerField(aStream, "NELEM");
    if(mSu2Data.mNumDimensions != -1 && mSu2Data.mNumElements != -1)
    {
        if(!readElementConnectivity(aStream))
            tRet = false;
        else
        {
            mSu2Data.mNumNodes = getNamedIntegerField(aStream, "NPOIN");
            if(mSu2Data.mNumNodes != -1)
            {
                if(!readNodeCoordinates(aStream))
                    tRet = false;
                else
                {
                    mSu2Data.mNumMarks = getNamedIntegerField(aStream, "NMARK");
                    if(!readMarks(aStream))
                        tRet = false;
                }
            }
            else
                tRet = false;
        }
    }
    else
        tRet = false;

    return tRet;
}

bool Su2ToExodus::readMarks(std::istream &aStream)
{
    bool tRet = true;
    for(int i=0; i<mSu2Data.mNumMarks; ++i)
    {
        int tMarkTag = getNamedIntegerField(aStream, "MARKER_TAG");
        int tMarkNumElems = getNamedIntegerField(aStream, "MARKER_ELEMS");
        if(tMarkTag != -1 && tMarkNumElems != -1)
        {
            std::vector<std::vector<int> > tCurMarkElems;
            mSu2Data.mMarkTags.push_back(tMarkTag);
            mSu2Data.mMarkNumElems.push_back(tMarkNumElems);
            for(int j=0; j<tMarkNumElems; ++j)
            {
                std::vector<int> tCurElemNodes;
                std::string tLine;
                std::getline(aStream, tLine);
                size_t tPos = tLine.find(' ');
                if(tPos != std::string::npos)
                {
                    int tElementType = std::atoi(tLine.substr(0, tPos).c_str());
                    int tNumNodes = 0;
                    if(tElementType == 5)
                        tNumNodes = 3;
                    tLine = tLine.substr(tPos+1);
                    for(int k=0; k<tNumNodes; ++k)
                    {
                        if(k==(tNumNodes-1))
                        {
                            int tNodeId = std::atoi(tLine.c_str());
                            tCurElemNodes.push_back(tNodeId);
                        }
                        else
                        {
                            tPos = tLine.find(' ');
                            if(tPos != std::string::npos)
                            {
                                int tNodeId = std::atoi(tLine.substr(0, tPos).c_str());
                                tCurElemNodes.push_back(tNodeId);
                                tLine = tLine.substr(tPos+1);
                            }
                            else
                            {
                                tRet = false;
                                k=tNumNodes;
                                j=tMarkNumElems;
                            }
                        }
                    }
                    if(tRet)
                        tCurMarkElems.push_back(tCurElemNodes);
                }
                else
                {
                    tRet = false;
                    j=tMarkNumElems;
                }

            }
            if(tRet)
                mSu2Data.mMarks.push_back(tCurMarkElems);
        }
        else
        {
            tRet = false;
            i=mSu2Data.mNumMarks;
        }
    }
    return tRet;
}

void Su2ToExodus::convert()
{
    if(mInputFilename.length() > 0 && mOutputFilename.length() > 0)
    {
        if(readSu2File(mInputFilename))
        {
            createNodeSetsFromMarks();
            createSideSetsFromMarks();
            if(openExodusFile(mOutputFilename))
            {
                writeExodusFile();
                closeExodusFile();
            }
        }
    }
}

bool Su2ToExodus::writeExodusFile()
{
    bool tRet=true;


    if( ex_put_init(mExodusFileID, "dummy title", mSu2Data.mNumDimensions, mSu2Data.mNumNodes, mSu2Data.mNumElements,
                    mSu2Data.mNumElementBlocks,
                    mSu2Data.mNodeSets.size(), mSu2Data.mNumSideSets) )
    {
        std::cout << "\n!!! Problem initializing exodus file \n";
        tRet = false;
    }

    writeCoordinates();
    writeElementConnectivity();
    writeNodeSets();
    writeSideSets();

    return tRet;
}

bool Su2ToExodus::writeCoordinates()
{
    bool tRet = true;
    double* tX = (double*)(mSu2Data.mNodeX.data());
    double* tY = (double*)(mSu2Data.mNodeY.data());
    double* tZ = NULL;
    if( mSu2Data.mNumDimensions == 3 )
        tZ = (double*)(mSu2Data.mNodeZ.data());
    if(ex_put_coord(mExodusFileID, tX, tY, tZ))
    {
        std::cout << "\n!!! Problem writing coords to exodus file \n";
        tRet = false;
    }
    return tRet;
}

bool Su2ToExodus::writeNodeSets()
{
    bool tRet = true;

    for(size_t i=0; i<mSu2Data.mNodeSets.size(); i++)
    {
        int tNumDistInSet = 0;
        int tNumInSet = mSu2Data.mNodeSets[i].size();
        int tRes = ex_put_set_param(mExodusFileID, EX_NODE_SET, mSu2Data.mNodeSetIDs[i], tNumInSet, tNumDistInSet);
        if(tRes)
        {
            std::cout << "\n!!! Problem initializing exodus file with node set id " << mSu2Data.mNodeSetIDs[i];
        }
        else
        {
            if(mSu2Data.mNodeSets[i].size())
            {
                if(ex_put_set(mExodusFileID, EX_NODE_SET, mSu2Data.mNodeSetIDs[i], mSu2Data.mNodeSets[i].data(), NULL))
                {
                    std::cout << "\n!!! Problem writing nodes in node set id: " << mSu2Data.mNodeSetIDs[i]
                            << " to file \n";
                }
            }
        }
    }

    return tRet;
}

bool Su2ToExodus::writeSideSets()
{
    bool tRet = true;
/*
    for(size_t i=0; i<mSu2Data.mNodeSets.size(); i++)
    {
        int tNumDistInSet = 0;
        int tNumInSet = mSu2Data.mNodeSets[i].size();
        int tRes = ex_put_set_param(mExodusFileID, EX_NODE_SET, mSu2Data.mNodeSetIDs[i], tNumInSet, tNumDistInSet);
        if(tRes)
        {
            std::cout << "\n!!! Problem initializing exodus file with node set id " << mSu2Data.mNodeSetIDs[i];
        }
        else
        {
            if(mSu2Data.mNodeSets[i].size())
            {
                if(ex_put_set(mExodusFileID, EX_NODE_SET, mSu2Data.mNodeSetIDs[i], mSu2Data.mNodeSets[i].data(), NULL))
                {
                    std::cout << "\n!!! Problem writing nodes in node set id: " << mSu2Data.mNodeSetIDs[i]
                            << " to file \n";
                }
            }
        }
    }
    */

    return tRet;
}

bool Su2ToExodus::createNodeSetsFromMarks()
{
    bool tRet = true;

    mSu2Data.mNodeSetIDs.clear();
    mSu2Data.mNodeSets.clear();
    for(size_t i=0; i<mSu2Data.mMarks.size(); ++i)
    {
        bool tCurMarkIsNodeset = false;
        for(size_t j=0; j<mSu2Data.mMarkTypeIDs.size(); ++j)
        {
            if(mSu2Data.mMarkTypeIDs[j] == mSu2Data.mMarkTags[i] &&
               mSu2Data.mMarkTypes[j] == "nodeset")
            {
                tCurMarkIsNodeset = true;
                j = mSu2Data.mMarkTypeIDs.size();
            }
        }
        if(tCurMarkIsNodeset)
        {
            int tIndex = i;
            tRet = createNodeSetFromMark(tIndex);
            if(!tRet)
                i = mSu2Data.mMarks.size();
        }
    }

    return tRet;
}

bool Su2ToExodus::createSideSetsFromMarks()
{
    bool tRet = true;
/*
    mSu2Data.mNodeSetIDs.clear();
    mSu2Data.mNodeSets.clear();
    for(size_t i=0; i<mSu2Data.mMarks.size(); ++i)
    {
        bool tCurMarkIsNodeset = false;
        for(size_t j=0; j<mSu2Data.mMarkTypeIDs.size(); ++j)
        {
            if(mSu2Data.mMarkTypeIDs[j] == mSu2Data.mMarkTags[i] &&
               mSu2Data.mMarkTypes[j] == "nodeset")
            {
                tCurMarkIsNodeset = true;
                j = mSu2Data.mMarkTypeIDs.size();
            }
        }
        if(tCurMarkIsNodeset)
        {
            int tIndex = i;
            tRet = createNodeSetFromMark(tIndex);
            if(!tRet)
                i = mSu2Data.mMarks.size();
        }
    }
    */

    return tRet;
}

bool Su2ToExodus::createNodeSetFromMark(int &aMarkIndex)
{
    bool tRet = true;

    int tCntr = 0;
    mSu2Data.mNodeSetIDs.push_back(mSu2Data.mMarkTags[aMarkIndex]);
    std::vector<int> tNewNodeSet(mSu2Data.mMarkNumElems[aMarkIndex]*mSu2Data.mMarks[aMarkIndex][0].size());
    for(int i=0; i<mSu2Data.mMarkNumElems[aMarkIndex]; ++i)
    {
        for(size_t j=0; j<mSu2Data.mMarks[aMarkIndex][0].size(); ++j)
        {
            tNewNodeSet[tCntr] = mSu2Data.mMarks[aMarkIndex][i][j] + 1;
            ++tCntr;
        }
    }
    mSu2Data.mNodeSets.push_back(tNewNodeSet);
    return tRet;
}

bool Su2ToExodus::writeElementConnectivity()
{
    bool tRet = true;
    for(int i=0; i<mSu2Data.mNumElementBlocks; ++i)
    {
        int tNumElements = mSu2Data.mNumElements;
        const char* tType = "tet";
        int tNumNodePerElem = 4;
        int tElementBlockID = 1;
        if(ex_put_block(mExodusFileID, EX_ELEM_BLOCK, tElementBlockID, tType, tNumElements, tNumNodePerElem, 0, 0, 0))
        {
            std::cout << "\n!!! Problem writing element block " << tElementBlockID << "\n"
                    << "!!! \tto exodus file \n";
            tRet = false;
        }

        if(tRet)
        {
            // Change connectivity to be 1-based rather than 0-based
            int* tTempConnectivity = new int [tNumNodePerElem * tNumElements];
            for(int n=0; n<tNumElements; ++n)
            {
                for(int h=0; h<tNumNodePerElem; ++h)
                {
                    int tLocalIndex = n*tNumNodePerElem;
                    tTempConnectivity[tLocalIndex+h] = mSu2Data.mElementConnectivity[n][h] + 1;
                }
            }
            if(ex_put_conn(mExodusFileID, EX_ELEM_BLOCK, tElementBlockID, tTempConnectivity, 0, 0))
            {
                std::cout << "\n!!! Problem writing connectivity for element block " << tElementBlockID << "\n"
                        << "!!! \tto exodus file \n";
                tRet = false;
            }
            delete [] tTempConnectivity;
        }
    }
    return tRet;
}

bool Su2ToExodus::openExodusFile(std::string &aFilename)
{
    bool tRet = true;
    int CPU_word_size = sizeof(double);
    int IO_word_size = 8;

    mExodusFileID = ex_create(aFilename.c_str(), EX_CLOBBER, &CPU_word_size, &IO_word_size);
    if( mExodusFileID <= 0 )
    {
        std::cout << "\n!!! Problem opening exodus file " << aFilename << "\n"
                << "!!! \tfor write. \n" << std::endl;
        tRet = false;
    }
    return tRet;

}

bool Su2ToExodus::closeExodusFile()
{
    bool tRet = true;

    if( mExodusFileID == -1 )
        return tRet;

    int errCode = ex_close(mExodusFileID);
    if( errCode )
    {
      std::cout << "\n!!! Problem closing exodus mesh file. \n";
      tRet = false;
    }

    return tRet;
}

bool Su2ToExodus::readNodeCoordinates(std::istream &aStream)
{
    bool tRet = true;
    mSu2Data.mNodeX.resize(mSu2Data.mNumNodes);
    mSu2Data.mNodeY.resize(mSu2Data.mNumNodes);
    mSu2Data.mNodeZ.resize(mSu2Data.mNumNodes);
    for(int i=0; i<mSu2Data.mNumNodes; ++i)
    {
        std::vector<double> tCurNodeCoordinates(3, 0.0);
        std::string tLine;
        std::getline(aStream, tLine);
        for(int j=0; j<mSu2Data.mNumDimensions; ++j)
        {
            size_t tPos = tLine.find(' ');
            if(tPos != std::string::npos)
            {
                double tCoordVal = std::atof(tLine.substr(0, tPos).c_str());
                tCurNodeCoordinates[j] = tCoordVal;
                tLine = tLine.substr(tPos+1);
            }
            else
            {
                tRet = false;
                j=mSu2Data.mNumDimensions;
                i=mSu2Data.mNumNodes;
            }
        }
        if(tRet)
        {
            mSu2Data.mNodeX[i] = tCurNodeCoordinates[0];
            mSu2Data.mNodeY[i] = tCurNodeCoordinates[1];
            if(mSu2Data.mNumDimensions == 3)
                mSu2Data.mNodeZ[i] = tCurNodeCoordinates[2];
        }
    }
    return tRet;
}

bool Su2ToExodus::readElementConnectivity(std::istream &aStream)
{
    bool tRet = true;
    for(int i=0; i<mSu2Data.mNumElements; ++i)
    {
        std::vector<int> tCurElemConnectivity;
        std::string tLine;
        std::getline(aStream, tLine);
        size_t tPos = tLine.find(' ');
        if(tPos != std::string::npos)
        {
            int tElementType = std::atoi(tLine.substr(0, tPos).c_str());
            int tNumNodes = 0;
            if(tElementType == 10)
                tNumNodes = 4;
            tLine = tLine.substr(tPos+1);
            for(int j=0; j<tNumNodes; ++j)
            {
                tPos = tLine.find(' ');
                if(tPos != std::string::npos)
                {
                    int tNodeId = std::atoi(tLine.substr(0, tPos).c_str());
                    tCurElemConnectivity.push_back(tNodeId);
                    tLine = tLine.substr(tPos+1);
                }
                else
                {
                    tRet = false;
                    j=tNumNodes;
                    i=mSu2Data.mNumElements;
                }
            }
            if(tRet)
                mSu2Data.mElementConnectivity.push_back(tCurElemConnectivity);
        }
        else
        {
            tRet = false;
            i=mSu2Data.mNumElements;
        }
    }
    return tRet;
}

int Su2ToExodus::getNamedIntegerField(std::istream &aStream, const char *aName)
{
    int tRet = -1;
    std::string tLine;
    std::getline(aStream, tLine);
    size_t tPos = tLine.find('=');
    std::string tValueName = tLine.substr(0, tPos);
    if(tPos != std::string::npos)
    {
        // Remove leading/trailing white space
        std::string::iterator tEndPos = std::remove(tValueName.begin(), tValueName.end(), ' ');
        tValueName.erase(tEndPos, tValueName.end());
        if(tValueName == aName)
        {
            std::string tNewString = tLine.substr(tPos+1);
            tRet = std::atoi(tNewString.c_str());
        }
    }

    return tRet;
}

} // namespace Su2ToExodus


