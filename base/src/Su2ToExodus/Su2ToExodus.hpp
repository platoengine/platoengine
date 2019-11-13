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
 * Su2ToExodus.hpp
 *
 *  Created on: Oct 31, 2019
 *
 */

#ifndef SU2TOEXODUS_HPP_
#define SU2TOEXODUS_HPP_

#include <string>
#include <vector>
#include <map>
#include <set>

namespace Su2ToExodus
{

struct Su2Data
{
    int mNumDimensions;
    int mNumElements;
    int mNumNodes;
    int mNumMarks;
    int mNumElementBlocks;
    std::vector<double> mNodeX;
    std::vector<double> mNodeY;
    std::vector<double> mNodeZ;
    std::vector<int> mNodeSetIDs;
    std::vector<int> mSideSetIDs;
    std::vector<std::vector<int> > mNodeSets;
    std::vector<std::vector<int> > mSideSetsElem;
    std::vector<std::vector<int> > mSideSetsFace;
    std::vector<std::string> mSideSetNames;
    std::vector<std::string> mNodeSetNames;
    std::vector<std::vector<int> > mElementConnectivity;
    std::vector<std::vector<std::vector<int> > > mMarks;
    std::vector<int> mMarkTags;
    std::vector<int> mMarkNumElems;
    std::vector<int> mMarkTypeIDs;
    std::vector<std::string> mMarkTypes;
    std::vector<std::string> mMarkNames;
    std::map<int,std::map<int,std::map<int,int>>> mFaceToTetMap;
   
};

class Su2ToExodus
{

public:
    Su2ToExodus();
    ~Su2ToExodus(){}

    bool readSu2File(std::string &aFilename);
    void setNumElements(int aValue) {mSu2Data.mNumElements = aValue;}
    void setNumNodes(int aValue) {mSu2Data.mNumNodes = aValue;}
    void setNumDimensions(int aValue) {mSu2Data.mNumDimensions = aValue;}
    void setNumMarks(int aValue) {mSu2Data.mNumMarks = aValue;}
    int getElementConnectivity(int aElementIndex, int aNodeIndex){return mSu2Data.mElementConnectivity[aElementIndex][aNodeIndex];}
    double getNodeX(int aNodeIndex){return mSu2Data.mNodeX[aNodeIndex];}
    double getNodeY(int aNodeIndex){return mSu2Data.mNodeY[aNodeIndex];}
    double getNodeZ(int aNodeIndex){return mSu2Data.mNodeZ[aNodeIndex];}
    int getMarkNodeIndex(int aMarkIndex, int aElemIndex, int aNodeIndex){return mSu2Data.mMarks[aMarkIndex][aElemIndex][aNodeIndex];}
    void setInputFilename(std::string aFilename){mInputFilename = aFilename;}
    void setOutputFilename(std::string aFilename){mOutputFilename = aFilename;}
    void convert();
    int getNodeSetNode(int aNodeSetIndex, int aNodeIndex){return mSu2Data.mNodeSets[aNodeSetIndex][aNodeIndex];}
    int getSideSetElem(int aSideSetIndex, int aSideIndex){return mSu2Data.mSideSetsElem[aSideSetIndex][aSideIndex];}
    int getSideSetSide(int aSideSetIndex, int aSideIndex){return mSu2Data.mSideSetsFace[aSideSetIndex][aSideIndex];}
    void addMarkTypeID(int aTypeID){mSu2Data.mMarkTypeIDs.push_back(aTypeID);}
    void addMarkType(std::string aType){mSu2Data.mMarkTypes.push_back(aType);}
    void addMarkName(std::string aName){mSu2Data.mMarkNames.push_back(aName);}

protected:

    int getNamedIntegerField(std::istream &aStream, const char *aName);
    bool readSu2FileFromStream(std::istream &aStream);
    bool readElementConnectivity(std::istream &aStream);
    bool readNodeCoordinates(std::istream &aStream);
    bool readMarks(std::istream &aStream);
    bool openExodusFile(std::string &aFilename);
    bool closeExodusFile();
    bool writeExodusFile();
    bool writeCoordinates();
    bool writeElementConnectivity();
    bool writeNodeSets();
    bool writeSideSets();
    bool createNodeSetFromMark(int &aMarkIndex, std::string &aName);
    bool createSideSetFromMark(int &aMarkIndex, std::string &aName);
    bool createNodeSetsFromMarks();
    bool createSideSetsFromMarks();
    bool createFaceToTetMap();
    int getFaceIndex(int &aConnectedElem, std::vector<int> &aFaceConn);

private:

    Su2Data mSu2Data;
    int mExodusFileID;
    std::string mInputFilename;
    std::string mOutputFilename;



};

} // namespace Su2ToExodus

#endif /* SU2TOEXODUS_HPP_ */
