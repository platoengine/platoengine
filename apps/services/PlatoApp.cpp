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

#include <sstream>
#include <fstream>

#ifdef ENABLE_ISO
#include "STKExtract.hpp"
#endif

#ifdef STK_ENABLED
#include "stk_mesh/base/MetaData.hpp"
#include <stk_mesh/base/Field.hpp>
#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>
#endif

#include "PlatoApp.hpp"
#include "Plato_Parser.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_PenaltyModel.hpp"
#include "PlatoEngine_FilterFactory.hpp"
#include "PlatoEngine_AbstractFilter.hpp"
#include "Ioss_Region.h"                // for Region, NodeSetContainer, etc
#include "Plato_TimersTree.hpp"
#include "data_container.hpp"
#include "lightmp.hpp"
#include "types.hpp"
#include "matrix_container.hpp"

/******************************************************************************/
/*! \brief Get the layout.
 *  If no 'Layout' keyword is given in the input xml_node (aNode), 
 *  aDefaultLayout is returned.
 */
/******************************************************************************/
Plato::data::layout_t getLayout(Plato::InputData& aNode, Plato::data::layout_t aDefaultLayout);

/******************************************************************************/
/*! \brief Get the layout.
 *  If no 'Layout' keyword is given in the input xml_node (aNode), 
 *  a Plato::ParsingException is thrown.
 */
/******************************************************************************/
Plato::data::layout_t getLayout(Plato::InputData& aNode);

/******************************************************************************/
/*! \brief Get the layout.
 *  Returns the layout_t with the given name.
 */
/******************************************************************************/
Plato::data::layout_t getLayout(const std::string & aLayoutStr);

/******************************************************************************/
void PlatoApp::SetLowerBounds::getArguments(std::vector<LocalArg> & aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg(Plato::data::layout_t::SCALAR, mInputName));
    aLocalArgs.push_back(LocalArg(mOutputLayout, mOutputName, mOutputSize));
}

/******************************************************************************/
PlatoApp::SetLowerBounds::SetLowerBounds(PlatoApp* p, Plato::InputData& aNode) :
  LocalOp(p)
/******************************************************************************/
{
    mInputName = "Lower Bound Value";
    auto tInputData = Plato::Get::InputData(aNode,"Input");
    mInputName = Plato::Get::String(tInputData, "ArgumentName");

    mOutputName = "Lower Bound Vector";
    auto tOutputNode = Plato::Get::InputData(aNode,"Output");
    mOutputLayout = getLayout(tOutputNode, Plato::data::layout_t::SCALAR_FIELD);
    mOutputSize = Plato::Get::Int(tOutputNode,"Size");

    auto tFixedBlocksNode = Plato::Get::InputData(aNode, "FixedBlocks");
    mFixedBlocks = Plato::Get::Ints(tFixedBlocksNode, "Index");

    auto tFixedSidesetsNode = Plato::Get::InputData(aNode, "FixedSidesets");
    mFixedSidesets = Plato::Get::Ints(tFixedSidesetsNode, "Index");

    auto tFixedNodesetsNode = Plato::Get::InputData(aNode, "FixedNodesets");
    mFixedNodesets = Plato::Get::Ints(tFixedNodesetsNode, "Index");
    mDiscretization = Plato::Get::String(aNode, "Discretization");
}

/******************************************************************************/
void PlatoApp::SetLowerBounds::operator()()
/******************************************************************************/
{
    // Get the output field
    double* toData; 
    int tDataLen=0;

    if( mOutputLayout == Plato::data::layout_t::SCALAR_FIELD ){
      auto& tOutputField = *(mPlatoApp->getNodeField(mOutputName));
      tOutputField.ExtractView(&toData);
      tDataLen = tOutputField.MyLength();
    } else 
    if( mOutputLayout == Plato::data::layout_t::ELEMENT_FIELD ){
      throw Plato::ParsingException("ELEMENT_FIELD not implemented for SetLowerBounds operation");
    } else
    if( mOutputLayout == Plato::data::layout_t::SCALAR ){
      auto tOutputScalar = mPlatoApp->getValue(mOutputName);
      tDataLen = mOutputSize;
      tOutputScalar->resize(tDataLen);
      toData = tOutputScalar->data();
    }

    // Get incoming global lower bound specified by user
    std::vector<double>* tInData = mPlatoApp->getValue(mInputName);
    double tLowerBoundIn = (*tInData)[0];

    // Set specified value for the user
    for(int i=0; i<tDataLen; i++)
    {
        toData[i] = tLowerBoundIn;
    }

    // Now update values based on fixed entities
    if(mDiscretization == "density" && mOutputLayout == Plato::data::layout_t::SCALAR_FIELD )
    {
        LightMP* tLightMP = mPlatoApp->getLightMP();
        const int dofsPerNode_1D = 1;
        SystemContainer* sysGraph_1D = new SystemContainer(tLightMP->getMesh(), dofsPerNode_1D, tLightMP->getInput());
        std::vector<VarIndex> single_value(1u);
        DataContainer* dataContainer = tLightMP->getDataContainer();
        bool plottable = true;
        single_value[0] = dataContainer->registerVariable(RealType, "lowerBoundWorking", NODE, !plottable);
        DistributedVector* dist_vector = new DistributedVector(sysGraph_1D, single_value);

        double tBoundaryValue = .5001;
        double tUpperValue = 1.;
        mPlatoApp->mMeshServices->updateLowerBoundsForFixedBlocks(toData,
                                                                  mFixedBlocks,
                                                                  tLowerBoundIn,
                                                                  tBoundaryValue,
                                                                  tUpperValue,
                                                                  *dist_vector);
        mPlatoApp->mMeshServices->updateLowerBoundsForFixedSidesets(toData, mFixedSidesets, tBoundaryValue);
        mPlatoApp->mMeshServices->updateLowerBoundsForFixedNodesets(toData, mFixedNodesets, tBoundaryValue);

        delete dist_vector;
        delete sysGraph_1D;
    }
}

/******************************************************************************/
void PlatoApp::EnforceBounds::getArguments(std::vector<LocalArg>& localArgs)
/******************************************************************************/
{
    localArgs.push_back(LocalArg(Plato::data::layout_t::SCALAR_FIELD, mLowerBoundVectorFieldName));
    localArgs.push_back(LocalArg(Plato::data::layout_t::SCALAR_FIELD, mUpperBoundVectorFieldName));
    localArgs.push_back(LocalArg(Plato::data::layout_t::SCALAR_FIELD, mTopologyFieldName));
}

/******************************************************************************/
PlatoApp::EnforceBounds::EnforceBounds(PlatoApp* p, Plato::InputData& node) :
  LocalOp(p)
/******************************************************************************/
{
    mLowerBoundVectorFieldName = "Lower Bound Vector";
    mUpperBoundVectorFieldName = "Upper Bound Vector";
    mTopologyFieldName = "Topology";
}

/******************************************************************************/
void PlatoApp::EnforceBounds::operator()()
/******************************************************************************/
{
    // Get the output field
    double* tOutputData;
    double* tLowerBoundData;
    double* tUpperBoundData;
    int tDataLen=0;

    auto& tOutputField = *(mPlatoApp->getNodeField(mTopologyFieldName));
    tOutputField.ExtractView(&tOutputData);
    tDataLen = tOutputField.MyLength();

    auto& tLowerBoundField = *(mPlatoApp->getNodeField(mLowerBoundVectorFieldName));
    tLowerBoundField.ExtractView(&tLowerBoundData);

    auto& tUpperBoundField = *(mPlatoApp->getNodeField(mUpperBoundVectorFieldName));
    tUpperBoundField.ExtractView(&tUpperBoundData);

    for(int tIndex = 0; tIndex < tDataLen; tIndex++)
    {
        tOutputData[tIndex] = std::max(tOutputData[tIndex], tLowerBoundData[tIndex]);
        tOutputData[tIndex] = std::min(tOutputData[tIndex], tUpperBoundData[tIndex]);
    }
}

/******************************************************************************/
void PlatoApp::SetUpperBounds::getArguments(std::vector<LocalArg> & aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg(Plato::data::layout_t::SCALAR, mInputName));
    aLocalArgs.push_back(LocalArg(mOutputLayout, mOutputName, mOutputSize));
}

/******************************************************************************/
PlatoApp::SetUpperBounds::SetUpperBounds(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
  LocalOp(aPlatoApp)
/******************************************************************************/
{
    mInputName = "Upper Bound Value";
    auto tInputData = Plato::Get::InputData(aNode,"Input");
    mInputName = Plato::Get::String(tInputData, "ArgumentName");

    mOutputName = "Upper Bound Vector";
    auto tOutputNode = Plato::Get::InputData(aNode,"Output");
    mOutputLayout = getLayout(tOutputNode, Plato::data::layout_t::SCALAR_FIELD);
    mOutputSize = Plato::Get::Int(tOutputNode,"Size");

    auto tFixedBlocksNode = Plato::Get::InputData(aNode, "FixedBlocks");
    mFixedBlocks = Plato::Get::Ints(tFixedBlocksNode, "Index");

    auto tFixedSidesetsNode = Plato::Get::InputData(aNode, "FixedSidesets");
    mFixedSidesets = Plato::Get::Ints(tFixedSidesetsNode, "Index");

    auto tFixedNodesetsNode = Plato::Get::InputData(aNode, "FixedNodesets");
    mFixedNodesets = Plato::Get::Ints(tFixedNodesetsNode, "Index");

    mDiscretization = Plato::Get::String(aNode, "Discretization");
}

/******************************************************************************/
void PlatoApp::SetUpperBounds::operator()()
/******************************************************************************/
{
    // Get the output field
    double* toData; 
    int tDataLen=0;

    if( mOutputLayout == Plato::data::layout_t::SCALAR_FIELD ){
      auto& tOutputField = *(mPlatoApp->getNodeField(mOutputName));
      tOutputField.ExtractView(&toData);
      tDataLen = tOutputField.MyLength();
    } else 
    if( mOutputLayout == Plato::data::layout_t::ELEMENT_FIELD ){
      throw Plato::ParsingException("ELEMENT_FIELD not implemented for SetUpperBounds operation");
    } else
    if( mOutputLayout == Plato::data::layout_t::SCALAR ){
      auto tOutputScalar = mPlatoApp->getValue(mOutputName);
      tDataLen = mOutputSize;
      tOutputScalar->resize(tDataLen);
      toData = tOutputScalar->data();
    }

    // Get incoming global Upper bound specified by user
    std::vector<double>* tInData = mPlatoApp->getValue(mInputName);
    double tUpperBoundIn = (*tInData)[0];

    // Set specified value for the user
    for(int i=0; i<tDataLen; i++)
    {
        toData[i] = tUpperBoundIn;
    }
    // Now update values based on fixed entities
    if(mDiscretization == "levelset" && mOutputLayout == Plato::data::layout_t::SCALAR_FIELD )
    {
        double tValue = -0.001;
        mPlatoApp->mMeshServices->updateUpperBoundsForFixedBlocks(toData, mFixedBlocks, tValue);
        mPlatoApp->mMeshServices->updateUpperBoundsForFixedSidesets(toData, mFixedSidesets, tValue);
        mPlatoApp->mMeshServices->updateUpperBoundsForFixedNodesets(toData, mFixedNodesets, tValue);
    }
}

/******************************************************************************/
void PlatoApp::InitializeField::getArguments(std::vector<LocalArg> & aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg(mOutputLayout, m_outputFieldName));
}

/******************************************************************************/
PlatoApp::InitializeField::InitializeField(PlatoApp* aPlatoApp, Plato::InputData & aNode) :
  LocalOp(aPlatoApp), m_outputFieldName("Initialized Field")
/******************************************************************************/
{
  m_strMethod = Plato::Get::String(aNode, "Method");

  Plato::InputData tMethodNode = Plato::Get::InputData(aNode, m_strMethod);
  if(m_strMethod == "Uniform")
  {
    m_uniformValue = Plato::Get::Double(tMethodNode, "Value");
  }
  else if(m_strMethod == "FromFile")
  {
      m_fileName = Plato::Get::String(tMethodNode, "Name");
      m_variableName = Plato::Get::String(tMethodNode, "VariableName");
      m_iteration = Plato::Get::Int(tMethodNode, "Iteration");

      // Copy all of the platomain.exo.* files to
      // platomain_restart.exo.* because platomain.exo
      // will get overwritten for the next run. I am doing
      // this in code to try to avoid making system calls.
      int tCommSize = 0;
      MPI_Comm_size(mPlatoApp->mLocalComm, &tCommSize);

      for(int i=0; i<tCommSize; ++i)
      {
          char tInFilename[200];
          char tOutFilename[200];
          if(tCommSize == 1)
          {
              sprintf(tInFilename, "platomain.exo.1.0");
              sprintf(tOutFilename, "platomain_restart.exo");
          }
          else
          {
              sprintf(tInFilename, "platomain.exo.%d.%d", tCommSize, i);
              sprintf(tOutFilename, "platomain_restart.exo.%d.%d", tCommSize, i);
          }

          int tBufferSize=1024;
          char    tBuffer[tBufferSize];
          FILE    *tInFile, *tOutFile;
          size_t  tNumRead;

          tInFile = fopen(tInFilename, "rb" );
          tOutFile= fopen(tOutFilename, "wb" );
          if(tInFile && tOutFile)
          {
              while((tNumRead=fread(tBuffer, 1, tBufferSize,tInFile)) != 0)
              {
                  fwrite(tBuffer, 1, tNumRead, tOutFile);
              }
              fclose(tInFile);
              fclose(tOutFile);
          }
      }
  }
  else if(m_strMethod == "SwissCheeseLevelSet")
  {
      m_fileName = Plato::Get::String(tMethodNode, "Name");
      m_createSpheres = true;
      m_createSpheres  = Plato::Get::Bool(tMethodNode, "CreateSpheres");
      m_sphereSpacingX = Plato::Get::String(tMethodNode, "SphereSpacingX");
      m_sphereSpacingY = Plato::Get::String(tMethodNode, "SphereSpacingY");
      m_sphereSpacingZ = Plato::Get::String(tMethodNode, "SphereSpacingZ");
      m_sphereRadius   = Plato::Get::String(tMethodNode, "SphereRadius");
      m_spherePackingFactor = Plato::Get::String(tMethodNode, "SpherePackingFactor");
      m_fileName       = Plato::Get::String(tMethodNode, "BackgroundMeshName");

      if( m_fileName.empty() )
      {
          std::cout << "\n\n********************************************************************************\n";
          std::cout << "ERROR: No BackgroundMeshName was specified in the SwissCheeseLevelSet section.\n";
          std::cout << "********************************************************************************\n\n";
      }
      m_levelSetNodesets = Plato::Get::Ints(tMethodNode, "NodeSet");
  }
  else if(m_strMethod == "PrimitivesLevelSet")
  {
      m_fileName = Plato::Get::String(tMethodNode, "BackgroundMeshName");

      if( m_fileName.empty() )
      {
          std::cout << "\n\n********************************************************************************\n";
          std::cout << "ERROR: No BackgroundMeshName was specified in the SwissCheeseLevelSet section.\n";
          std::cout << "********************************************************************************\n\n";
      }
      auto tNodes = tMethodNode.getByName<Plato::InputData>("MaterialBox");
      for(auto tNode=tNodes.begin(); tNode!=tNodes.end(); ++tNode)
      {
          std::string tTempString = Plato::Get::String(*tNode, "MinCoords");
          if(tTempString != "")
          {
              int pos = tTempString.find(" ");
              m_minCoords[0] = atof(tTempString.substr(0,pos).c_str());
              tTempString = tTempString.substr(pos+1);
              pos = tTempString.find(" ");
              m_minCoords[1] = atof(tTempString.substr(0,pos).c_str());
              tTempString = tTempString.substr(pos+1);
              pos = tTempString.find(" ");
              m_minCoords[2] = atof(tTempString.substr(0,pos).c_str());
          }
          tTempString = Plato::Get::String(*tNode, "MaxCoords");
          if(tTempString != "")
          {
              int pos = tTempString.find(" ");
              m_maxCoords[0] = atof(tTempString.substr(0,pos).c_str());
              tTempString = tTempString.substr(pos+1);
              pos = tTempString.find(" ");
              m_maxCoords[1] = atof(tTempString.substr(0,pos).c_str());
              tTempString = tTempString.substr(pos+1);
              pos = tTempString.find(" ");
              m_maxCoords[2] = atof(tTempString.substr(0,pos).c_str());
          }
      }
  }
  else if(m_strMethod == "FromFieldOnInputMesh")
  {
      m_variableName = Plato::Get::String(tMethodNode, "VariableName");
  }

  Plato::InputData tOutputNode = Plato::Get::InputData(aNode,"Output");
  mOutputLayout = getLayout(tOutputNode, Plato::data::layout_t::SCALAR_FIELD);
}

/******************************************************************************/
void PlatoApp::InitializeField::operator()()
/******************************************************************************/
{
    double* toData; 

    if( mOutputLayout == Plato::data::layout_t::SCALAR_FIELD ){
      auto& field = *(mPlatoApp->getNodeField(m_outputFieldName));
      field.ExtractView(&toData);
      int dataLen = field.MyLength();

      if(m_strMethod == "Uniform")
      {
          for(int i=0; i<dataLen; i++){
              toData[i] = m_uniformValue;
          }
      }
      else if(m_strMethod == "FromFile")
      {
          std::vector<double> values;
          getInitialValuesForRestart(field, values);
          for(int i=0; i<dataLen; i++)
          {
              toData[i] = values[i];
          }
      }
      else if(m_strMethod == "SwissCheeseLevelSet")
      {
          std::vector<double> values;
          getInitialValuesForSwissCheeseLevelSet(field, values);
          for(int i=0; i<dataLen; i++)
          {
              toData[i] = values[i];
          }
      }
      else if(m_strMethod == "PrimitivesLevelSet")
      {
          std::vector<double> values;
          getInitialValuesForPrimitivesLevelSet(field, values);
          for(int i=0; i<dataLen; i++)
          {
              toData[i] = values[i];
          }
      }
      else if(m_strMethod == "FromFieldOnInputMesh")
      {
          mPlatoApp->getLightMP()->getMesh()->readNodePlot(toData, m_variableName);
      }
    } else
    if( mOutputLayout == Plato::data::layout_t::ELEMENT_FIELD ){
      auto fieldIndex    = mPlatoApp->getElementField(m_outputFieldName);
      auto dataContainer = mPlatoApp->mLightMp->getDataContainer();
      dataContainer->getVariable(fieldIndex, toData);
      int dataLen = mPlatoApp->mLightMp->getMesh()->getNumElems();

      if(m_strMethod == "Uniform")
      {
          for(int i=0; i<dataLen; i++){
              toData[i] = m_uniformValue;
          }
      }
      else if(m_strMethod == "FromFile")
      {
          throw Plato::ParsingException("InitializeField is not implemented for ELEMENT_FIELD layout.");
      }
      
    }
}

/******************************************************************************/
double PlatoApp::InitializeField::evaluateSwissCheeseLevelSet(const double &aX, const double &aY, const double &aZ,
                                                            std::vector<double> aLowerCoordBoundsOfDomain,
                                                            std::vector<double> aUpperCoordBoundsOfDomain,
                                                            double aAverageElemLength)
/******************************************************************************/
{
    bool doSimpleBoundsOnly = false;
    if (doSimpleBoundsOnly)
    {
        double relDist = 1e-1*(aUpperCoordBoundsOfDomain[1] - aLowerCoordBoundsOfDomain[1]);
        double lsTop = aY - (aUpperCoordBoundsOfDomain[1] - relDist);
        double lsBottom = (aLowerCoordBoundsOfDomain[1] + relDist) - aY;
        double lsval = std::max(lsTop, lsBottom);

        double relXVal = aUpperCoordBoundsOfDomain[0] - aLowerCoordBoundsOfDomain[0];
        relXVal*=1.e-3;
        if (   (std::abs(aLowerCoordBoundsOfDomain[0] - aX) < relXVal) || (std::abs(aUpperCoordBoundsOfDomain[0] - aX) < relXVal))
        {
            lsval = -4.5;
        }

        return -lsval;
    }
    // get characteristic lengths
    size_t numDims = 3;
    std::vector<double> domainLengths(numDims,0.0);
    for(size_t dInd =0; dInd < numDims; ++dInd)
    {
        domainLengths[dInd] = aUpperCoordBoundsOfDomain[dInd] - aLowerCoordBoundsOfDomain[dInd];
    }

    // figure out a size and number of spheres to lay down
    // assume we should have at least 4 or 5 elements per sphere
    double sphereRad = aAverageElemLength*1.5;
    double packingFactor = 4.0;
    if(m_sphereRadius != "")
        sphereRad = atof(m_sphereRadius.c_str());
    if(m_spherePackingFactor != "")
        packingFactor = atof(m_spherePackingFactor.c_str());

    std::vector<size_t> numSpheresInDimension(numDims, 0);
    std::vector<double> sphereSpacing(numDims,0);

    for (size_t dInd=0; dInd < numDims; ++dInd)
    {
        // try to compute a spacing that leaves one sphere's width between each sphere, but err on the side of fewer spheres in each dimension
        numSpheresInDimension[dInd] = std::floor(domainLengths[dInd]/(packingFactor*sphereRad));
        if (numSpheresInDimension[dInd] < 1)
            numSpheresInDimension[dInd] = 1;

        // now that we have an actual number in each dimension, find the actual spacing between sphere centers
        sphereSpacing[dInd] = domainLengths[dInd]/static_cast<double>(numSpheresInDimension[dInd]);
    }

/* this kind of brakes things right now*/
    if(m_sphereSpacingX != "")
        sphereSpacing[0] = atof(m_sphereSpacingX.c_str());
    if(m_sphereSpacingY != "")
        sphereSpacing[1] = atof(m_sphereSpacingY.c_str());
    if(m_sphereSpacingZ != "")
        sphereSpacing[2] = atof(m_sphereSpacingZ.c_str());

    assert(numDims == 3); // lazy, wrote 3 nested for loops for 3 dimensions
    double lsValOut = 2;
    // now for each sphere, compute the levelset, take the min value
    for (size_t s1Ind = 0; s1Ind < numSpheresInDimension[0]; ++s1Ind)
    {
        for (size_t s2Ind = 0; s2Ind < numSpheresInDimension[1]; ++s2Ind)
        {
            for (size_t s3Ind = 0; s3Ind < numSpheresInDimension[2]; ++s3Ind)
            {
                // compute center of this sphere
                double cx = (0.5+static_cast<double>(s1Ind))*sphereSpacing[0]+aLowerCoordBoundsOfDomain[0];
                double cy = (0.5+static_cast<double>(s2Ind))*sphereSpacing[1]+aLowerCoordBoundsOfDomain[1];
                double cz = (0.5+static_cast<double>(s3Ind))*sphereSpacing[2]+aLowerCoordBoundsOfDomain[2];

                // calculate the distance from the sphere center to the input point
                double radsquared = (aX - cx)*(aX - cx) +(aY - cy)*(aY - cy)+(aZ - cz)*(aZ - cz);
                double rad = std::sqrt(radsquared);

                // now the levelset value is just the radius - teh sphere's radius, resulting in negative values inside the sphere, positive outside, zero on the boundary
                double lsVal = rad - sphereRad;

                // if this is the first sphere just take the resulting levelset value, if we are a subsequent sphere, take this sphere's levelset if it is less than any previous value (
                if ((s1Ind == 0) && (s2Ind ==0) && (s3Ind == 0))
                    lsValOut = lsVal;
                else if (lsVal < lsValOut)
                {
                    lsValOut = lsVal;
                }
            }
        }
    }

    return lsValOut;
}

/******************************************************************************/
void PlatoApp::InitializeField::getInitialValuesForSwissCheeseLevelSet(DistributedVector &field, std::vector<double> &values)
/******************************************************************************/
{
#ifdef STK_ENABLED
    stk::io::StkMeshIoBroker *broker = new stk::io::StkMeshIoBroker(mPlatoApp->mLocalComm);
    stk::mesh::MetaData *meta_data = new stk::mesh::MetaData;
    stk::mesh::BulkData *bulk_data = new stk::mesh::BulkData(*meta_data, mPlatoApp->mLocalComm);
    broker->set_bulk_data(*bulk_data);

    broker->set_option_to_not_collapse_sequenced_fields();
    broker->property_add(Ioss::Property("LOWER_CASE_VARIABLE_NAMES", "OFF"));
    broker->add_mesh_database(m_fileName, "exodus", stk::io::READ_MESH);
    broker->create_input_mesh();

    stk::mesh::Field<double> &tTempField = meta_data->declare_field<stk::mesh::Field<double>>(stk::topology::NODE_RANK, "swiss", 1);

    std::vector<double> tTempFieldVals(2541, 0);
    stk::mesh::put_field_on_mesh(tTempField, meta_data->universal_part(), tTempFieldVals.data());

    broker->populate_bulk_data();

    stk::mesh::Field<double, stk::mesh::Cartesian> *tCoordsField = meta_data->get_field<stk::mesh::Field<double, stk::mesh::Cartesian> >
                              (stk::topology::NODE_RANK, "coordinates");

    std::vector<stk::mesh::Entity> nodes;
    bulk_data->get_entities(stk::topology::NODE_RANK, meta_data->universal_part(), nodes);


    double minX, maxX;
    double minY, maxY;
    double minZ, maxZ;
    double* coords = stk::mesh::field_data(*tCoordsField, nodes[0]);
    minX = maxX = coords[0];
    minY = maxY = coords[1];
    minZ = maxZ = coords[2];
    for(size_t inode=1; inode<nodes.size(); inode++)
    {
        coords = stk::mesh::field_data(*tCoordsField, nodes[inode]);
        if(coords[0] < minX)
            minX = coords[0];
        else if(coords[0] > maxX)
            maxX = coords[0];
        if(coords[1] < minY)
            minY = coords[1];
        else if(coords[1] > maxY)
            maxY = coords[1];
        if(coords[2] < minZ)
            minZ = coords[2];
        else if(coords[2] > maxZ)
            maxZ = coords[2];
    }
    float tSendBuffer[3], tReceiveBuffer[3];
    tSendBuffer[0] = minX;
    tSendBuffer[1] = minY;
    tSendBuffer[2] = minZ;
    MPI_Allreduce(&tSendBuffer, &tReceiveBuffer, 3, MPI_FLOAT, MPI_MIN, mPlatoApp->mLocalComm);
    minX = tReceiveBuffer[0];
    minY = tReceiveBuffer[1];
    minZ = tReceiveBuffer[2];
    tSendBuffer[0] = maxX;
    tSendBuffer[1] = maxY;
    tSendBuffer[2] = maxZ;
    MPI_Allreduce(&tSendBuffer, &tReceiveBuffer, 3, MPI_FLOAT, MPI_MAX, mPlatoApp->mLocalComm);
    maxX = tReceiveBuffer[0];
    maxY = tReceiveBuffer[1];
    maxZ = tReceiveBuffer[2];

    stk::mesh::Selector myselector = meta_data->locally_owned_part();
    const stk::mesh::BucketVector &elem_buckets = bulk_data->get_buckets(stk::topology::ELEMENT_RANK, myselector);

    double local_dist_squared = 0;
    int cntr = 0;
    for ( stk::mesh::BucketVector::const_iterator bucket_iter = elem_buckets.begin();
            bucket_iter != elem_buckets.end();
            ++bucket_iter )
    {
        stk::mesh::Bucket &tmp_bucket = **bucket_iter;
        stk::topology bucket_top = tmp_bucket.topology();
        if(bucket_top == stk::topology::TET_4 ||
                bucket_top == stk::topology::HEX_8)
        {
            size_t num_elems = tmp_bucket.size();
            for (size_t i=0; i<num_elems; ++i)
            {
                stk::mesh::Entity cur_elem = tmp_bucket[i];
                stk::mesh::Entity const *elem_nodes = bulk_data->begin_nodes(cur_elem);
                double* coords1 = stk::mesh::field_data(*tCoordsField, elem_nodes[0]);
                double* coords2 = stk::mesh::field_data(*tCoordsField, elem_nodes[1]);
                double dist_squared = (coords1[0]-coords2[0])*(coords1[0]-coords2[0]) +
                        (coords1[1]-coords2[1])*(coords1[1]-coords2[1]) +
                        (coords1[2]-coords2[2])*(coords1[2]-coords2[2]);
                local_dist_squared += dist_squared;
                cntr++;
            }
        }
    }
    local_dist_squared = local_dist_squared/((double)cntr);
    int numProcs;
    MPI_Comm_size(mPlatoApp->mLocalComm, &numProcs);
    tSendBuffer[0] = local_dist_squared;
    MPI_Allreduce(&tSendBuffer, &tReceiveBuffer, 1, MPI_FLOAT, MPI_SUM, mPlatoApp->mLocalComm);
    double global_average_distance = sqrt(tReceiveBuffer[0]/((double)numProcs));

    // Get the ids of all of the nodes that are in nodesets to be included as level sets.
    std::set<int> tNodeSetGlobalIds;
    const stk::mesh::PartVector tParts = meta_data->get_mesh_parts();
    for(size_t i=0; i<tParts.size(); ++i)
    {
        stk::mesh::Part *tCurPart = tParts[i];
        stk::mesh::EntityRank tCurRank = tCurPart->primary_entity_rank();
        if(tCurRank == 0)
        {
            int tPartId = tCurPart->id();
            if(std::find(m_levelSetNodesets.begin(), m_levelSetNodesets.end(), tPartId) != m_levelSetNodesets.end())
            {
                nodes.clear();
                bulk_data->get_entities(stk::topology::NODE_RANK, *tCurPart, nodes);
                for(size_t b=0; b<nodes.size(); ++b)
                {
                    tNodeSetGlobalIds.insert(bulk_data->identifier(nodes[b]));
                }
            }
        }
    }

    std::vector<double> tLowerCoordBoundsOfDomain = {minX, minY, minZ};
    std::vector<double> tUpperCoordBoundsOfDomain = {maxX, maxY, maxZ};

    int length = field.MyLength();
    for(int i=0; i<length; ++i)
    {
        int global_id = field.getAssemblyEpetraVector()->Map().GID(i);
        stk::mesh::Entity entity = bulk_data->get_entity(stk::topology::NODE_RANK, global_id);
        double* vals = stk::mesh::field_data(*tCoordsField, entity);
        double tAverageElemLength=global_average_distance;
        double val;
        if(tNodeSetGlobalIds.find(global_id) != tNodeSetGlobalIds.end())
        {
            val = 1.0;
        }
        else
        {
            if(m_createSpheres)
            {
                val = -1.0 * evaluateSwissCheeseLevelSet(vals[0], vals[1], vals[2],
                                                         tLowerCoordBoundsOfDomain,
                                                         tUpperCoordBoundsOfDomain,
                                                         tAverageElemLength);
            }
            else
                val = -1.0;
        }
        double* vals2 = stk::mesh::field_data(tTempField, entity);
        vals2[0] = val;
        values.push_back(val);
    }

    /*
    size_t results_output_index = broker->create_output_mesh("swisscheese.exo", stk::io::WRITE_RESULTS);
    broker->add_field(results_output_index, tTempField);

    broker->begin_output_step(results_output_index, 0);
    broker->write_defined_output_fields(results_output_index);
    broker->end_output_step(results_output_index);

    broker->write_output_mesh(results_output_index);
    */



    delete bulk_data;
    delete meta_data;
    delete broker;
#else
    throw Plato::LogicException("Functionality not available.  Recompile with STK enabled.");
#endif // STK_ENABLED
}

/******************************************************************************/
void PlatoApp::InitializeField::getInitialValuesForPrimitivesLevelSet(DistributedVector &field, std::vector<double> &values)
/******************************************************************************/
{
#ifdef STK_ENABLED
    stk::io::StkMeshIoBroker *broker = new stk::io::StkMeshIoBroker(mPlatoApp->mLocalComm);
    stk::mesh::MetaData *meta_data = new stk::mesh::MetaData;
    stk::mesh::BulkData *bulk_data = new stk::mesh::BulkData(*meta_data, mPlatoApp->mLocalComm);
    broker->set_bulk_data(*bulk_data);

    broker->set_option_to_not_collapse_sequenced_fields();
    broker->property_add(Ioss::Property("LOWER_CASE_VARIABLE_NAMES", "OFF"));
    broker->add_mesh_database(m_fileName, "exodus", stk::io::READ_MESH);
    broker->create_input_mesh();

    broker->populate_bulk_data();

    stk::mesh::Field<double, stk::mesh::Cartesian> *tCoordsField = meta_data->get_field<stk::mesh::Field<double, stk::mesh::Cartesian> >
                              (stk::topology::NODE_RANK, "coordinates");

    // Hard code 4 plane values (brick)
//    double p[6][3] = {{-5.25,.1875,.1875},{-5.25,.1875,.1875},{-5.25,-.1875,-.1875},{-5.25,-.1875,-.1875},{-5.25,.1875,.1875},{-.25,-.1875,-.1875}};
    double p[6][3] = {{m_minCoords[0],m_maxCoords[1],m_maxCoords[2]},{m_minCoords[0],m_maxCoords[1],m_maxCoords[2]},
                      {m_minCoords[0],m_minCoords[1],m_minCoords[2]},{m_minCoords[0],m_minCoords[1],m_minCoords[2]},
                      {m_minCoords[0],m_maxCoords[1],m_maxCoords[2]},{m_maxCoords[0],m_minCoords[1],m_minCoords[2]}};
    double n[6][3] = {{0,-1,0},{0,0,-1},{0,1,0},{0,0,1},{1,0,0},{-1,0,0}};

    int length = field.MyLength();
    for(int i=0; i<length; ++i)
    {
        int global_id = field.getAssemblyEpetraVector()->Map().GID(i);
        stk::mesh::Entity entity = bulk_data->get_entity(stk::topology::NODE_RANK, global_id);
        double* cur_coords = stk::mesh::field_data(*tCoordsField, entity);
        double all_dots[6];
        bool all_dots_positive = true;
        for(int j=0; j<6; j++)
        {
            double vec[3];
            for(int k=0; k<3; k++)
                vec[k] = cur_coords[k]-p[j][k];
            double dot = 0.0;
            for(int k=0; k<3; k++)
                dot += vec[k]*n[j][k];
            all_dots[j] = dot;
            if(dot < 0.0)
                all_dots_positive = false;
        }
        if(all_dots_positive == true) // point is inside brick
        {
            double smallest_dot = 9999999.;
            // find smallest dot and use as value
            for(int j=0; j<6; j++)
            {
                if(all_dots[j] < smallest_dot)
                    smallest_dot = all_dots[j];
            }
            values.push_back(-1.0*smallest_dot);
        }
        else
        {
            // for all the planes this had a negative dot with project
            // the point to that plane.  This will give us the point on
            // the brick closest to the original point and then we can caluculate
            // a distance.
            double new_coords[3] = {cur_coords[0],cur_coords[1],cur_coords[2]};
            for(int j=0; j<6; j++)
            {
                if(all_dots[j] < 0.0)
                {
                    for(int k=0; k<3; ++k)
                        new_coords[k] -= all_dots[j] * n[j][k];
                }
            }
            double dist = sqrt((cur_coords[0]-new_coords[0])*(cur_coords[0]-new_coords[0]) +
                               (cur_coords[1]-new_coords[1])*(cur_coords[1]-new_coords[1]) +
                               (cur_coords[2]-new_coords[2])*(cur_coords[2]-new_coords[2]));
            values.push_back(dist);
        }
    }

    delete bulk_data;
    delete meta_data;
    delete broker;
#else
    throw Plato::LogicException("Functionality not available.  Recompile with STK enabled.");
#endif // STK_ENABLED
}

/******************************************************************************/
void PlatoApp::InitializeField::getInitialValuesForRestart(DistributedVector &field, std::vector<double> &values)
/******************************************************************************/
{
#ifdef STK_ENABLED
    bool input_file_is_spread = true;

    stk::io::StkMeshIoBroker *broker = new stk::io::StkMeshIoBroker(mPlatoApp->mLocalComm);
    stk::mesh::MetaData *meta_data = new stk::mesh::MetaData;
    stk::mesh::BulkData *bulk_data = new stk::mesh::BulkData(*meta_data, mPlatoApp->mLocalComm);
    broker->set_bulk_data(*bulk_data);

    broker->set_option_to_not_collapse_sequenced_fields();
    if(!input_file_is_spread)
        broker->property_add(Ioss::Property("DECOMPOSITION_METHOD", "RIB"));
    broker->property_add(Ioss::Property("LOWER_CASE_VARIABLE_NAMES", "OFF"));
    broker->add_mesh_database(m_fileName, "exodus", stk::io::READ_MESH);
    broker->create_input_mesh();

    broker->add_all_mesh_fields_as_input_fields();

    broker->populate_bulk_data();
    stk::mesh::Field<double> *iso_field;


    iso_field = meta_data->get_field<stk::mesh::Field<double> >(stk::topology::NODE_RANK, m_variableName);

    if(m_iteration == -1)
        m_iteration = broker->get_input_io_region().get()->get_property("state_count").get_int();
    broker->read_defined_input_fields(m_iteration);

    // This code right here makes the assumption that the mesh we are reading the values (restart mesh) from has the exact
    // same decomposition as the mesh we are using as input to this plato run (the input mesh).  It is also
    // taking advantage of the fact that when we load the result mesh into a stk_mesh the field has a local
    // ordering that is the same as the local ordering of the field we are loading--which ordering came from
    // the reading in of the input mesh into LightMP.
    int length = field.MyLength();
    for(int i=0; i<length; ++i)
    {
        int global_id = field.getAssemblyEpetraVector()->Map().GID(i);
        stk::mesh::Entity entity = bulk_data->get_entity(stk::topology::NODE_RANK, global_id);
        double* vals = stk::mesh::field_data(*iso_field, entity);
        values.push_back(*vals);
    }

    delete bulk_data;
    delete meta_data;
    delete broker;

#else
    throw Plato::LogicException("Functionality not available.  Recompile with STK enabled.");
#endif // STK_ENABLED
}

/******************************************************************************/
PlatoApp::ComputeVolume::ComputeVolume(PlatoApp* p, Plato::InputData& node) :
        LocalOp(p),
        m_topologyName("Topology"),
        m_volumeName("Volume"),
        m_gradientName("Volume Gradient"),
        m_penaltyModel(nullptr)
/******************************************************************************/
{
  Plato::PenaltyModelFactory pmFactory;
  m_penaltyModel = pmFactory.create(node);
}

/******************************************************************************/
PlatoApp::ComputeVolume::~ComputeVolume()
/******************************************************************************/
{
    if(m_penaltyModel)
    {
        delete m_penaltyModel;
        m_penaltyModel = nullptr;
    }
}


PlatoApp::UpdateProblem::UpdateProblem(PlatoApp* p, Plato::InputData& node) :
        LocalOp(p)
{
}
PlatoApp::UpdateProblem::~UpdateProblem()
{
}
void PlatoApp::UpdateProblem::operator()()
{
    // update filter
    Plato::AbstractFilter* filter = mPlatoApp->getFilter();
    filter->advance_continuation();

    // update other portions of the problem here
}
void PlatoApp::UpdateProblem::getArguments(std::vector<LocalArg>& aLocalArgs)
{
}

/******************************************************************************/
Plato::data::layout_t 
getLayout(Plato::InputData& aNode, Plato::data::layout_t aDefaultLayout)
/******************************************************************************/
{
  auto layoutStr = Plato::Get::String(aNode, "Layout");
  Plato::data::layout_t layout = aDefaultLayout;
  if(!layoutStr.empty()){
    layout = getLayout(layoutStr);
  }
  return layout;
}

/******************************************************************************/
Plato::data::layout_t 
getLayout(Plato::InputData& aNode)
/******************************************************************************/
{
  auto layoutStr = Plato::Get::String(aNode, "Layout");
  if( layoutStr.empty() )
  {
    Plato::ParsingException pe(" PlatoApp: required 'Layout' not specified");
    throw pe;
  } 
  return getLayout(layoutStr);
}

/******************************************************************************/
Plato::data::layout_t getLayout(const std::string & aLayoutStr)
/******************************************************************************/
{

    Plato::data::layout_t tLayout;
    if(aLayoutStr == "Nodal Field")
    {
        tLayout = Plato::data::layout_t::SCALAR_FIELD;
    }
    else if(aLayoutStr == "Element Field")
    {
        tLayout = Plato::data::layout_t::ELEMENT_FIELD;
    }
    else if(aLayoutStr == "Value" || aLayoutStr == "Global" || aLayoutStr == "Scalar" )
    {
        tLayout = Plato::data::layout_t::SCALAR;
    }
    else
    {
        std::stringstream tError;
        tError << std::endl << " PlatoApp: " << std::endl;
        tError << "   Unknown layout specified: '" << aLayoutStr << "'" << std::endl;
        Plato::ParsingException tParsingException(tError.str());
        throw tParsingException;
    }
    return tLayout;
}


/******************************************************************************/
PlatoApp::PlatoMainOutput::PlatoMainOutput(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        LocalOp(aPlatoApp)
/******************************************************************************/
{
    bool tPlotTable = true;
    for(auto tInputNode : aNode.getByName<Plato::InputData>("Input"))
    {
        std::string tName = Plato::Get::String(tInputNode, "ArgumentName");

        auto tInputLayout = getLayout(tInputNode, /*default=*/Plato::data::layout_t::SCALAR_FIELD);

        m_outputData.push_back(LocalArg {tInputLayout, tName, 0, tPlotTable});
    }

    // configure iso surface output
    //
    m_outputFrequency = 5;
    m_outputMethod = 2;
    if(aNode.size<std::string>("OutputFrequency"))
        m_outputFrequency = Plato::Get::Int(aNode, "OutputFrequency");
    if(aNode.size<std::string>("OutputMethod"))
    {
        std::string tMethod = Plato::Get::String(aNode, "OutputMethod");
        if(!tMethod.compare("epu"))
        {
            m_outputMethod = 2;
        }
        else if(!tMethod.compare("parallel write"))
        {
            m_outputMethod = 1;
        }
        else
        {
            m_outputMethod = 2;
        }
    }

    mDiscretization = Plato::Get::String(aNode, "Discretization");

    mWriteRestart   = Plato::Get::Bool(aNode, "WriteRestart");
}

/******************************************************************************/
PlatoApp::PlatoMainOutput::~PlatoMainOutput()
/******************************************************************************/
{
}

/******************************************************************************/
PlatoApp::Filter::Filter(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        LocalOp(aPlatoApp),
        mFilter(),
        m_input_toFilter_name(),
        m_input_baseField_name(),
        m_output_fromFilter_name(),
        m_isGradient()
/******************************************************************************/
{
    // retrieve filter
    mFilter = mPlatoApp->getFilter();

    // decide names differently
    m_isGradient = Plato::Get::Bool(aNode, "Gradient");
    if(m_isGradient)
    {
        m_input_toFilter_name = "Gradient";
        m_input_baseField_name = "Field";
        m_output_fromFilter_name = "Filtered Gradient";
    }
    else
    {
        m_input_toFilter_name = "Field";
        m_input_baseField_name = "";
        m_output_fromFilter_name = "Filtered Field";
    }
}

/******************************************************************************/
PlatoApp::Filter::~Filter()
/******************************************************************************/
{
    mFilter = nullptr;
}

/******************************************************************************/
void PlatoApp::Filter::getArguments(std::vector<LocalArg>& aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_input_toFilter_name});
    if(!m_input_baseField_name.empty())
    {
        aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_input_baseField_name});
    }
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_output_fromFilter_name});
}

/******************************************************************************/
void PlatoApp::Filter::operator()()
/******************************************************************************/
{
    if(mPlatoApp->mTimersTree)
    {
        mPlatoApp->mTimersTree->begin_partition(Plato::timer_partition_t::timer_partition_t::filter);
    }

    // get input data
    auto infield = mPlatoApp->getNodeField(m_input_toFilter_name);
    Real* input_field;
    infield->ExtractView(&input_field);
    auto outfield = mPlatoApp->getNodeField(m_output_fromFilter_name);
    Real* output_field;
    outfield->ExtractView(&output_field);

    // copy input field to output
    const int length = infield->MyLength();
    std::copy(input_field, input_field + length, output_field);

    if(m_isGradient)
    {
        // get base field for gradient application
        auto basefield = mPlatoApp->getNodeField(m_input_baseField_name);
        Real* base_field;
        basefield->ExtractView(&base_field);

        mFilter->apply_on_gradient(length, base_field, output_field);
    }
    else
    {
        mFilter->apply_on_field(length, output_field);
    }

    if(mPlatoApp->mTimersTree)
    {
        mPlatoApp->mTimersTree->end_partition();
    }
}


/******************************************************************************/
PlatoApp::Roughness::Roughness(PlatoApp* aPlatoAppp, Plato::InputData& aNode) :
        LocalOp(aPlatoAppp),
        m_topologyName("Topology"),
        m_roughnessName("Roughness"),
        m_gradientName("Roughness Gradient")
{
}
/******************************************************************************/

/******************************************************************************/
void PlatoApp::Roughness::operator()()
/******************************************************************************/
{
  // Input:  Topology
  // Output: Roughness, Roughness Gradient

  // get local topology
  auto& tTopology = *(mPlatoApp->getNodeField(m_topologyName));

  // get local roughness
  std::vector<double>* tData = mPlatoApp->getValue(m_roughnessName);
  double& tRoughnessValue = (*tData)[0];

  // get local gradient
  auto& tRoughnessGradient = *(mPlatoApp->getNodeField(m_gradientName));

  mPlatoApp->mMeshServices->getRoughness(tTopology, tRoughnessValue, tRoughnessGradient);

  return;
}

/******************************************************************************/
void PlatoApp::Roughness::getArguments(std::vector<LocalArg>& aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_topologyName});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_roughnessName,/*length=*/1});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_gradientName});
}

/******************************************************************************/
PlatoApp::InitializeValues::InitializeValues(PlatoApp* aPlatoAppp, Plato::InputData& aNode) :
        LocalOp(aPlatoAppp),
        m_valuesName("Values")
{
    m_value = Plato::Get::Double(aNode, "InitialValue");
}
/******************************************************************************/

/******************************************************************************/
void PlatoApp::InitializeValues::operator()()
/******************************************************************************/
{
  // Output: Values

  std::vector<double>* tData = mPlatoApp->getValue(m_valuesName);
  (*tData)[0] = m_value;
}

/******************************************************************************/
void PlatoApp::InitializeValues::getArguments(std::vector<LocalArg>& aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_valuesName,/*length=*/1});
}

/******************************************************************************/
PlatoApp::NormalizeObjectiveValue::NormalizeObjectiveValue(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        LocalOp(aPlatoApp)
/******************************************************************************/
{
    Plato::InputData tOutputNode = Plato::Get::InputData(aNode,"Output");
    if( aNode.size<Plato::InputData>("Output") > 1 )
    {
        throw ParsingException("PlatoApp::NormalizeObjectiveValue: more than one Output specified.");
    }
    m_outputName = Plato::Get::String(tOutputNode, "ArgumentName");

    Plato::InputData tInputNode = Plato::Get::InputData(aNode,"Input");
    if( aNode.size<Plato::InputData>("Input") > 1 )
    {
        throw ParsingException("PlatoApp::NormalizeObjectiveValue: more than one Input specified.");
    }
    m_inputName = Plato::Get::String(tInputNode, "ArgumentName");

    Plato::InputData tRefValNode = Plato::Get::InputData(aNode,"ReferenceValue");
    if( aNode.size<Plato::InputData>("ReferenceValue") > 1 )
    {
        throw ParsingException("PlatoApp::NormalizeObjectiveGradient: more than one ReferenceValue specified.");
    }
    m_refValName = Plato::Get::String(tRefValNode, "ArgumentName");
}

/******************************************************************************/
PlatoApp::NormalizeObjectiveGradient::NormalizeObjectiveGradient(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        LocalOp(aPlatoApp)
/******************************************************************************/
{
    Plato::InputData tOutputNode = Plato::Get::InputData(aNode,"Output");
    if( aNode.size<Plato::InputData>("Output") > 1 )
    {
        throw ParsingException("PlatoApp::NormalizeObjectiveGradient: more than one Output specified.");
    }
    m_outputName = Plato::Get::String(tOutputNode, "ArgumentName");

    Plato::InputData tInputNode = Plato::Get::InputData(aNode,"Input");
    if( aNode.size<Plato::InputData>("Input") > 1 )
    {
        throw ParsingException("PlatoApp::NormalizeObjectiveGradient: more than one Input specified.");
    }
    m_inputName = Plato::Get::String(tInputNode, "ArgumentName");

    Plato::InputData tRefValNode = Plato::Get::InputData(aNode,"ReferenceValue");
    if( aNode.size<Plato::InputData>("ReferenceValue") > 1 )
    {
        throw ParsingException("PlatoApp::NormalizeObjectiveGradient: more than one ReferenceValue specified.");
    }
    m_refValName = Plato::Get::String(tRefValNode, "ArgumentName");
}

/******************************************************************************/
PlatoApp::WriteGlobalValue::WriteGlobalValue(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        LocalOp(aPlatoApp)
/******************************************************************************/
{
    Plato::InputData tInputNode = Plato::Get::InputData(aNode,"Input");
    if( aNode.size<Plato::InputData>("Input") > 1 )
    {
        throw ParsingException("PlatoApp::WriteGlobalValue: more than one Input specified.");
    }
    m_inputName = Plato::Get::String(tInputNode, "ArgumentName");

    m_size = Plato::Get::Int(aNode,"Size");
    m_filename = Plato::Get::String(aNode, "Filename");
}

/******************************************************************************/
PlatoApp::ReciprocateObjectiveValue::ReciprocateObjectiveValue(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        LocalOp(aPlatoApp)
/******************************************************************************/
{
    Plato::InputData tOutputNode = Plato::Get::InputData(aNode,"Output");
    if( aNode.size<Plato::InputData>("Output") > 1 )
    {
        throw ParsingException("PlatoApp::ReciprocateObjectiveValue: more than one Output specified.");
    }
    m_outputName = Plato::Get::String(tOutputNode, "ArgumentName");

    Plato::InputData tInputNode = Plato::Get::InputData(aNode,"Input");
    if( aNode.size<Plato::InputData>("Input") > 1 )
    {
        throw ParsingException("PlatoApp::ReciprocateObjectiveValue: more than one Input specified.");
    }
    m_inputName = Plato::Get::String(tInputNode, "ArgumentName");
}

/******************************************************************************/
PlatoApp::ReciprocateObjectiveGradient::ReciprocateObjectiveGradient(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        LocalOp(aPlatoApp)
/******************************************************************************/
{
    Plato::InputData tOutputNode = Plato::Get::InputData(aNode,"Output");
    if( aNode.size<Plato::InputData>("Output") > 1 )
    {
        throw ParsingException("PlatoApp::ReciprocateObjectiveGradient: more than one Output specified.");
    }
    m_outputName = Plato::Get::String(tOutputNode, "ArgumentName");

    Plato::InputData tInputNode = Plato::Get::InputData(aNode,"Input");
    if( aNode.size<Plato::InputData>("Input") > 1 )
    {
        throw ParsingException("PlatoApp::ReciprocateObjectiveGradient: more than one Input specified.");
    }
    m_inputName = Plato::Get::String(tInputNode, "ArgumentName");

    Plato::InputData tRefValNode = Plato::Get::InputData(aNode,"ObjectiveValue");
    if( aNode.size<Plato::InputData>("ObjectiveValue") > 1 )
    {
        throw ParsingException("PlatoApp::ReciprocateObjectiveGradient: more than one ObjectiveValue specified.");
    }
    m_refValName = Plato::Get::String(tRefValNode, "ArgumentName");
}

/******************************************************************************/
PlatoApp::Aggregator::Aggregator(PlatoApp* aPlatoApp, Plato::InputData& aNode) :
        LocalOp(aPlatoApp)
/******************************************************************************/
{
    Plato::InputData tWeightNode = Plato::Get::InputData(aNode,"Weighting");
    for(auto tNode : tWeightNode.getByName<Plato::InputData>("Weight"))
    {
        m_weights.push_back(Plato::Get::Double(tNode, "Value"));
    }
    m_weightMethod = Plato::Get::String(tWeightNode, "Method",/*toUpper=*/true);
    if(m_weightMethod == "FIXED")
    {
        m_limitWeight = Plato::Get::Double(tWeightNode, "Limit");
        if(m_limitWeight == 0.0) m_limitWeight = 1e9;
        Plato::InputData tBasesNode = Plato::Get::InputData(tWeightNode,"Bases");
        for(auto tInputNode : tBasesNode.getByName<Plato::InputData>("Input"))
        {
            std::string tBasisName = Plato::Get::String(tInputNode, "ArgumentName");
            m_weightBases.push_back(tBasisName);
        }
    }
    Plato::InputData tNormalsNode = Plato::Get::InputData(tWeightNode,"Normals");
    for(auto tInputNode : tNormalsNode.getByName<Plato::InputData>("Input"))
    {
        std::string tNormalName = Plato::Get::String(tInputNode, "ArgumentName");
        m_weightNormals.push_back(tNormalName);
    }
    // TODO check that number of weights == number of fields, etc.

    for(Plato::InputData tNode : aNode.getByName<Plato::InputData>("Aggregate"))
    {
        AggStruct tNewAggStruct;
        Plato::InputData tOutputNode = Plato::Get::InputData(tNode,"Output");
        if( tNode.size<Plato::InputData>("Output") > 1 )
        {
            throw ParsingException("PlatoApp::Aggregator: more than one Output specified.");
        }
        tNewAggStruct.outputName = Plato::Get::String(tOutputNode, "ArgumentName");

        for(Plato::InputData tInputNode : tNode.getByName<Plato::InputData>("Input"))
        {
            tNewAggStruct.inputNames.push_back(Plato::Get::String(tInputNode, "ArgumentName"));
        }
        tNewAggStruct.layout = getLayout(tNode);

        m_aggStructs.push_back(tNewAggStruct);
    }
}

/******************************************************************************/
void PlatoApp::importData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
/******************************************************************************/
{
    importDataT(aArgumentName, aImportData);
}

/******************************************************************************/
void PlatoApp::exportData(const std::string & aArgumentName, Plato::SharedData & aExportData)
/******************************************************************************/
{
    exportDataT(aArgumentName, aExportData);
}

/******************************************************************************/
void PlatoApp::exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs)
/******************************************************************************/
{
    aMyOwnedGlobalIDs.clear();

    if(aDataLayout == Plato::data::layout_t::VECTOR_FIELD)
    {
        // Plato::data::layout_t = VECTOR_FIELD
        int tMyNumElements = mSysGraph->getRowMap()->NumMyElements();
        aMyOwnedGlobalIDs.resize(tMyNumElements);
        mSysGraph->getRowMap()->MyGlobalElements(aMyOwnedGlobalIDs.data());
    }
    else if(aDataLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        // Plato::data::layout_t = SCALAR_FIELD
        int tMyNumElements = mSysGraph->getNodeRowMap()->NumMyElements();
        aMyOwnedGlobalIDs.resize(tMyNumElements);
        mSysGraph->getNodeRowMap()->MyGlobalElements(aMyOwnedGlobalIDs.data());
    }
    else if(aDataLayout == Plato::data::layout_t::ELEMENT_FIELD)
    {
        int tMyNumElements = mLightMp->getMesh()->getNumElems();
        int* tElemGlobalIds = mLightMp->getMesh()->elemGlobalIds;
        aMyOwnedGlobalIDs.resize(tMyNumElements);
        std::copy(tElemGlobalIds, tElemGlobalIds + tMyNumElements, aMyOwnedGlobalIDs.begin());
    }
    else
    {
        // TODO: THROW
    }
}

/******************************************************************************/
void PlatoApp::initialize()
/******************************************************************************/
{
    // conditionally begin timers
    if(mAppfileData.size<Plato::InputData>("Timers"))
    {
        auto tTimersNode = mAppfileData.get<Plato::InputData>("Timers");
        if(tTimersNode.size<std::string>("time") > 0)
        {
            const bool do_time = Plato::Get::Bool(tTimersNode, "time");
            if(do_time)
            {
                mTimersTree = new Plato::TimersTree(mLocalComm);
            }
        }
    }

    // Define system graph and mesh services (e.g. output) for problems with shared data fields (mesh-based fields)
    const int tDofsPerNode = 1;
    if(mLightMp != nullptr)
    {
        mMeshServices = new MeshServices(mLightMp->getMesh());
        mSysGraph = new SystemContainer(mLightMp->getMesh(), tDofsPerNode, mLightMp->getInput());
    }

    // If PlatoApp operations file is defined, parse/create requested operations
    if( !mAppfileData.empty() )
    {
        for( auto tNode : mAppfileData.getByName<Plato::InputData>("Operation") )
        {
            std::string tStrName = Plato::Get::String(tNode, "Name");
            std::string tStrFunction = Plato::Get::String(tNode, "Function");

            std::vector<std::string> tFunctions;

            tFunctions.push_back("NormalizeObjectiveValue");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new NormalizeObjectiveValue(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("NormalizeObjectiveGradient");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new NormalizeObjectiveGradient(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("WriteGlobalValue");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new WriteGlobalValue(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("ReciprocateObjectiveValue");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new ReciprocateObjectiveValue(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("ReciprocateObjectiveGradient");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new ReciprocateObjectiveGradient(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("Aggregator");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new Aggregator(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("EnforceBounds");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new EnforceBounds(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("PlatoMainOutput");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new PlatoMainOutput(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("SetLowerBounds");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new SetLowerBounds(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("SetUpperBounds");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new SetUpperBounds(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("Filter");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new Filter(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("ComputeMLSField");
            if(tStrFunction == tFunctions.back())
            {
  #ifdef GEOMETRY
                auto tMLSName = Plato::Get::String(tNode,"MLSName");
                if( mMLS.count(tMLSName) == 0 )
                { throw ParsingException("PlatoApp::ComputeMLSField: Requested a PointArray that isn't defined."); }

                auto tMLS = mMLS[tMLSName];
                if( tMLS->dimension == 3 ) { mOperationMap[tStrName] = new ComputeMLSField<3>(this, tNode); }
                else
                if( tMLS->dimension == 2 ) { mOperationMap[tStrName] = new ComputeMLSField<2>(this, tNode); }
                else
                if( tMLS->dimension == 1 ) { mOperationMap[tStrName] = new ComputeMLSField<1>(this, tNode); }
                this->createLocalData(mOperationMap[tStrName]);
                continue;
  #else 
                throw ParsingException("PlatoApp was not compiled with ComputeMLSField enabled.  Turn on 'GEOMETRY' option and rebuild.");
  #endif // GEOMETRY
            }

            tFunctions.push_back("InitializeMLSPoints");
            if(tStrFunction == tFunctions.back())
            {
  #ifdef GEOMETRY
                auto tMLSName = Plato::Get::String(tNode,"MLSName");
                if( mMLS.count(tMLSName) == 0 )
                { throw ParsingException("PlatoApp::InitializeMLSPoints: Requested a PointArray that isn't defined."); }

                auto tMLS = mMLS[tMLSName];
                if( tMLS->dimension == 3 ) { mOperationMap[tStrName] = new InitializeMLSPoints<3>(this, tNode); }
                else
                if( tMLS->dimension == 2 ) { mOperationMap[tStrName] = new InitializeMLSPoints<2>(this, tNode); }
                else
                if( tMLS->dimension == 1 ) { mOperationMap[tStrName] = new InitializeMLSPoints<1>(this, tNode); }
                this->createLocalData(mOperationMap[tStrName]);
                continue;
  #else 
                throw ParsingException("PlatoApp was not compiled with InitializeMLSPoints enabled.  Turn on 'GEOMETRY' option and rebuild.");
  #endif // GEOMETRY
            }

            tFunctions.push_back("MapMLSField");
            if(tStrFunction == tFunctions.back())
            {
  #ifdef GEOMETRY
                auto tMLSName = Plato::Get::String(tNode,"MLSName");
                if( mMLS.count(tMLSName) == 0 )
                { throw ParsingException("PlatoApp::MapMLSField: Requested a PointArray that isn't defined."); }

                auto tMLS = mMLS[tMLSName];
                if( tMLS->dimension == 3 ) { mOperationMap[tStrName] = new MapMLSField<3>(this, tNode); }
                else
                if( tMLS->dimension == 2 ) { mOperationMap[tStrName] = new MapMLSField<2>(this, tNode); }
                else
                if( tMLS->dimension == 1 ) { mOperationMap[tStrName] = new MapMLSField<1>(this, tNode); }
                this->createLocalData(mOperationMap[tStrName]);
                continue;
  #else 
                throw ParsingException("PlatoApp was not compiled with MapMLSField enabled.  Turn on 'GEOMETRY' option and rebuild.");
  #endif // GEOMETRY
            }


            tFunctions.push_back("ComputeRoughness");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new Roughness(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("InitializeValues");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new InitializeValues(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("ComputeVolume");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new ComputeVolume(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("DesignVolume");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new DesignVolume(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("InitializeField");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new InitializeField(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            tFunctions.push_back("Update Problem");
            if(tStrFunction == tFunctions.back())
            {
                mOperationMap[tStrName] = new UpdateProblem(this, tNode);
                this->createLocalData(mOperationMap[tStrName]);
                continue;
            }

            std::stringstream tMessage;
            tMessage << "Cannot find specified Function: " << tStrFunction << std::endl;
            tMessage << "Available Functions: " << std::endl;
            for(auto tMyFunction : tFunctions)
            {
                tMessage << tMyFunction << std::endl;
            }
            Plato::ParsingException tParsingException(tMessage.str());
            throw tParsingException;
        } // Loop over XML nodes
    } // Check f operation file is defined

    // If mesh services were requested by the user, finalize mesh services setup.
    if(mLightMp != nullptr)
    {
        mLightMp->setWriteTimeStepDuringSetup(false);
        mLightMp->finalizeSetup();
    }
}

/******************************************************************************/
void
PlatoApp::createLocalData(LocalOp* aLocalOperation)
/******************************************************************************/
{
    std::vector<LocalArg> tLocalArgs;
    aLocalOperation->getArguments(tLocalArgs);
    for(auto tArgument : tLocalArgs)
    {
        createLocalData(tArgument);
    }
}

/******************************************************************************/
void PlatoApp::createLocalData(LocalArg aLocalArguments)
/******************************************************************************/
{
    if(aLocalArguments.mLayout == Plato::data::layout_t::SCALAR_FIELD)
    {
        auto tIterator = mNodeFieldMap.find(aLocalArguments.mName);
        if(tIterator != mNodeFieldMap.end())
        {
            return;
        }
        std::vector<VarIndex> tNewData(1);
        tNewData[0] = mLightMp->getDataContainer()->registerVariable(RealType, aLocalArguments.mName, NODE, aLocalArguments.mWrite);
        mNodeFieldMap[aLocalArguments.mName] = new DistributedVector(mSysGraph, tNewData);
    }
    else if(aLocalArguments.mLayout == Plato::data::layout_t::ELEMENT_FIELD)
    {
        auto tIterator = mElementFieldMap.find(aLocalArguments.mName);
        if(tIterator != mElementFieldMap.end())
        {
            return;
        }
        VarIndex tNewDataIndex;
        tNewDataIndex = mLightMp->getDataContainer()->registerVariable(RealType, aLocalArguments.mName, ELEM, aLocalArguments.mWrite);
        mElementFieldMap[aLocalArguments.mName] = tNewDataIndex;
    }
    else if(aLocalArguments.mLayout == Plato::data::layout_t::SCALAR)
    {
        auto tIterator = mValueMap.find(aLocalArguments.mName);
        if(tIterator != mValueMap.end())
        {
            return;
        }
        std::vector<double>* tNewData = new std::vector<double>(aLocalArguments.mLength);
        mValueMap[aLocalArguments.mName] = tNewData;
    }
}


/******************************************************************************/
void PlatoApp::compute(const std::string & aOperationName)
/******************************************************************************/
{
  auto it = mOperationMap.find(aOperationName);
  if(it == mOperationMap.end()){
    std::stringstream ss;
    ss << "Request for operation ('" << aOperationName << "') that doesn't exist.";
    throw Plato::LogicException(ss.str());
  }
  
  (*mOperationMap[aOperationName])();
}

/******************************************************************************/
void PlatoApp::finalize()
/******************************************************************************/
{
}

/******************************************************************************/
PlatoApp::~PlatoApp()
/******************************************************************************/
{
    if(mLightMp)
    {
        delete mLightMp;
        mLightMp = nullptr;
    }
    if(mSysGraph)
    {
        delete mSysGraph;
        mSysGraph = nullptr;
    }
    if(mMeshServices)
    {
        delete mMeshServices;
        mMeshServices = nullptr;
    }
    for(auto& tMyValue : mValueMap)
    {
        delete tMyValue.second;
    }
    mValueMap.clear();
    for(auto& tMyField : mNodeFieldMap)
    {
        delete tMyField.second;
    }
    mNodeFieldMap.clear();
    for(auto& tMyOperation : mOperationMap)
    {
        delete tMyOperation.second;
    }
    mOperationMap.clear();
    if(mFilter)
    {
        delete mFilter;
        mFilter = nullptr;
    }
    if(mTimersTree)
    {
        mTimersTree->print_results();
        delete mTimersTree;
        mTimersTree = nullptr;
    }
}

/******************************************************************************/
void PlatoApp::NormalizeObjectiveValue::operator()() 
/******************************************************************************/
{
    std::vector<double>& toData    = *(mPlatoApp->getValue(m_outputName));
    std::vector<double>& fromData  = *(mPlatoApp->getValue(m_inputName));
    std::vector<double>& refValVec = *(mPlatoApp->getValue(m_refValName));
    double refVal = refValVec[0];

    for(unsigned int i=0; i<toData.size(); i++)
    {
      toData[i] = fromData[i] / refVal;
    }
}

/******************************************************************************/
void PlatoApp::NormalizeObjectiveGradient::operator()() 
/******************************************************************************/
{
    auto& inField  = *(mPlatoApp->getNodeField(m_inputName));
    double* inData;
    inField.ExtractView(&inData);

    auto& outField = *(mPlatoApp->getNodeField(m_outputName));
    double* outData;
    outField.ExtractView(&outData);

    std::vector<double>& refValVec = *(mPlatoApp->getValue(m_refValName));
    double refVal = refValVec[0];

    int dataLen = inField.MyLength();
    for(int i=0; i<dataLen; i++)
    {
      outData[i] = inData[i] / refVal;
    }
}

/******************************************************************************/
void PlatoApp::WriteGlobalValue::operator()() 
/******************************************************************************/
{
    int my_rank = 0;
    MPI_Comm_rank(mPlatoApp->mLocalComm, &my_rank);
    if(my_rank == 0)
    {
        fstream outfile;
        outfile.open(m_filename, std::ios::app);
        std::vector<double>& fromData = *(mPlatoApp->getValue(m_inputName));

        for(unsigned int i=0; i<fromData.size(); i++)
        {
          outfile << std::setprecision(8) << fromData[i] << " ";
        }
        outfile << std::endl;
        outfile.close();
    }
}

/******************************************************************************/
void PlatoApp::ReciprocateObjectiveValue::operator()() 
/******************************************************************************/
{
    std::vector<double>& toData   = *(mPlatoApp->getValue(m_outputName));
    std::vector<double>& fromData = *(mPlatoApp->getValue(m_inputName));

    for(unsigned int i=0; i<toData.size(); i++)
    {
      toData[i] = fromData[i] != 0 ? 1.0/fromData[i] : 0.0;
    }
}

/******************************************************************************/
void PlatoApp::ReciprocateObjectiveGradient::operator()() 
/******************************************************************************/
{
    auto& inField  = *(mPlatoApp->getNodeField(m_inputName));
    double* inData;
    inField.ExtractView(&inData);

    auto& outField = *(mPlatoApp->getNodeField(m_outputName));
    double* outData;
    outField.ExtractView(&outData);


    std::vector<double>& refValVec = *(mPlatoApp->getValue(m_refValName));
    double refVal = refValVec[0];

    int dataLen = inField.MyLength();
    double factor = refVal!=0 ? -1.0/(refVal*refVal) : 1.0;
    for(int i=0; i<dataLen; i++)
    {
      outData[i] = factor*inData[i];
    }
}

/******************************************************************************/
void PlatoApp::Aggregator::operator()() 
/******************************************************************************/
{
    // begin timer if timing
    if(mPlatoApp->mTimersTree)
    {
        mPlatoApp->mTimersTree->begin_partition(Plato::timer_partition_t::timer_partition_t::aggregator);
    }

  std::vector<double> weights(m_weights);
  if(!m_weightBases.empty()){
    int nvals = m_weightBases.size();
    std::vector<double> B(nvals);
    for(int ival=0; ival<nvals; ival++){
      std::vector<double>* data = mPlatoApp->getValue(m_weightBases[ival]);
      B[ival] = *(data->data());
    }
    double etaSum = 0.0;
    for(int ival=1; ival<nvals; ival++){
      etaSum += m_weights[ival];
    }
    if(!m_weightNormals.empty()){
      nvals = m_weightNormals.size();
      std::vector<double> N(nvals);
      for(int ival=0; ival<nvals; ival++){
        std::vector<double>* data = mPlatoApp->getValue(m_weightNormals[ival]);
        N[ival] = *(data->data());
      }
      for(int ival=0; ival<nvals; ival++){
        weights[ival] = B[0]*m_weights[ival]/(B[ival]*(1.0-etaSum));
        weights[ival] *= N[ival]/N[0];
        if(weights[ival] > m_limitWeight) weights[ival] = m_limitWeight;
      }
    } else {
      for(int ival=0; ival<nvals; ival++){
        weights[ival] = B[0]*m_weights[ival]/(B[ival]*(1.0-etaSum));
        if(weights[ival] > m_limitWeight) weights[ival] = m_limitWeight;
      }
    }
  } else {
    if(!m_weightNormals.empty()){
      int nvals = m_weightNormals.size();
      std::vector<double> N(nvals);
      for(int ival=0; ival<nvals; ival++){
        std::vector<double>* data = mPlatoApp->getValue(m_weightNormals[ival]);
        N[ival] = *(data->data());
      }
      for(int ival=0; ival<nvals; ival++){
        weights[ival] /= N[ival];
      }
    }
  }

  for(AggStruct& agg : m_aggStructs){

    if(agg.layout == Plato::data::layout_t::SCALAR_FIELD){

      auto& field = *(mPlatoApp->getNodeField(agg.outputName));
      double* toData; field.ExtractView(&toData);
      int dataLen = field.MyLength();
  
      int nvals = agg.inputNames.size();
      std::vector<double*> fromData(nvals);
      for(int ival=0; ival<nvals; ival++){
        auto pfield = mPlatoApp->getNodeField(agg.inputNames[ival]);
        pfield->ExtractView(&fromData[ival]);
      }
  
      for(int i=0; i<dataLen; i++){
        toData[i] = 0.0;
        for(int j=0; j<nvals; j++){
          toData[i] += fromData[j][i]*weights[j];
        }
      }

    } else
    if(agg.layout == Plato::data::layout_t::SCALAR){

      std::vector<double>& toData = *(mPlatoApp->getValue(agg.outputName));
  
      unsigned int dataLen = 0;
      int nvals = agg.inputNames.size();
      std::vector<double*> fromData(nvals);

      // read first input value
      std::vector<double>* v = mPlatoApp->getValue(agg.inputNames[0]);
      fromData[0] = v->data();
      dataLen = v->size();
   
      // read remaining input values
      for(int ival=1; ival<nvals; ival++){
        v = mPlatoApp->getValue(agg.inputNames[ival]);
        fromData[ival] = v->data();
        if( v->size() != dataLen ){
          throw ParsingException("PlatoApp::Aggregator: attempted to aggregate vectors of differing lengths.");
        }
      }
  
      toData.resize(dataLen);
      for(unsigned int i=0; i<dataLen; i++){
        toData[i] = 0.0;
        for(int j=0; j<nvals; j++){
          toData[i] += fromData[j][i]*weights[j];
        }
      }

    } else {
 
      std::stringstream message;
      message << "Unknown 'Layout' (" << agg.layout << ") specified in PlatoApp::Aggregator." << std::endl;
      Plato::ParsingException pe(message.str());
      throw pe;
    }
  }

  // end: "aggregator"
  if(mPlatoApp->mTimersTree)
    {
        mPlatoApp->mTimersTree->end_partition();
    }

  return;
}

/******************************************************************************/
void PlatoApp::NormalizeObjectiveValue::getArguments(std::vector<LocalArg> & aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_refValName,/*length=*/1});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_outputName,/*length=*/1});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_inputName, /*length=*/1});
}

/******************************************************************************/
void PlatoApp::NormalizeObjectiveGradient::getArguments(std::vector<LocalArg> & aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_refValName,/*length=*/1});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_outputName});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_inputName});
}

/******************************************************************************/
void PlatoApp::WriteGlobalValue::getArguments(std::vector<LocalArg> & aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_inputName, m_size});
}

/******************************************************************************/
void PlatoApp::ReciprocateObjectiveValue::getArguments(std::vector<LocalArg> & aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_outputName,/*length=*/1});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_inputName, /*length=*/1});
}

/******************************************************************************/
void PlatoApp::ReciprocateObjectiveGradient::getArguments(std::vector<LocalArg> & aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_refValName,/*length=*/1});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_outputName});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_inputName});
}

/******************************************************************************/
void PlatoApp::Aggregator::getArguments(std::vector<LocalArg> & aLocalArgs)
/******************************************************************************/
{
    for(auto& tArgument : m_aggStructs)
    {
        aLocalArgs.push_back(LocalArg {tArgument.layout, tArgument.outputName});
        for(auto& tInput : tArgument.inputNames)
        {
            aLocalArgs.push_back(LocalArg {tArgument.layout, tInput});
        }
    }
    for(auto& tArgument : m_weightBases)
    {
        aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, tArgument});
    }
    for(auto& tArgument : m_weightNormals)
    {
        aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, tArgument});
    }
}

/******************************************************************************/
void PlatoApp::PlatoMainOutput::getArguments(std::vector<LocalArg>& aLocalArgs)
/******************************************************************************/
{
  aLocalArgs = m_outputData;
}

/******************************************************************************/
void PlatoApp::PlatoMainOutput::extract_iso_surface(int aIteration)
/******************************************************************************/
{
#ifdef ENABLE_ISO
    std::string output_filename = "";
    char tmp_str[200];
    if(aIteration < 10)
        sprintf(tmp_str, "00%d", aIteration);
    else if(aIteration < 100)
        sprintf(tmp_str, "0%d", aIteration);
    else if(aIteration < 1000)
        sprintf(tmp_str, "%d", aIteration);
    output_filename = "Iteration";
    output_filename += tmp_str;
    output_filename += ".exo";
    iso::STKExtract ex;
    std::string input_filename = "platomain.exo";
    int num_procs = 0;
    MPI_Comm_size(mPlatoApp->mLocalComm, &num_procs);
    if(num_procs == 1)
        input_filename += ".1.0";

    std::string tOutputFields = "";
    for(size_t i=0; i<m_outputData.size(); ++i)
    {
        tOutputFields += m_outputData[i].mName;
        if(i < (m_outputData.size()-1))
        {
            tOutputFields += ",";
        }
    }
    if(ex.create_mesh_apis_read_from_file(&mPlatoApp->mLocalComm, // MPI_Comm
                                          input_filename,         // input filename
                                          output_filename,        // output filename
                                          "Topology",             // iso field name
                                          tOutputFields,          // names of fields to output
                                          1e-5,                   // min edge length
                                          0.5,                    // iso value
                                          0,                      // level_set data?
                                          m_outputMethod,         // epu results
                                          1,                      // iso_only
                                          1,                      // read spread file
                                          aIteration))            // time step/iteration
    {
        ex.run_extraction(aIteration, 1);
    }
    int my_rank = 0;
    MPI_Comm_rank(mPlatoApp->mLocalComm, &my_rank);
    if(my_rank == 0)
    {
        FILE *fp = fopen("last_time_step.txt", "w");
        if(fp)
        {
            fprintf(fp, "%s\n", tmp_str);
            fclose(fp);
            system("ls Iteration*.exo >> last_time_step.txt");
        }
    }
#endif
}

/******************************************************************************/
void PlatoApp::PlatoMainOutput::operator()()
/******************************************************************************/
{
    // time operation
    if(mPlatoApp->mTimersTree)
    {
        mPlatoApp->mTimersTree->begin_partition(Plato::timer_partition_t::timer_partition_t::file_input_output);
    }

    LightMP* lmp = mPlatoApp->getLightMP();
    double time = lmp->getCurrentTime();
    time += 1.0;
    lmp->setCurrentTime(time);
    int int_time = (int)time;
    lmp->WriteOutput();
    int tMyRank = 0;
    MPI_Comm_rank(mPlatoApp->mLocalComm, &tMyRank);
    if(m_outputFrequency > 0 && int_time % m_outputFrequency == 0)
    {
        if(mDiscretization == "density")
        {
            extract_iso_surface(int_time);

            // Write restart file
            if((tMyRank == 0) && mWriteRestart)
            {
                std::ostringstream theCommand;
                std::string tInputFilename = "platomain.exo.1.0";
                int tNumProcs = 0;
                MPI_Comm_size(mPlatoApp->mLocalComm, &tNumProcs);
                if(tNumProcs > 1)
                {
                    theCommand << "epu -auto platomain.exo." << tNumProcs << ".0 > epu.txt;";
                    tInputFilename = "platomain.exo";
                }
                theCommand << "echo times " << int_time << " > commands.txt;";
                theCommand << "echo save optimizationdofs >> commands.txt;";
                theCommand << "echo end >> commands.txt;";
                theCommand << "algebra " << tInputFilename << " restart_" << int_time << ".exo < commands.txt > algebra.txt";
                std::cout << "\nExecuting system call: " << theCommand.str() << "\n";
                system(theCommand.str().c_str());
            }
        }
        else if(mDiscretization == "levelset")
        {
            if((tMyRank == 0) && mWriteRestart)
            {
                std::string tListCommand = "ls -t IterationHistory* > junk.txt";
                system(tListCommand.c_str());
                FILE *fp = fopen("junk.txt", "r");
                if(fp)
                {
                    char tLastHistFileName[200] = " ";
                    fscanf(fp, "%s", tLastHistFileName);
                    fclose(fp);
                    std::string tNewFilename = "Iteration";
                    char tmp_str[200];
                    if(int_time < 10)
                        sprintf(tmp_str, "00%d", int_time);
                    else if(int_time < 100)
                        sprintf(tmp_str, "0%d", int_time);
                    else if(int_time < 1000)
                        sprintf(tmp_str, "%d", int_time);
                    tNewFilename += tmp_str;
                    tNewFilename += ".exo";
                    std::string tCopyCommand = "cp ";
                    tCopyCommand += tLastHistFileName;
                    tCopyCommand += " ";
                    tCopyCommand += tNewFilename;
                    system(tCopyCommand.c_str());
                    system("rm -f IterationHistory*");
                    fp = fopen("last_time_step.txt", "w");
                    if(fp)
                    {
                        fprintf(fp, "%s\n", tmp_str);
                        fclose(fp);
                        system("ls Iteration*.exo >> last_time_step.txt");
                    }
                }
            }
        }
    }

    // end I/O timer
    if(mPlatoApp->mTimersTree)
    {
        mPlatoApp->mTimersTree->end_partition();
    }
}

/******************************************************************************/
void PlatoApp::DesignVolume::getArguments(std::vector<LocalArg>& aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_outValueName,/*length=*/1});
}

/******************************************************************************/
PlatoApp::DesignVolume::DesignVolume(PlatoApp* p, Plato::InputData& node) : 
  LocalOp(p), m_outValueName("Design Volume")
/******************************************************************************/
{

}

/******************************************************************************/
void PlatoApp::DesignVolume::operator()()
/******************************************************************************/
{
  double tVolume = mPlatoApp->mMeshServices->getTotalVolume();
  std::vector<double>* tData = mPlatoApp->getValue(m_outValueName);

  (*tData)[0] = tVolume;

  return;
}

/******************************************************************************/
std::vector<double>* PlatoApp::getValue(const std::string & aName)
/******************************************************************************/
{
  auto tIterator = mValueMap.find(aName);
  if(tIterator == mValueMap.end()){ throwParsingException(aName, mValueMap); }
  return tIterator->second;
}

/******************************************************************************/
DistributedVector* PlatoApp::getNodeField(const std::string & aName)
/******************************************************************************/
{
  auto tIterator = mNodeFieldMap.find(aName);
  if(tIterator == mNodeFieldMap.end()){ throwParsingException(aName, mNodeFieldMap); }
  return tIterator->second;
}

/******************************************************************************/
VarIndex PlatoApp::getElementField(const std::string & aName)
/******************************************************************************/
{
  auto tIterator = mElementFieldMap.find(aName);
  if(tIterator == mElementFieldMap.end()){ throwParsingException(aName, mElementFieldMap); }
  return tIterator->second;
}

/******************************************************************************/
template<typename ValueType>
void PlatoApp::throwParsingException(const std::string & aName, const std::map<std::string, ValueType> & aValueMap)
/******************************************************************************/
{
    std::stringstream tMessage;
    tMessage << "Cannot find specified Argument: " << aName.c_str() << std::endl;
    tMessage << "Available Arguments: " << std::endl;
    for(auto tIterator : aValueMap)
    {
        tMessage << tIterator.first << std::endl;
    }
    Plato::ParsingException tParsingException(tMessage.str());
    throw tParsingException;
}

/******************************************************************************/
void PlatoApp::ComputeVolume::getArguments(std::vector<LocalArg>& aLocalArgs)
/******************************************************************************/
{
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_topologyName});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR, m_volumeName,/*length=*/1});
    aLocalArgs.push_back(LocalArg {Plato::data::layout_t::SCALAR_FIELD, m_gradientName});
}

/******************************************************************************/
void PlatoApp::ComputeVolume::operator()()
/******************************************************************************/
{
  // Input:  Topology
  // Output: Volume, Volume Gradient

  // get local topology
  auto& tTopology = *(mPlatoApp->getNodeField(m_topologyName));

  // get local volume
  std::vector<double>* tData = mPlatoApp->getValue(m_volumeName);
  double& tVolumeValue = (*tData)[0];

  // get local gradient
  auto& tVolumeGradient = *(mPlatoApp->getNodeField(m_gradientName));

  mPlatoApp->mMeshServices->getCurrentVolume(tTopology, tVolumeValue, tVolumeGradient);

  return;
}

/******************************************************************************/
LightMP* PlatoApp::getLightMP()
/******************************************************************************/
{
    return mLightMp;
}

/******************************************************************************/
Plato::AbstractFilter* PlatoApp::getFilter()
/******************************************************************************/
{
    if(!mFilter)
    {
        if(mTimersTree)
        {
            mTimersTree->begin_partition(Plato::timer_partition_t::timer_partition_t::filter);
        }
        mFilter = Plato::build_filter(mAppfileData, mLocalComm, mLightMp->getMesh());
        if(mTimersTree)
        {
            mTimersTree->end_partition();
        }
    }
    return mFilter;
}

/******************************************************************************/
PlatoApp::PlatoApp(MPI_Comm& aLocalComm) :
        mLocalComm(aLocalComm),
        mLightMp(nullptr),
        mSysGraph(nullptr),
        mMeshServices(nullptr),
        mFilter(nullptr),
        mAppfileData("Appfile Data"),
        mInputfileData("Inputfile Data"),
        mTimersTree(nullptr)
/******************************************************************************/
{
}

/******************************************************************************/
PlatoApp::PlatoApp(int aArgc, char **aArgv, MPI_Comm& aLocalComm) :
        mLocalComm(aLocalComm),
        mLightMp(nullptr),
        mSysGraph(nullptr),
        mMeshServices(nullptr),
        mFilter(nullptr),
        mAppfileData("Appfile Data"),
        mInputfileData("Inputfile Data"),
        mTimersTree(nullptr)
/******************************************************************************/
{
    const char* input_char = getenv("PLATO_APP_FILE");
    Plato::Parser* parser = new Plato::PugiParser();
    mAppfileData = parser->parseFile(input_char);

    // create the FEM utility object
    std::string tInputfile;
    if(aArgc == 2)
    {
        tInputfile = aArgv[1];
    }
    else
    {
        tInputfile = "platomain.xml";
    }
    mInputfileData = parser->parseFile(tInputfile.c_str());
    mLightMp = new LightMP(tInputfile);

    if (parser)
    {
        delete parser;
        parser = nullptr;
    }

    // parse/create the MLS PointArrays
    auto tPointArrayInputs = mInputfileData.getByName<Plato::InputData>("PointArray");
    for(auto tPointArrayInput=tPointArrayInputs.begin(); tPointArrayInput!=tPointArrayInputs.end(); ++tPointArrayInput)
    {
  #ifdef GEOMETRY
        auto tPointArrayName = Plato::Get::String(*tPointArrayInput,"Name");
        auto tPointArrayDims = Plato::Get::Int(*tPointArrayInput,"Dimensions");
        if( mMLS.count(tPointArrayName) != 0 )
        {
            throw Plato::ParsingException("PointArray names must be unique.");
        }
        else
        {
            if( tPointArrayDims == 1 )
                mMLS[tPointArrayName] = std::make_shared<MLSstruct>(MLSstruct({Plato::any(Plato::Geometry::MovingLeastSquares<1,double>(*tPointArrayInput)),1}));
            else
            if( tPointArrayDims == 2 )
                mMLS[tPointArrayName] = std::make_shared<MLSstruct>(MLSstruct({Plato::any(Plato::Geometry::MovingLeastSquares<2,double>(*tPointArrayInput)),2}));
            else
            if( tPointArrayDims == 3 )
                mMLS[tPointArrayName] = std::make_shared<MLSstruct>(MLSstruct({Plato::any(Plato::Geometry::MovingLeastSquares<3,double>(*tPointArrayInput)),3}));
        }
  #else
        throw ParsingException("PlatoApp was not compiled with PointArray support.  Turn on 'GEOMETRY' option and rebuild.");
  #endif
    }
}

/******************************************************************************/
PlatoApp::PlatoApp(const std::string &aPhysics_XML_File, const std::string &aApp_XML_File, MPI_Comm& aLocalComm) :
        mLocalComm(aLocalComm),
        mSysGraph(nullptr),
        mMeshServices(nullptr),
        mFilter(nullptr),
        mAppfileData("Input Data"),
        mTimersTree(nullptr)
/******************************************************************************/
{

    const char* input_char = getenv("PLATO_APP_FILE");
    Plato::Parser* parser = new Plato::PugiParser();
    mAppfileData = parser->parseFile(input_char);

    std::shared_ptr<pugi::xml_document> tTempDoc = std::make_shared<pugi::xml_document>();
    tTempDoc->load_string(aPhysics_XML_File.c_str());

    mInputfileData = parser->parseFile(aPhysics_XML_File.c_str());
    mLightMp = new LightMP(tTempDoc);
}
