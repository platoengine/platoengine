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

/*!
 * Plato_ESP.hpp
 *
 * Created on: Nov 13, 2019
 *
 */

#ifndef PLATO_ESP_HPP_
#define PLATO_ESP_HPP_

#include <Kokkos_Core.hpp>
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cctype>

extern "C" {
#include "egads.h"
#include "bodyTess.h"
#include "common.h"
#include "OpenCSM.h"
// the 'undef' and 'define' below are a work around.  OpenCSM defines SUCCESS which 
// conflicts with a definition in opempi.
#undef SUCCESS
#define OCSM_SUCCESS 0

/* undocumented EGADS function */
extern int EG_loadTess(ego body, const char *name, ego *tess);
}

#include "Plato_Exceptions.hpp"


namespace Plato {
namespace Geometry {

template <typename ScalarType, typename ScalarVectorType>
class ESP
{
    typedef std::vector<ScalarType> VectorType;
    std::vector<VectorType> mSensitivity;

    std::vector<std::string>  mParameterNames;
   
    std::string  mModelFileName;
    std::string  mTessFileName;
    std::string  mActiveParameterName;
    int          mParameterIndex, mActiveParameterIndex, mBuiltTo;
    ego          mContext;
    void         *mModel;
    modl_T       *mModelT;

    static constexpr int mSpaceDim = 3;

  public:

    ESP(std::string aModelFileName, std::string aTessFileName, int aParameterIndex=-1) :
      mModelFileName(aModelFileName),
      mTessFileName(aTessFileName),
      mParameterIndex(aParameterIndex),
      mActiveParameterIndex(-1),
      mBuiltTo(0),
      mContext(nullptr),
      mModel(nullptr),
      mModelT(nullptr)
    {
        readNames();

        if(mParameterIndex != -1){
          auto tName = mParameterNames[mParameterIndex];
          mParameterNames.resize(1);
          mParameterNames[0] = tName;
        }

        mSensitivity.resize(mParameterNames.size());

        openContext();
        openModel();
        checkModel();
        buildGeometryAndGetBodies();
        tesselate();
    }

    ~ESP()
    {
        safeFreeOCSM();
        safeFreeContext();
    }

    decltype(mModelFileName) getModelFileName(){ return mModelFileName; }
    decltype(mTessFileName)  getTessFileName() { return mTessFileName;  }

    int getNumParameters(){ return mParameterNames.size(); }

    decltype(mSensitivity) getSensitivities(){ return mSensitivity; }

    ScalarType sensitivity(int aParameterIndex, ScalarVectorType aGradientX)
    {
        ScalarType tDfDp(0.0);
        auto& tSensitivity = mSensitivity[aParameterIndex];
        int tNumData = tSensitivity.size();
        for (int k=0; k<tNumData; k++)
        {
            tDfDp += tSensitivity[k]*aGradientX[k];
        }
        return tDfDp;
      
    }

  private:
    void readNames()
    {
        mParameterNames.empty();
        std::ifstream tInputStream;
        tInputStream.open(mModelFileName.c_str());
        if(tInputStream.good())
        {
            std::string tLine, tWord;
            while(std::getline(tInputStream, tLine))
            {
                std::istringstream tStream(tLine, std::istringstream::in);
                while( tStream >> tWord )    
                {
                    for(auto& c : tWord) { c = std::tolower(c); }
                    if (tWord == "despmtr")
                    {
                        tStream >> tWord;
                        mParameterNames.push_back(tWord);
                    }
                }
            }
            tInputStream.close();
        }
    }

    void tesselate()
    {
        /* store the tessellation object in OpenCSM */
        for (int ibody=1; ibody<=mModelT->nbody; ibody++)
        {
            if (mModelT->body[ibody].onstack != 1) continue;
            if (mModelT->body[ibody].botype == OCSM_NULL_BODY) continue;
            ego tBody = mModelT->body[ibody].ebody;
            if (mModelT->body[ibody].etess != NULL)
            {
                EG_deleteObject(mModelT->body[ibody].etess);
            }
            auto tStatus = EG_loadTess(tBody, (char*)mTessFileName.c_str(), &mModelT->body[ibody].etess);
            if (tStatus != EGADS_SUCCESS)
            {
                throwWithError("EG_loadTess failed.");
            }
        }
        int tNumParams = mParameterNames.size();
        for(int iParam=0; iParam<tNumParams; iParam++)
        {
            activateParameterInModel(mParameterNames[iParam]);
            computeSensitivity(mSensitivity[iParam]);
        }
    }


    ScalarType computeSensitivity(VectorType& aDXDp)
    {
        ScalarType tSensitivity(0.0);

        /* clear all then set the parameter & tell OpenCSM */
        ocsmSetVelD(mModel, 0,     0,    0,    0.0);
        ocsmSetVelD(mModel, mActiveParameterIndex, /*irow=*/ 1, /*icol=*/ 1, 1.0);
        int tBuildTo = 0;
        int tNbody   = 0;
        ocsmBuild(mModel, tBuildTo, &mBuiltTo, &tNbody, NULL);
        
        /* retrieve the sensitivities for the active bodies */
        int tNface, tNedge, tNvert, tNtri;
        int *tTris(nullptr);
        ScalarType *tCoords(nullptr);
        verTags *tVtags;
        for (int ibody = 1; ibody <= mModelT->nbody; ibody++)
        {
            if (mModelT->body[ibody].onstack != 1) continue;
            if (mModelT->body[ibody].botype  == OCSM_NULL_BODY) continue;
            auto tStatus = bodyTess(mModelT->body[ibody].etess, &tNface, &tNedge, &tNvert, &tCoords, &tVtags, &tNtri, &tTris);
            if (tStatus != EGADS_SUCCESS)
            {
                throwWithError("bodyTess failed.");
            }

            aDXDp = VectorType(tNvert*mSpaceDim);

            const ScalarType *tPcsens;
      
            /* fill in the Nodes */
            for (int j=0; j<tNvert; j++)
            {
                if (tVtags[j].ptype != 0) continue;
                auto tStatus = ocsmGetTessVel(mModel, ibody, OCSM_NODE, tVtags[j].pindex, &tPcsens);
                if (tStatus != EGADS_SUCCESS)
                {
                    EG_free(tTris);
                    EG_free(tVtags);
                    EG_free(tCoords);
                    std::stringstream ss;
                    ss << "ocsmGetTessVel Parameter " << mActiveParameterName << " vert " << j+1 << " failed: " 
                       << tStatus << " (Node = " << tVtags[j].pindex << ")!";
                    throwWithError(ss.str());
                }
                aDXDp[3*j  ] = tPcsens[0];
                aDXDp[3*j+1] = tPcsens[1];
                aDXDp[3*j+2] = tPcsens[2];
            }
      
            /* next do all of the edges */
            int tIndex;
            for (int j=1; j<=tNedge; j++)
            {
                auto tStatus = ocsmGetTessVel(mModel, ibody, OCSM_EDGE, j, &tPcsens);
                if (tStatus != EGADS_SUCCESS)
                {
                    EG_free(tTris);
                    EG_free(tVtags);
                    EG_free(tCoords);
                    std::stringstream ss;
                    ss << " ocsmGetTessVel Parameter " << mActiveParameterName << " Edge " << j << " failed: " << tStatus;
                    throwWithError(ss.str());
                }
                for (int k = 0; k < tNvert; k++)
                {
                    if ((tVtags[k].ptype > 0) && (tVtags[k].pindex == j)) {
                        tIndex       = tVtags[k].ptype - 1;
                        aDXDp[3*k  ] = tPcsens[3*tIndex  ];
                        aDXDp[3*k+1] = tPcsens[3*tIndex+1];
                        aDXDp[3*k+2] = tPcsens[3*tIndex+2];
                    }
                }
            }
              
            /* do all of the faces */
            for (int j=1; j<=tNface; j++) {
              auto tStatus = ocsmGetTessVel(mModel, ibody, OCSM_FACE, j, &tPcsens);
              if (tStatus != EGADS_SUCCESS) {
                EG_free(tTris);
                EG_free(tVtags);
                EG_free(tCoords);
                std::stringstream ss;
                ss << " ocsmGetTessVel Parameter " << mActiveParameterName << " Face " << j << "failed: " << tStatus;
                throwWithError(ss.str());
              }
              for (int k=0; k<tNvert; k++)
                if ((tVtags[k].ptype < 0) && (tVtags[k].pindex == j)) {
                  tIndex       = -tVtags[k].ptype - 1;
                  aDXDp[3*k  ] = tPcsens[3*tIndex  ];
                  aDXDp[3*k+1] = tPcsens[3*tIndex+1];
                  aDXDp[3*k+2] = tPcsens[3*tIndex+2];
                }
            }
      
            EG_free(tTris);
            EG_free(tVtags);
            EG_free(tCoords);

        }
        return tSensitivity;
    }
  
    
    void buildGeometryAndGetBodies()
    {

        int tBuildTo = 0; /* all */
        int tNbody   = 0;
        auto tStatus = ocsmBuild(mModel, tBuildTo, &mBuiltTo, &tNbody, NULL);
        EG_deleteObject(mContext); mContext = nullptr;
        if (tStatus != OCSM_SUCCESS)
        {
           throwWithError("ocsmBuild failed");
        }
        tNbody = 0;
        for (int ibody=1; ibody<=mModelT->nbody; ibody++)
        {
            if (mModelT->body[ibody].onstack != 1) continue;
            if (mModelT->body[ibody].botype  == OCSM_NULL_BODY) continue;
            tNbody++;
        }
        if (tNbody <= 0)
        {
            throwWithError("No bodies found.");
        }
        if (tNbody != 1) {
            throwWithError(" ERROR: ETO option only works for a single body!");
        }
    }


    void activateParameterInModel(const std::string& aParameterName)
    {
        int tNumRows(1), tNumCols(1); // no matrix variables allowed currently
        int tType(0); // not sure what this variable is for
        char tParameterName[129];
        for (int j=0; j<mModelT->npmtr; j++)
        {
            auto tStatus = ocsmGetPmtr(mModel, j+1, &tType, &tNumRows, &tNumCols, tParameterName);
            if (tStatus != OCSM_SUCCESS) {
                throwWithError("ocsmGetPmtr failed.");
            }
            if (strcmp(aParameterName.c_str(), tParameterName) == 0) {
                mActiveParameterIndex = j+1;
                mActiveParameterName = aParameterName;
                break;
            }
        }
        if (mActiveParameterIndex == 0)
        {
            std::stringstream ss;
            ss << "Parameter not found: " << aParameterName;
            throwWithError(ss.str());
        }
    }


    void openContext()
    {
        auto tStatus = EG_open(&mContext);
        if (tStatus != EGADS_SUCCESS)
        {
            throwWithError("EGADS failed to Open");
        }
    }

    void openModel()
    {
        auto tStatus = ocsmLoad((char*)mModelFileName.c_str(), &mModel);
        if (tStatus < OCSM_SUCCESS)
        {
            throwWithError("ocsmLoad failed");
        }
        mModelT = (modl_T*)mModel;
        mModelT->context = mContext;
    }

    void checkModel()
    {
        auto tStatus = ocsmCheck(mModel);
        if (tStatus < OCSM_SUCCESS)
        {
            throwWithError("ocsmCheck failed");
        }
    }

    void throwWithError(std::string aError)
    {
        safeFreeOCSM();
        safeFreeContext();
        throw Plato::ParsingException(aError);
    }
    void safeFreeOCSM()
    {
        if (mModel != nullptr)
        {
            ocsmFree(mModel);
            ocsmFree(nullptr);
        }
    }
    void safeFreeContext()
    {
        if (mContext != nullptr)
        {
            EG_close(mContext);
        }
    }
};

} // end namespace Geometry
} // end namespace Plato

#endif
