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
/* undocumented EGADS function */
extern int EG_loadTess(ego body, const char *name, ego *tess);
}

#include "Plato_Exceptions.hpp"


namespace Plato {
namespace Geometry {

template <typename ScalarType, typename ScalarVectorType>
class ESP
{
    std::vector<std::string>  mParameterNames;
   
    std::string  mModelFileName;
    std::string  mTessFileName;
    std::string  mActiveParameterName;
    int          mActiveParameterIndex, mBuiltTo;
    ego          mContext;
    void         *mModel;
    modl_T       *mModelT;

    static constexpr int mSpaceDim = 3;

  public:

    ESP(std::string aModelFileName, std::string aTessFileName) :
      mModelFileName(aModelFileName),
      mTessFileName(aTessFileName),
      mActiveParameterIndex(-1),
      mBuiltTo(0),
      mContext(nullptr),
      mModel(nullptr),
      mModelT(nullptr)
    {
        readNames();
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

    ScalarType sensitivity(int aParameterIndex, ScalarVectorType aGradientX)
    {
        activateParameterInModel(mParameterNames[aParameterIndex]);

        return computeSensitivity(aGradientX);
    }

  private:
    void tesselate()
    {
        // JR TODO: get these as inputs
        /* set the global tessellation parameters */
        std::array<ScalarType, mSpaceDim> tGlobal;
        tGlobal[0] = 0.025;
        tGlobal[1] = 0.001;
        tGlobal[2] = 12.0;

        /* tessellate & store the tessellation object in OpenCSM */
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
            else
            {
                std::array<ScalarType, mSpaceDim> tParam = tGlobal;
                constexpr int nBoxVals = 2*mSpaceDim;
                std::array<ScalarType, nBoxVals> tBox;
                // tparam[0] = tGlobal[0];
                // tparam[1] = tGlobal[1];
                // tparam[2] = tGlobal[2];
                auto tStatus = EG_getBoundingBox(tBody, tBox.data());
                if (tStatus != EGADS_SUCCESS) {
                    throwWithError("EG_getBoundingBox failed.");
                }
                auto size = sqrt((tBox[3]-tBox[0])*(tBox[3]-tBox[0]) +
                                 (tBox[4]-tBox[1])*(tBox[4]-tBox[1]) +
                                 (tBox[5]-tBox[2])*(tBox[5]-tBox[2]));
                tParam[0] *= size;
                tParam[1] *= size;
                tStatus = EG_makeTessBody(tBody, tParam.data(), &mModelT->body[ibody].etess);
                if (tStatus != EGADS_SUCCESS) {
                    throwWithError("EG_makeTessBody failed.");
                }
            }
        }
    }


    ScalarType computeSensitivity(ScalarVectorType aGradientX)
    {
        ScalarType tSensitivity(0.0);

        /* clear all then set the parameter & tell OpenCSM */
        ocsmSetVelD(mModel, 0,     0,    0,    0.0);
        ocsmSetVelD(mModel, mActiveParameterIndex, /*irow=*/ 1, /*icol=*/ 1, 1.0);
        int tBuildTo = 0;
        int tNbody   = 0;
        auto tStatus = ocsmBuild(mModel, tBuildTo, &mBuiltTo, &tNbody, NULL);
        
        /* retrieve the sensitivities for the active bodies */
        int tNface, tNedge, tNvert, tNtri;
        int *tTris(nullptr);
        ScalarType *tCoords(nullptr), *tDvar(nullptr);
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
            tDvar = (ScalarType *) EG_alloc(3*tNvert*sizeof(ScalarType));
            if (tDvar == NULL)
            {
                EG_free(tTris);
                EG_free(tVtags);
                throwWithError("Malloc failed on verts.");
            }

            const ScalarType *tPcsens;
      
            /* fill in the Nodes */
            for (int j=0; j<tNvert; j++)
            {
                if (tVtags[j].ptype != 0) continue;
                auto tStatus = ocsmGetTessVel(mModel, ibody, OCSM_NODE, tVtags[j].pindex, &tPcsens);
                if (tStatus != EGADS_SUCCESS)
                {
                    EG_free(tDvar);
                    EG_free(tTris);
                    EG_free(tVtags);
                    EG_free(tCoords);
                    std::stringstream ss;
                    ss << "ocsmGetTessVel Parameter " << mActiveParameterName << " vert " << j+1 << " failed: " 
                       << tStatus << " (Node = " << tVtags[j].pindex << ")!";
                    throwWithError(ss.str());
                }
                tDvar[3*j  ] = tPcsens[0];
                tDvar[3*j+1] = tPcsens[1];
                tDvar[3*j+2] = tPcsens[2];
            }
      
            /* next do all of the edges */
            int tIndex;
            for (int j=1; j<=tNedge; j++)
            {
                auto tStatus = ocsmGetTessVel(mModel, ibody, OCSM_EDGE, j, &tPcsens);
                if (tStatus != EGADS_SUCCESS)
                {
                    EG_free(tDvar);
                    EG_free(tTris);
                    EG_free(tVtags);
                    EG_free(tCoords);
                    std::stringstream ss;
                    ss << " ocsmGetTessVel Parameter " << mActiveParameterName << " Edge " << j << " failed: " << stat;
                    throwWithError(ss.str());
                }
                for (int k = 0; k < tNvert; k++)
                {
                    if ((tVtags[k].ptype > 0) && (tVtags[k].pindex == j)) {
                        tIndex       = tVtags[k].ptype - 1;
                        tDvar[3*k  ] = tPcsens[3*tIndex  ];
                        tDvar[3*k+1] = tPcsens[3*tIndex+1];
                        tDvar[3*k+2] = tPcsens[3*tIndex+2];
                    }
                }
            }
              
            /* do all of the faces */
            for (int j=1; j<=tNface; j++) {
              auto tStatus = ocsmGetTessVel(mModel, ibody, OCSM_FACE, j, &tPcsens);
              if (tStatus != EGADS_SUCCESS) {
                EG_free(tDvar);
                EG_free(tTris);
                EG_free(tVtags);
                EG_free(tCoords);
                std::stringstream ss;
                ss << " ocsmGetTessVel Parameter " << mActiveParameterName << " Face " << j << "failed: " << stat;
                throwWithError(ss.str());
              }
              for (int k=0; k<tNvert; k++)
                if ((tVtags[k].ptype < 0) && (tVtags[k].pindex == j)) {
                  tIndex       = -tVtags[k].ptype - 1;
                  tDvar[3*k  ] = tPcsens[3*tIndex  ];
                  tDvar[3*k+1] = tPcsens[3*tIndex+1];
                  tDvar[3*k+2] = tPcsens[3*tIndex+2];
                }
            }
            for (int k=0; k<mSpaceDim*tNvert; k++)
            {
                tSensitivity += tDvar[k]*aGradientX(k);
            }
      
            EG_free(tDvar);
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
        if (tStatus != SUCCESS)
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
        int tType(0); // not sure what this does
        char tParameterName[129];
        for (int j=0; j<mModelT->npmtr; j++)
        {
            auto tStatus = ocsmGetPmtr(mModel, j+1, &tType, &tNumRows, &tNumCols, tParameterName);
            if (tStatus != SUCCESS) {
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
        if (tStatus < SUCCESS)
        {
            throwWithError("ocsmLoad failed");
        }
        mModelT = (modl_T*)mModel;
        mModelT->context = mContext;
    }

    void checkModel()
    {
        auto tStatus = ocsmCheck(mModel);
        if (tStatus < SUCCESS)
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
