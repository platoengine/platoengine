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
 * Plato_XTKSharedData.hpp
 *
 *  Created on: Jan 29, 2019
 *      Author: ktdoble
 */

#ifndef BASE_SRC_XTK_PLATO_XTK_XTKSharedData_HPP_
#define BASE_SRC_XTK_PLATO_XTK_XTKSharedData_HPP_

#include "assert.h"
#include "Plato_XTK_Application.hpp"

namespace Plato
{

class XTKSharedData: public SharedData
{
public:
    XTKSharedData(){};

    XTKSharedData(int                   aSize,
                  std::string           aName,
                  Plato::data::layout_t aLayout):
                      mData(aSize),
                      mName(aName),
                      mLayout(aLayout){};

    int size() const {return mData.size();};
    std::string myName() const{return mName;};
    std::string myContext() const { return std::string(); };
    Plato::data::layout_t myLayout() const {return mLayout;};

    void transmitData()
    {
         std::cerr<<"Transmit Data not implemented in XTKSharedData."<<std::endl;
    }
    void setData(const std::vector<double> & aData)
    {
        std::copy(aData.begin(),aData.end(),mData.begin());

    }
    void getData(std::vector<double> & aData) const
    {
        std::copy(mData.begin(),mData.end(),aData.begin());
    }

private:
    Plato::data::layout_t mLayout;
    std::string           mName;
    std::vector<double>   mData;
};


}


#endif /* BASE_SRC_XTK_PLATO_XTK_XTKSharedData_HPP_ */
