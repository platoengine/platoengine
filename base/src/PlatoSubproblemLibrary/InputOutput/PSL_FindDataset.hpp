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

#pragma once

#include <vector>
#include <string>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;

namespace datasets_t {
enum datasets_t {
    unset_dataset,
    simple_dataset,
    iris_dataset,
    german_credit_dataset,
    pop_failures_dataset,
    online_news_dataset,
    nicotine_dataset,
    lsvt_dataset,
    htru2_dataset,
    frogs_MFCCs_dataset,
    dota2_train_dataset,
    dota2_test_dataset,
    default_credit_dataset,
    biodeg_dataset,
    wilt_dataset,
    waveform_dataset,
    wine_quality_dataset,
    legacy_mltr_dataset,
    cervical_cancer_dataset,
    connect_4_dataset,
    pen_digits_dataset,
    seizure_dataset,
    sensorless_drive_dataset,
};
}

class FindDataset
{
public:
    FindDataset(AbstractAuthority* authority);
    ~FindDataset();

    std::string find(datasets_t::datasets_t dataset);

    void set_prefer_binary(const bool& prefer_binary) { m_prefer_binary = prefer_binary; }

protected:

    std::string write_simple();
    std::string write_iris();
    std::string write_germanCredit();

    std::string search(const std::string& filename);

    void build_parameter_data(ParameterData& parameter_data);

    AbstractAuthority* m_authority;
    bool m_prefer_binary;
    std::vector<std::string> m_files_to_delete;

};

}
