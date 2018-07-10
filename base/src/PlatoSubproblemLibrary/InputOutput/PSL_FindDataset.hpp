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
