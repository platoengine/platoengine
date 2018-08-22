#include "PSL_FindDataset.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <string>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

FindDataset::FindDataset(AbstractAuthority* authority) :
        m_authority(authority),
        m_prefer_binary(true),
        m_files_to_delete()
{
}

FindDataset::~FindDataset()
{
    // delete files
    const size_t num_files_to_delete = m_files_to_delete.size();
    for(size_t f = 0u; f < num_files_to_delete; f++)
    {
        InputOutputManager io_manager(m_authority);
        io_manager.set_filename(m_files_to_delete[f]);
        io_manager.delete_file();
    }
    m_files_to_delete.clear();

    m_authority = NULL;
}

std::string FindDataset::find(datasets_t::datasets_t dataset)
{
    // attempt to match
    switch(dataset)
    {
        case datasets_t::datasets_t::simple_dataset:
        {
            const std::string result = write_simple();
            m_files_to_delete.push_back(result);
            return result;
            break;
        }
        case datasets_t::datasets_t::iris_dataset:
        {
            const std::string result = write_iris();
            m_files_to_delete.push_back(result);
            return result;
            break;
        }
        case datasets_t::datasets_t::pop_failures_dataset:
        {
            return search("pop_failures.psl");
            break;
        }
        case datasets_t::datasets_t::online_news_dataset:
        {
            return search("online_news.psl");
            break;
        }
        case datasets_t::datasets_t::nicotine_dataset:
        {
            return search("nicotine.psl");
            break;
        }
        case datasets_t::datasets_t::lsvt_dataset:
        {
            return search("lsvt.psl");
            break;
        }
        case datasets_t::datasets_t::htru2_dataset:
        {
            return search("htru2.psl");
            break;
        }
        case datasets_t::datasets_t::frogs_MFCCs_dataset:
        {
            return search("frogs_MFCCs.psl");
            break;
        }
        case datasets_t::datasets_t::dota2_train_dataset:
        {
            return search("dota2_train.psl");
            break;
        }
        case datasets_t::datasets_t::dota2_test_dataset:
        {
            return search("dota2_test.psl");
            break;
        }
        case datasets_t::datasets_t::default_credit_dataset:
        {
            return search("default_credit.psl");
            break;
        }
        case datasets_t::datasets_t::biodeg_dataset:
        {
            return search("biodeg.psl");
            break;
        }
        case datasets_t::datasets_t::wilt_dataset:
        {
            return search("wilt.psl");
            break;
        }
        case datasets_t::datasets_t::waveform_dataset:
        {
            return search("waveform.psl");
            break;
        }
        case datasets_t::datasets_t::wine_quality_dataset:
        {
            return search("wine_quality.psl");
            break;
        }
        case datasets_t::datasets_t::legacy_mltr_dataset:
        {
            return search("legacy_mltr.psl");
            break;
        }
        case datasets_t::datasets_t::cervical_cancer_dataset:
        {
            return search("cervicalCancer.psl");
            break;
        }
        case datasets_t::datasets_t::connect_4_dataset:
        {
            return search("connect4.psl");
            break;
        }
        case datasets_t::datasets_t::pen_digits_dataset:
        {
            return search("pendigits.psl");
            break;
        }
        case datasets_t::datasets_t::seizure_dataset:
        {
            return search("seizure.psl");
            break;
        }
        case datasets_t::datasets_t::sensorless_drive_dataset:
        {
            return search("sensorlessDrive.psl");
            break;
        }
        case datasets_t::datasets_t::unset_dataset:
        default:
        {
            break;
        }
    }

    // could not match
    const std::string message = std::string(__func__) + std::string(": could not match dataset (") + std::to_string(dataset)
                                + std::string("). fatal error.\n");
    m_authority->utilities->fatal_error(message);
    return std::string();
}

std::string FindDataset::write_simple()
{
    const std::string filename = "_simple_dataset_archive.psl";

    // initialize archive
    ClassificationArchive archive(m_authority);
    ParameterData parameter_data;
    build_parameter_data(parameter_data);
    parameter_data.set_archive_filename(filename);
    archive.initialize(&parameter_data);

    // enum sizes
    const std::vector<int> input_enum = {};
    const int output_enum_size = 2;
    archive.set_enum_size(input_enum, output_enum_size);

    // add rows
    archive.add_row( {-.3, -.1}, input_enum, 0);
    archive.add_row( {-.2, 0.}, input_enum, 0);
    archive.add_row( {-.1, .3}, input_enum, 0);
    archive.add_row( {.1, .5}, input_enum, 1);
    archive.add_row( {.15, -.1}, input_enum, 1);
    archive.add_row( {.2, .2}, input_enum, 1);
    archive.add_row( {.25, -.5}, input_enum, 1);
    archive.finalize_archive();

    return filename;
}

std::string FindDataset::search(const std::string& filename)
{
    const std::vector<std::string> valid_prefix = {"", "../"};

    // for each prefix
    const size_t num_valid_prefix = valid_prefix.size();
    for(size_t p = 0u; p < num_valid_prefix; p++)
    {

        // concatenate prefix
        const std::string full_filename = valid_prefix[p] + filename;

        // determine if file exists
        InputOutputManager io_manager(m_authority);
        io_manager.set_filename(full_filename);
        const bool is_empty = io_manager.is_empty();

        // if exists, return this file
        if(!is_empty)
        {
            return full_filename;
        }
    }

    return "";
}

void FindDataset::build_parameter_data(ParameterData& parameter_data)
{
    parameter_data.set_archive_filetype_binary_preference(m_prefer_binary);
}

}
