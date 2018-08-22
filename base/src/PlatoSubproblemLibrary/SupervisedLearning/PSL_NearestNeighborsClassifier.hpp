#pragma once

#include "PSL_Classifier.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class ParameterData;
class ClassificationArchive;
class DistanceVoter;
class KNearestNeighborsSearcher;

class NearestNeighborsClassifier : public Classifier
{
public:
    NearestNeighborsClassifier(AbstractAuthority* authority);
    virtual ~NearestNeighborsClassifier();

    virtual void unpack_data();

    virtual void initialize(ParameterData* parameters, ClassificationArchive* archive);
    virtual void train(ParameterData* parameters, ClassificationArchive* archive);
    virtual int classify(const std::vector<double>& input_scalars, const std::vector<int>& input_enums);

protected:

    void allocate_classifier_data(ParameterData* parameters, ClassificationArchive* archive);
    void check_parameters(ParameterData* parameters);

    void clear_derived_data();
    void build_derived_data();

    void clear_knnSearcher();
    void ensureBuilt_knnSearcher();

    // DataSequence data
    int* m_output_enum_size;
    int* m_num_neighbors;
    int* m_processed_length;
    int* m_distance_voter_enum;
    std::vector<int>* m_input_enum_sizes;

    // derived data
    DistanceVoter* m_distance_voter;

    // searcher
    KNearestNeighborsSearcher* m_knn_searcher;

};

}
