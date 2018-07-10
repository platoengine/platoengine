#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{

void get_mean_and_std(const std::vector<double>& data, double& mean, double& std);
void get_mean_and_std_of_lowest_fraction(const double& fraction, const std::vector<double>& data, double& mean, double& std);

void get_median(std::vector<double>& data_to_sort, double& median);
void get_median_indexes(const int& length, int& index0, int& index1);

void get_nonparametric_skew(const std::vector<double>& data, double& skew, double& mean, double& std);
void get_nonparametric_skew(const std::vector<double>& data, double& skew);

void solve_nonparametric_skew(const std::vector<double>& data, double& alpha, double& median);

void get_skew_normalize(const std::vector<double>& input, double& alpha, double& median, double& std);
void skew_normalize(std::vector<double>& data, const double& alpha, const double& median, const double& std);
void skew_normalize(double& data, const double& alpha, const double& median, const double& std);

void standardize(std::vector<double>& data, const double& mean, const double& std);
void standardize(double& data, const double& mean, const double& std);

void get_bipolar_normalize(const std::vector<double>& input, double& min, double& max);
void bipolar_normalize(std::vector<double>& data, const double& min, const double& max);
void bipolar_normalize(double& data, const double& min, const double& max);

}
