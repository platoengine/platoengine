// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Implementation_MeshModular.hpp"
#include "PSL_Interface_MeshModular.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_NodeSet.hpp"
#include "PSL_Selector.hpp"
#include "PSL_NodeSetMatcher.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Interface_PointCloud.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Point.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_Random.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <cstddef>
#include <algorithm>
#include <mpi.h>

namespace PlatoSubproblemLibrary
{

namespace NodeSetMatcherTest
{

void specifyTwoNodesProblem(example::Interface_PointCloud& point_cloud,
                            std::vector<int>& local_to_global,
                            Point& pvec,
                            NodeSet& ns_source,
                            NodeSet& ns_destination)
{
    // specify points
    point_cloud.set_num_points(2u);
    std::vector<double> p0_data = {-0.1, 0.2, -0.4};
    point_cloud.set_point_data(0u, p0_data);
    std::vector<double> p1_data = {0.2, -0.1, 0.6};
    point_cloud.set_point_data(1u, p1_data);
    local_to_global = {0, 1};

    // specify matching
    std::vector<double> vec = {p1_data[0] - p0_data[0], p1_data[1] - p0_data[1], p1_data[2] - p0_data[2]};
    pvec.set(0u, vec);
    ns_source.set_index(5u);
    ns_source.insert(0u);
    ns_destination.set_index(3u);
    ns_destination.insert(1u);
}

PSL_TEST(NodeSetMatcher,uninvolvedProcessors)
{
    set_rand_seed();
    AbstractAuthority authority;

    // specify problem
    example::Interface_PointCloud point_cloud;
    std::vector<int> local_to_global;
    Point pvec;
    NodeSet ns_source;
    NodeSet ns_destination;

    // specify points
    const size_t mpi_rank = authority.mpi_wrapper->get_rank();
    point_cloud.set_num_points(2u);
    std::vector<double> p0_data = {-0.1 + mpi_rank, 0.2 + mpi_rank, -0.4 + mpi_rank};
    point_cloud.set_point_data(0u, p0_data);
    std::vector<double> p1_data = {0.2 + mpi_rank, -0.1 + mpi_rank, 0.6 + mpi_rank};
    point_cloud.set_point_data(1u, p1_data);
    local_to_global = { 0 + int(mpi_rank) * 2, 1 + int(mpi_rank) * 2};

    // specify matching
    std::vector<double> vec = {p1_data[0] - p0_data[0], p1_data[1] - p0_data[1], p1_data[2] - p0_data[2]};
    pvec.set(0u, vec);
    ns_source.set_index(5u);
    ns_destination.set_index(3u);
    if(mpi_rank == 0u)
    {
        ns_source.insert(0u);
        ns_destination.insert(1u);
    }

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.set_spatial_tolerance(0.1);

    matcher.register_pair(&ns_source, &ns_destination, &pvec);
    matcher.match(&point_cloud, local_to_global);

    if(mpi_rank == 0u)
    {
        std::vector<std::pair<int, int> > global_source_and_destination;
        matcher.get_final_pairs(ns_source.get_index(), ns_destination.get_index(), global_source_and_destination);
        ASSERT_EQ(global_source_and_destination.size(), 1u);
        EXPECT_EQ(global_source_and_destination[0].first, 0);
        EXPECT_EQ(global_source_and_destination[0].second, 1);
    }
}

PSL_TEST(NodeSetMatcher,twoNodesAccurate)
{
    set_rand_seed();
    AbstractAuthority authority;

    // only run in serial
    const size_t mpi_size = authority.mpi_wrapper->get_size();
    if(mpi_size > 1u)
    {
        return;
    }

    // specify problem
    example::Interface_PointCloud point_cloud;
    std::vector<int> local_to_global;
    Point pvec;
    NodeSet ns_source;
    NodeSet ns_destination;
    specifyTwoNodesProblem(point_cloud, local_to_global, pvec, ns_source, ns_destination);

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.set_spatial_tolerance(0.1);

    matcher.register_pair(&ns_source, &ns_destination, &pvec);
    matcher.match(&point_cloud, local_to_global);
    std::vector<std::pair<int, int> > global_source_and_destination;
    matcher.get_final_pairs(ns_source.get_index(), ns_destination.get_index(), global_source_and_destination);

    ASSERT_EQ(global_source_and_destination.size(), 1u);
    EXPECT_EQ(global_source_and_destination[0].first, 0);
    EXPECT_EQ(global_source_and_destination[0].second, 1);
}

PSL_TEST(NodeSetMatcher,twoNodesBadFinalPairs)
{
    set_rand_seed();
    AbstractAuthority authority;

    // only run in serial
    const size_t mpi_size = authority.mpi_wrapper->get_size();
    if(mpi_size > 1u)
    {
        return;
    }

    // specify problem
    example::Interface_PointCloud point_cloud;
    std::vector<int> local_to_global;
    Point pvec;
    NodeSet ns_source;
    NodeSet ns_destination;
    specifyTwoNodesProblem(point_cloud, local_to_global, pvec, ns_source, ns_destination);

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.set_spatial_tolerance(0.1);

    matcher.register_pair(&ns_source, &ns_destination, &pvec);
    matcher.match(&point_cloud, local_to_global);

    std::vector<std::pair<int, int> > global_source_and_destination;
    matcher.get_final_pairs(ns_source.get_index(), ns_destination.get_index(), global_source_and_destination);
    ASSERT_EQ(global_source_and_destination.size(), 1u);
    EXPECT_EQ(global_source_and_destination[0].first, 0);
    EXPECT_EQ(global_source_and_destination[0].second, 1);

    example::Interface_BasicGlobalUtilities* utilities =
            dynamic_cast<example::Interface_BasicGlobalUtilities*>(authority.utilities);
    assert(utilities);
    utilities->set_never_abort(true);

    // expect abort
    matcher.get_final_pairs(ns_source.get_index() + 1u, ns_destination.get_index(), global_source_and_destination);
    EXPECT_EQ(utilities->get_ignored_abort(), true);
    utilities->reset_ignored_abort();
    EXPECT_EQ(utilities->get_ignored_abort(), false);

    // expect abort
    matcher.get_final_pairs(ns_source.get_index(), ns_destination.get_index() + 1u, global_source_and_destination);
    EXPECT_EQ(utilities->get_ignored_abort(), true);
    utilities->reset_ignored_abort();
    EXPECT_EQ(utilities->get_ignored_abort(), false);

    // expect abort
    matcher.get_final_pairs(ns_source.get_index() + 1u, ns_destination.get_index() + 1u, global_source_and_destination);
    EXPECT_EQ(utilities->get_ignored_abort(), true);
    utilities->reset_ignored_abort();
    EXPECT_EQ(utilities->get_ignored_abort(), false);

    utilities->set_never_abort(false);
}

PSL_TEST(NodeSetMatcher,twoNodesUnderSourced)
{
    set_rand_seed();
    AbstractAuthority authority;

    // only run in serial
    const size_t mpi_size = authority.mpi_wrapper->get_size();
    if(mpi_size > 1u)
    {
        return;
    }

    // specify problem
    example::Interface_PointCloud point_cloud;
    std::vector<int> local_to_global;
    Point pvec;
    NodeSet ns_source;
    NodeSet ns_destination;
    specifyTwoNodesProblem(point_cloud, local_to_global, pvec, ns_source, ns_destination);
    ns_source.m_nodes.clear();

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.set_spatial_tolerance(0.1);

    matcher.register_pair(&ns_source, &ns_destination, &pvec);

    // get utilities
    example::Interface_BasicGlobalUtilities* utilities =
            dynamic_cast<example::Interface_BasicGlobalUtilities*>(authority.utilities);
    assert(utilities);
    utilities->set_never_abort(true);

    // expect abort
    matcher.match(&point_cloud, local_to_global);
    EXPECT_EQ(utilities->get_ignored_abort(), true);
    utilities->reset_ignored_abort();

    utilities->set_never_abort(false);
}

PSL_TEST(NodeSetMatcher,twoNodesUnderDestination)
{
    set_rand_seed();
    AbstractAuthority authority;

    // only run in serial
    const size_t mpi_size = authority.mpi_wrapper->get_size();
    if(mpi_size > 1u)
    {
        return;
    }

    // specify problem
    example::Interface_PointCloud point_cloud;
    std::vector<int> local_to_global;
    Point pvec;
    NodeSet ns_source;
    NodeSet ns_destination;
    specifyTwoNodesProblem(point_cloud, local_to_global, pvec, ns_source, ns_destination);
    ns_destination.m_nodes.clear();

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.set_spatial_tolerance(0.1);

    matcher.register_pair(&ns_source, &ns_destination, &pvec);

    // get utilities
    example::Interface_BasicGlobalUtilities* utilities =
            dynamic_cast<example::Interface_BasicGlobalUtilities*>(authority.utilities);
    assert(utilities);
    utilities->set_never_abort(true);

    // expect abort
    matcher.match(&point_cloud, local_to_global);
    EXPECT_EQ(utilities->get_ignored_abort(), true);
    utilities->reset_ignored_abort();

    utilities->set_never_abort(false);
}

PSL_TEST(NodeSetMatcher,twoNodesVectorMisaligned)
{
    set_rand_seed();
    AbstractAuthority authority;

    // only run in serial
    const size_t mpi_size = authority.mpi_wrapper->get_size();
    if(mpi_size > 1u)
    {
        return;
    }

    // specify problem
    example::Interface_PointCloud point_cloud;
    std::vector<int> local_to_global;
    Point pvec;
    NodeSet ns_source;
    NodeSet ns_destination;
    specifyTwoNodesProblem(point_cloud, local_to_global, pvec, ns_source, ns_destination);
    pvec = pvec * (-1.);

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.set_spatial_tolerance(0.1);

    matcher.register_pair(&ns_source, &ns_destination, &pvec);

    // get utilities
    example::Interface_BasicGlobalUtilities* utilities =
            dynamic_cast<example::Interface_BasicGlobalUtilities*>(authority.utilities);
    assert(utilities);
    utilities->set_never_abort(true);

    // expect abort
    matcher.match(&point_cloud, local_to_global);
    EXPECT_EQ(utilities->get_ignored_abort(), true);
    utilities->reset_ignored_abort();

    utilities->set_never_abort(false);
}

void build_parallel_problem(AbstractAuthority* authority,
                            example::Interface_MeshModular& modular_interface,
                            example::ElementBlock& modular_block,
                            std::vector<int>& local_to_global,
                            NodeSet& ns_source,
                            NodeSet& ns_destination,
                            Point& source_to_destination_vector)
{
    const double xdist = 3.0;
    const double ydist = 8.0;
    const double zdist = 15.0;
    const size_t approx_points_per_processor = 8u;

    // build for modular
    modular_block.build_random_subset_of_structured_grid(approx_points_per_processor,
                                                         xdist, ydist, zdist,
                                                         authority);
    std::vector<size_t> local_to_global_sizet;
    modular_block.get_global_ids(local_to_global_sizet);
    local_to_global.assign(local_to_global_sizet.begin(), local_to_global_sizet.end());
    modular_interface.set_mesh(&modular_block);

    // define nodeset pair
    Selector selector;
    selector.set_selection_criteria_y(0.);
    selector.select(&modular_interface, ns_source);
    ns_source.set_index(0u);
    selector.set_selection_criteria_y(ydist);
    selector.select(&modular_interface, ns_destination);
    ns_destination.set_index(1u);
    std::vector<double> s2dv = {0, ydist, 0.};
    source_to_destination_vector = Point(0u, s2dv);
}

PSL_TEST(NodeSetMatcher,simpleAccuratePerturbParallel)
{
    set_rand_seed();
    AbstractAuthority authority;


    example::Interface_MeshModular modular_interface;
    example::ElementBlock modular_block;
    std::vector<int> local_to_global;
    NodeSet ns_source;
    NodeSet ns_destination;
    Point source_to_destination_vector;

    build_parallel_problem(&authority,
                           modular_interface,
                           modular_block,
                           local_to_global,
                           ns_source,
                           ns_destination,
                           source_to_destination_vector);

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.infer_spatial_tolerance(&modular_interface);
    const double spatial_tolerance = matcher.get_spatial_tolerance();

    // perturb
    modular_block.random_perturb_local_nodal_locations(spatial_tolerance / 10.0);

    // register, match, and solve
    matcher.register_pair(&ns_source, &ns_destination, &source_to_destination_vector);
    matcher.match(&modular_interface, local_to_global);
    std::vector<std::pair<int, int> > global_source_and_destination_pairs;
    matcher.get_final_pairs(ns_source.get_index(), ns_destination.get_index(), global_source_and_destination_pairs);

    // matches that were found, are correct
    const size_t num_final_pairs = global_source_and_destination_pairs.size();
    for(size_t final_pair = 0u; final_pair < num_final_pairs; final_pair++)
    {
        int global_source_index = global_source_and_destination_pairs[final_pair].first;
        Point* global_source = modular_block.get_global_point(global_source_index);
        int global_destination_index = global_source_and_destination_pairs[final_pair].second;
        Point* global_destination = modular_block.get_global_point(global_destination_index);

        Point estimated_destination = (*global_source) + source_to_destination_vector;
        const double distance_error = estimated_destination.distance(global_destination);
        EXPECT_GE(spatial_tolerance, distance_error);
    }
}

PSL_TEST(NodeSetMatcher,underSourcedParallel)
{
    set_rand_seed();
    AbstractAuthority authority;

    const size_t mpi_rank = authority.mpi_wrapper->get_rank();
    const size_t rank_to_remove = 0u;
    bool did_remove_on_rank = false;

    example::Interface_MeshModular modular_interface;
    example::ElementBlock modular_block;
    std::vector<int> local_to_global;
    NodeSet ns_source;
    NodeSet ns_destination;
    Point source_to_destination_vector;

    build_parallel_problem(&authority,
                           modular_interface,
                           modular_block,
                           local_to_global,
                           ns_source,
                           ns_destination,
                           source_to_destination_vector);
    std::map<int, int> global_to_local;
    invert_vector_to_map(local_to_global, global_to_local);

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.infer_spatial_tolerance(&modular_interface);

    // remove a node
    if(mpi_rank == rank_to_remove)
    {
        int local_source_to_erase = -1;
        int global_source_to_erase = -1;
        if(ns_source.size() > 0u)
        {
            local_source_to_erase = *ns_source.m_nodes.begin();
            did_remove_on_rank = true;
            global_source_to_erase = local_to_global[local_source_to_erase];
        }
        authority.mpi_wrapper->broadcast(rank_to_remove, global_source_to_erase);
        if(local_source_to_erase >= 0)
        {
            ns_source.m_nodes.erase(local_source_to_erase);
        }
    }
    else
    {
        int local_source_to_erase = -1;
        int global_source_to_erase = -1;
        authority.mpi_wrapper->broadcast(rank_to_remove, global_source_to_erase);
        std::map<int, int>::iterator g2l_it = global_to_local.find(global_source_to_erase);
        if(g2l_it != global_to_local.end())
        {
            local_source_to_erase = g2l_it->second;
        }
        if(local_source_to_erase >= 0)
        {
            ns_source.m_nodes.erase(local_source_to_erase);
        }
    }

    matcher.register_pair(&ns_source, &ns_destination, &source_to_destination_vector);

    // get utilities
    example::Interface_BasicGlobalUtilities* utilities =
            dynamic_cast<example::Interface_BasicGlobalUtilities*>(authority.utilities);
    assert(utilities);
    utilities->set_never_abort(true);

    // expect abort
    matcher.match(&modular_interface, local_to_global);
    int locally_found_an_abort = utilities->get_ignored_abort();
    int globally_found_an_abort = -1;
    authority.mpi_wrapper->all_reduce_max(locally_found_an_abort, globally_found_an_abort);
    if(mpi_rank == rank_to_remove)
    {
        EXPECT_EQ(globally_found_an_abort, int(did_remove_on_rank));
    }
    utilities->reset_ignored_abort();

    utilities->set_never_abort(false);
}

PSL_TEST(NodeSetMatcher,underDestinationParallel)
{
    set_rand_seed();
    AbstractAuthority authority;

    const size_t mpi_rank = authority.mpi_wrapper->get_rank();
    const size_t mpi_size = authority.mpi_wrapper->get_size();
    const size_t rank_to_remove = mpi_size - 1;
    bool did_remove_on_rank = false;

    example::Interface_MeshModular modular_interface;
    example::ElementBlock modular_block;
    std::vector<int> local_to_global;
    NodeSet ns_source;
    NodeSet ns_destination;
    Point source_to_destination_vector;

    build_parallel_problem(&authority,
                           modular_interface,
                           modular_block,
                           local_to_global,
                           ns_source,
                           ns_destination,
                           source_to_destination_vector);
    std::map<int, int> global_to_local;
    invert_vector_to_map(local_to_global, global_to_local);

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.infer_spatial_tolerance(&modular_interface);

    // remove a node
    if(mpi_rank == rank_to_remove)
    {
        int local_destination_to_erase = -1;
        int global_destination_to_erase = -1;
        if(ns_destination.size() > 0u)
        {
            local_destination_to_erase = *ns_destination.m_nodes.begin();
            did_remove_on_rank = true;
            global_destination_to_erase = local_to_global[local_destination_to_erase];
        }
        authority.mpi_wrapper->broadcast(rank_to_remove, global_destination_to_erase);
        if(local_destination_to_erase >= 0)
        {
            ns_destination.m_nodes.erase(local_destination_to_erase);
        }
    }
    else
    {
        int local_destination_to_erase = -1;
        int global_destination_to_erase = -1;
        authority.mpi_wrapper->broadcast(rank_to_remove, global_destination_to_erase);
        std::map<int, int>::iterator g2l_it = global_to_local.find(global_destination_to_erase);
        if(g2l_it != global_to_local.end())
        {
            local_destination_to_erase = g2l_it->second;
        }
        if(local_destination_to_erase >= 0)
        {
            ns_destination.m_nodes.erase(local_destination_to_erase);
        }
    }

    matcher.register_pair(&ns_source, &ns_destination, &source_to_destination_vector);

    // get utilities
    example::Interface_BasicGlobalUtilities* utilities =
            dynamic_cast<example::Interface_BasicGlobalUtilities*>(authority.utilities);
    assert(utilities);
    utilities->set_never_abort(true);

    // expect abort
    matcher.match(&modular_interface, local_to_global);
    int locally_found_an_abort = utilities->get_ignored_abort();
    int globally_found_an_abort = -1;
    authority.mpi_wrapper->all_reduce_max(locally_found_an_abort, globally_found_an_abort);
    if(mpi_rank == rank_to_remove)
    {
        EXPECT_EQ(globally_found_an_abort, int(did_remove_on_rank));
    }
    utilities->reset_ignored_abort();

    utilities->set_never_abort(false);
}

PSL_TEST(NodeSetMatcher,vectorMisalginedParallel)
{
    set_rand_seed();
    AbstractAuthority authority;

    example::Interface_MeshModular modular_interface;
    example::ElementBlock modular_block;
    std::vector<int> local_to_global;
    NodeSet ns_source;
    NodeSet ns_destination;
    Point source_to_destination_vector;

    build_parallel_problem(&authority,
                           modular_interface,
                           modular_block,
                           local_to_global,
                           ns_source,
                           ns_destination,
                           source_to_destination_vector);
    source_to_destination_vector = source_to_destination_vector * (1.5);

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.infer_spatial_tolerance(&modular_interface);

    matcher.register_pair(&ns_source, &ns_destination, &source_to_destination_vector);

    // get utilities
    example::Interface_BasicGlobalUtilities* utilities =
            dynamic_cast<example::Interface_BasicGlobalUtilities*>(authority.utilities);
    assert(utilities);
    utilities->set_never_abort(true);

    // expect abort
    utilities->set_never_abort(true);
    matcher.match(&modular_interface, local_to_global);
    if(ns_source.size() > 0u || ns_destination.size() > 0u)
    {
        EXPECT_EQ(utilities->get_ignored_abort(), true);
    }
    utilities->reset_ignored_abort();

    utilities->set_never_abort(false);
}

PSL_TEST(NodeSetMatcher,multipleNodesetsParallel)
{
    set_rand_seed();
    AbstractAuthority authority;

    // build
    const double xdist = 3.0;
    const double ydist = 8.0;
    const double zdist = 15.0;
    const size_t approx_points_per_processor = 8u; // small
//    const size_t approx_points_per_processor = 5000u; // medium
//    const size_t approx_points_per_processor = 50000u; // large

    // build for modular
    example::ElementBlock modular_block;
    modular_block.build_random_subset_of_structured_grid(approx_points_per_processor,
                                                         xdist, ydist, zdist,
                                                         &authority);
    std::vector<size_t> local_to_global_sizet;
    modular_block.get_global_ids(local_to_global_sizet);
    std::vector<int> local_to_global(local_to_global_sizet.begin(), local_to_global_sizet.end());
    example::Interface_MeshModular modular_interface;
    modular_interface.set_mesh(&modular_block);

    Selector selector;
    const size_t num_nodeset_pairs = 3u;
    std::vector<NodeSet> ns_source(num_nodeset_pairs);
    std::vector<NodeSet> ns_destination(num_nodeset_pairs);
    std::vector<Point> source_to_destination_vector(num_nodeset_pairs);

    // define nodeset pair 0
    selector.set_selection_criteria_x(0.);
    selector.select(&modular_interface, ns_source[0]);
    ns_source[0].set_index(0u);
    selector.set_selection_criteria_x(xdist);
    selector.select(&modular_interface, ns_destination[0]);
    ns_destination[0].set_index(1u);
    std::vector<double> s2dv0 = {xdist, 0., 0.};
    source_to_destination_vector[0] = Point(0u, s2dv0);

    // define nodeset pair 1
    selector.set_selection_criteria_z(zdist);
    selector.select(&modular_interface, ns_source[1]);
    ns_source[1].set_index(2u);
    selector.set_selection_criteria_z(0);
    selector.select(&modular_interface, ns_destination[1]);
    ns_destination[1].set_index(4u);
    std::vector<double> s2dv1 = {0., 0., -zdist};
    source_to_destination_vector[1] = Point(0u, s2dv1);

    // define nodeset pair 2
    NodeSet working0;
    NodeSet working1;
    selector.set_selection_criteria_y(0.);
    selector.select(&modular_interface, working0);
    selector.set_selection_criteria_x(xdist);
    selector.select(&modular_interface, working1);
    set_intersection(working0.m_nodes.begin(),working0.m_nodes.end(),
                     working1.m_nodes.begin(),working1.m_nodes.end(),
                     std::inserter(ns_source[2].m_nodes,ns_source[2].m_nodes.begin()));
    ns_source[2].set_index(3u);
    working0.m_nodes.clear();
    working1.m_nodes.clear();
    selector.set_selection_criteria_y(ydist);
    selector.select(&modular_interface, working0);
    selector.set_selection_criteria_x(0.);
    selector.select(&modular_interface, working1);
    set_intersection(working0.m_nodes.begin(),working0.m_nodes.end(),
                     working1.m_nodes.begin(),working1.m_nodes.end(),
                     std::inserter(ns_destination[2].m_nodes,ns_destination[2].m_nodes.begin()));
    ns_destination[2].set_index(5u);
    working0.m_nodes.clear();
    working1.m_nodes.clear();
    std::vector<double> s2dv2 = {-xdist, ydist, 0.};
    source_to_destination_vector[2] = Point(0u, s2dv2);

    // build matcher
    NodeSetMatcher matcher(&authority);
    matcher.infer_spatial_tolerance(&modular_interface);
    const double spatial_tolerance = matcher.get_spatial_tolerance();

    for(size_t nodeset_pair = 0u; nodeset_pair < num_nodeset_pairs; nodeset_pair++)
    {
        matcher.register_pair(&ns_source[nodeset_pair],
                              &ns_destination[nodeset_pair],
                              &source_to_destination_vector[nodeset_pair]);
    }

    double run_time = -getTimeInSeconds();
    matcher.match(&modular_interface, local_to_global);
    run_time += getTimeInSeconds();
    if(0u == authority.mpi_wrapper->get_rank())
    {
        std::cout << "global num nodes:" << modular_block.get_global_num_nodes()
                  << ",num processors:" << authority.mpi_wrapper->get_size()
                  << ",run time:" << run_time << std::endl;
    }

    std::vector<std::vector<std::pair<int, int> > > global_source_and_destination_pairs(num_nodeset_pairs);
    for(size_t nodeset_pair = 0u; nodeset_pair < num_nodeset_pairs; nodeset_pair++)
    {
        matcher.get_final_pairs(ns_source[nodeset_pair].get_index(),
                                ns_destination[nodeset_pair].get_index(),
                                global_source_and_destination_pairs[nodeset_pair]);
    }

    for(size_t nodeset_pair = 0u; nodeset_pair < num_nodeset_pairs; nodeset_pair++)
    {
        // weak checking of minimal conditions
        const int max_input_size = std::max(ns_source[nodeset_pair].size(), ns_destination[nodeset_pair].size());
        const int output_size = global_source_and_destination_pairs[nodeset_pair].size();
        EXPECT_GE(output_size, max_input_size);

        // build global_to_local
        std::map<int,int> global_to_local;
        invert_vector_to_map(local_to_global, global_to_local);

        // ensure all of ns_source and ns_destination found in final pairs
        std::vector<int> ns_source_globals(ns_source[nodeset_pair].m_nodes.begin(), ns_source[nodeset_pair].m_nodes.end());
        std::map<int,int> ns_source_to_index;
        invert_vector_to_map(ns_source_globals, ns_source_to_index);
        const size_t num_ns_source = ns_source_globals.size();
        std::vector<bool> ns_source_in_final(num_ns_source, false);
        std::vector<int> ns_destination_globals(ns_destination[nodeset_pair].m_nodes.begin(), ns_destination[nodeset_pair].m_nodes.end());
        std::map<int,int> ns_destination_to_index;
        invert_vector_to_map(ns_destination_globals, ns_destination_to_index);
        const size_t num_ns_destination = ns_destination_globals.size();
        std::vector<bool> ns_destination_in_final(num_ns_destination, false);
        const size_t num_final_pairs = global_source_and_destination_pairs[nodeset_pair].size();
        for(size_t final_pair = 0u; final_pair < num_final_pairs; final_pair++)
        {
            bool a_local = false;

            // expect each source or destination to be found exactly once
            const int this_source_global = global_source_and_destination_pairs[nodeset_pair][final_pair].first;
            std::map<int, int>::iterator source_local_it = global_to_local.find(this_source_global);
            if(source_local_it != global_to_local.end())
            {
                const int this_source_local = source_local_it->second;
                std::map<int, int>::iterator ns_source_it = ns_source_to_index.find(this_source_local);
                if(ns_source_it != ns_source_to_index.end())
                {
                    const int ns_source_index = ns_source_it->second;
                    EXPECT_EQ(ns_source_in_final[ns_source_index], false);
                    ns_source_in_final[ns_source_index] = true;

                    a_local |= true;
                }
            }

            const int this_destination_global = global_source_and_destination_pairs[nodeset_pair][final_pair].second;
            std::map<int, int>::iterator destination_local_it = global_to_local.find(this_destination_global);
            if(destination_local_it != global_to_local.end())
            {
                const int this_destination_local = destination_local_it->second;
                std::map<int, int>::iterator ns_destination_it = ns_destination_to_index.find(this_destination_local);
                if(ns_destination_it != ns_destination_to_index.end())
                {
                    const int ns_destination_index = ns_destination_it->second;
                    EXPECT_EQ(ns_destination_in_final[ns_destination_index], false);
                    ns_destination_in_final[ns_destination_index] = true;

                    a_local |= true;
                }
            }

            // expect each pair to contain at least one local
            EXPECT_EQ(a_local, true);
        }
        for(size_t ns_source_index = 0u; ns_source_index < num_ns_source; ns_source_index++)
        {
            EXPECT_EQ(ns_source_in_final[ns_source_index], true);
        }
        for(size_t ns_destination_index = 0u; ns_destination_index < num_ns_destination; ns_destination_index++)
        {
            EXPECT_EQ(ns_destination_in_final[ns_destination_index], true);
        }

        // matches that were found, are correct
        for(size_t final_pair = 0u; final_pair < num_final_pairs; final_pair++)
        {
            int global_source_index = global_source_and_destination_pairs[nodeset_pair][final_pair].first;
            Point* global_source = modular_block.get_global_point(global_source_index);
            int global_destination_index = global_source_and_destination_pairs[nodeset_pair][final_pair].second;
            Point* global_destination = modular_block.get_global_point(global_destination_index);

            Point estimated_destination = (*global_source) + source_to_destination_vector[nodeset_pair];
            const double distance_error = estimated_destination.distance(global_destination);
            EXPECT_GE(spatial_tolerance, distance_error);
        }
    }
}

}

}

