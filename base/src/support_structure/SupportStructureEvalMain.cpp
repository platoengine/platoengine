
/*------------------------------------------------------------------------*/
/*                 Copyright 2010, 2011 Sandia Corporation.                     */
/*  Under terms of Contract DE-AC04-94AL85000, there is a non-exclusive   */
/*  license for use of this work by or on behalf of the U.S. Government.  */
/*  Export of this program may require a license from the                 */
/*  United States Government.                                             */
/*------------------------------------------------------------------------*/

#if defined( STK_HAS_MPI )
#include <mpi.h>
#endif
#include <stk_mesh/base/MetaData.hpp>
#include <stk_mesh/base/Field.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>
#include <stk_util/parallel/ParallelReduce.hpp>
#include <stk_util/parallel/CommSparse.hpp>
#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_mesh/base/Types.hpp>
#include <stk_mesh/base/FieldRestriction.hpp>
#include "Ioss_Region.h"                // for Region, NodeSetContainer, etc
#include "Teuchos_CommandLineProcessor.hpp"

#include <iostream>

//using namespace plato;
//using namespace support_structure;

int main(int argc,  char **argv)
{
    // Initialize communicator
    stk::ParallelMachine *comm = new stk::ParallelMachine(stk::parallel_machine_init(&argc, &argv));
    if(!comm)
    {
        std::cout << "Failed to initialize the parallel machine." << std::endl;
        return false;
    }

    // parse command line arguments
    std::string inputFilename;
    std::string outputFilename;
    int iteration;
    Teuchos::CommandLineProcessor clp;
    clp.setDocString("support_structure_eval options");
    clp.setOption("input_mesh",  &inputFilename, "in mesh file (ExodusII).", true );
    clp.setOption("output_file",  &outputFilename, "output text filename to append to.", true );
    clp.setOption("iteration",  &iteration, "iteartion number.", true );

    Teuchos::CommandLineProcessor::EParseCommandLineReturn parseReturn =
            Teuchos::CommandLineProcessor::PARSE_SUCCESSFUL;
    try
    {
        parseReturn = clp.parse( argc, argv );
    }
    catch (std::exception exc)
    {
        std::cout << "Failed to parse the command line arguments." << std::endl;
        return false;
    }

    // Initialize stk::mesh data
    stk::mesh::MetaData *metaData = new stk::mesh::MetaData;
    stk::mesh::BulkData *bulkData = new stk::mesh::BulkData(*metaData, *comm);
    stk::io::StkMeshIoBroker *ioBroker = new stk::io::StkMeshIoBroker(*comm);
    ioBroker->set_bulk_data(*bulkData);

    // Read the exodus mesh
    ioBroker->set_option_to_not_collapse_sequenced_fields();
    ioBroker->property_add(Ioss::Property("DECOMPOSITION_METHOD", "RIB"));
    ioBroker->add_mesh_database(inputFilename, "exodus", stk::io::READ_MESH);
    ioBroker->create_input_mesh();
    ioBroker->add_all_mesh_fields_as_input_fields();
    ioBroker->populate_bulk_data();
    stk::mesh::Field<double> *supportStructureField = metaData->get_field<stk::mesh::Field<double> >(
            stk::topology::NODE_RANK, "support_structure");
    ioBroker->read_defined_input_fields(1);

    // Loop over all nodes and add up support structure values
    stk::mesh::Selector myselector = metaData->universal_part();
    stk::mesh::BucketVector const &node_buckets = bulkData->get_buckets(
            stk::topology::NODE_RANK, myselector );
    if(node_buckets.size() == 0)
    {
        std::cout << "Failed to find any nodes." << std::endl;
        return false;
    }
    double totalSum = 0.0;
    for ( stk::mesh::BucketVector::const_iterator bucket_iter = node_buckets.begin();
            bucket_iter != node_buckets.end();
            ++bucket_iter )
    {
        stk::mesh::Bucket &tmp_bucket = **bucket_iter;
        size_t num_nodes = tmp_bucket.size();
        for (size_t i=0; i<num_nodes; ++i)
        {
            stk::mesh::Entity nodeEntity = tmp_bucket[i];
            double* val = stk::mesh::field_data(*supportStructureField, nodeEntity);
            totalSum += *val;
        }
    }

    FILE *fp = fopen(outputFilename.c_str(), "a");
    if(fp)
    {
        fprintf(fp, "%d %lf\n", iteration, totalSum);
        fclose(fp);
    }

    delete bulkData;
    delete metaData;
    delete ioBroker;
    stk::parallel_machine_finalize();


    return 0;
}
