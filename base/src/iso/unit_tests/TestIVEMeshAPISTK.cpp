
/*--------------------------------------------------------------------*/
/*    Copyright 2002 - 2008, 2010, 2011 National Technology &         */
/*    Engineering Solutions of Sandia, LLC (NTESS). Under the terms   */
/*    of Contract DE-NA0003525 with NTESS, there is a                 */
/*    non-exclusive license for use of this work by or on behalf      */
/*    of the U.S. Government.  Export of this program may require     */
/*    a license from the United States Government.                    */
/*--------------------------------------------------------------------*/
#include <gtest/gtest.h>

#include "IVEMeshAPISTK.hpp"
#include "stk_io/StkMeshIoBroker.hpp"


namespace iso {

//void build_fixture(iobroker, "1x1x3");
static void build_fixture
(
  IVEMeshAPISTK &mesh_api,
  stk::io::StkMeshIoBroker &stkMeshIoBroker,
  const std::string &nxXnyXnz
)
{
  std::string generatedMeshSpecification = "generated:";
  generatedMeshSpecification += nxXnyXnz;
  stkMeshIoBroker.add_mesh_database(generatedMeshSpecification, stk::io::READ_MESH);
  stkMeshIoBroker.create_input_mesh();

  stk::mesh::MetaData &meta = stkMeshIoBroker.meta_data();
  mesh_api.set_meta_data_ptr(&meta);

  mesh_api.prepare_to_create_tris();

  stkMeshIoBroker.populate_bulk_data();

  stk::mesh::BulkData &bulk = stkMeshIoBroker.bulk_data();
  mesh_api.set_bulk_data_ptr(&bulk);
}

TEST(MSMeshSTK, test_global_iterators)
{
  //================================================
  // Build a MSMeshSTK interface to test
  //================================================

  IVEMeshAPISTK mesh_api(NULL);

  //================================================
  // Build a mesh for testing, 2x2x2
  //================================================
  stk::io::StkMeshIoBroker iobroker(MPI_COMM_WORLD);
  build_fixture(mesh_api, iobroker,"2x2x2");


  ////////////////// Test an api //////////////////////
}

} // namespace iso





