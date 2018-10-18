/*------------------------------------------------------------------------*/
/*                 Copyright 2010, 2011 Sandia Corporation.                     */
/*  Under terms of Contract DE-AC04-94AL85000, there is a non-exclusive   */
/*  license for use of this work by or on behalf of the U.S. Government.  */
/*  Export of this program may require a license from the                 */
/*  United States Government.                                             */
/*------------------------------------------------------------------------*/
#include "SupportStructure.hpp"

#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_io/IossBridge.hpp>
#include <stk_mesh/base/CreateFaces.hpp>
#include <stk_mesh/base/CreateEdges.hpp>
#include <stk_mesh/base/MetaData.hpp>
#include <stk_mesh/base/DestroyElements.hpp>
#include <stk_util/parallel/CommSparse.hpp>

#include <Ioss_SubSystem.h>

namespace plato
{

namespace support_structure
{

SupportStructure::~SupportStructure()
{
    if(mSTKMeshIn)
        delete mSTKMeshIn;
    if(mSTKMeshOut)
        delete mSTKMeshOut;
}

bool SupportStructure::createMeshAPIsStandAlone(int argc, char **argv)
{
    // Set some defaults
    mMeshIn = "";
    mMeshOut = "";
    mDesignFieldName = "Topology";
    mSTKMeshIn = NULL;
    mSTKMeshOut = NULL;
    mDesignFieldThresholdValue = 0.5;
    mConcatenateResults = 0;
    mReadSpreadFile = 0;
    mTimeStep = 1;
    mBuildPlateNormal[0] = 0.0;
    mBuildPlateNormal[1] = 0.0;
    mBuildPlateNormal[2] = 1.1;
    mOutputFields = "";

    stk::ParallelMachine *comm = new stk::ParallelMachine(stk::parallel_machine_init(&argc, &argv));
    if(!comm)
    {
        std::cout << "Failed to initialize the parallel machine." << std::endl;
        return false;
    }
    mComm = comm;
    if ( !readCommandLine( argc, argv ) )
        return false;

    if(!initSingleMeshAPIs())
    {
        return false;
    }

    return true;
}

bool SupportStructure::initSingleMeshAPIs()
{

    mSTKMeshIn = new MeshWrapper(mComm);
    mSTKMeshIn->prepare_as_source();

    if(!mSTKMeshIn->read_exodus_mesh(mMeshIn, mDesignFieldName, mOutputFields, mReadSpreadFile, mTimeStep))
        return false;

    mSTKMeshOut = mSTKMeshIn;

    return true;
}


bool SupportStructure::run()
{
    bool return_val = true;
    try
    {
        return_val = runPrivate();
    }
    catch (std::exception exc)
    {
        delete mSTKMeshIn;
        mSTKMeshIn = mSTKMeshOut = NULL;
        stk::parallel_machine_finalize();
        return false;
    }

    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
    {
        if ( return_val )
            std::cout << "Exiting Successfully " << std::endl;
        else
            std::cout << "Exiting with Failure " << std::endl;
    }

    delete mSTKMeshIn;
    mSTKMeshIn = mSTKMeshOut = NULL;
    stk::parallel_machine_finalize();

    return return_val;
}

bool SupportStructure::runPrivate()
{
    bool return_val = true;
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        std::cout << "Generating support material field. " << std::endl;

    stk::mesh::Selector myselector = mSTKMeshIn->meta_data()->universal_part();
    stk::mesh::BucketVector const &node_buckets = mSTKMeshIn->bulk_data()->get_buckets(
            stk::topology::NODE_RANK, myselector );

    if(node_buckets.size() == 0)
    {
        std::cout << "Failed to find any nodes." << std::endl;
        return false;
    }

    // Get initial set of nodes to check
    std::vector<uint64_t> myNodesToCheck;
    for ( stk::mesh::BucketVector::const_iterator bucket_iter = node_buckets.begin();
            bucket_iter != node_buckets.end();
            ++bucket_iter )
    {
        stk::mesh::Bucket &tmp_bucket = **bucket_iter;
        size_t num_nodes = tmp_bucket.size();
        for (size_t i=0; i<num_nodes; ++i)
        {
            stk::mesh::Entity cur_node = tmp_bucket[i];
            uint64_t localId = cur_node.m_value;
            double val = mSTKMeshIn->getMaxNodalIsoFieldVariable(localId);
            if(val >= mDesignFieldThresholdValue)
            {
                myNodesToCheck.push_back(localId);
            }
            // Initialize all support structure values to 0.0
            mSTKMeshIn->setSetSupportStructureFieldValue(localId, 0.0);
        }
    }

    // Get the build plate normal
    std::string workingString = mBuildPlateNormalString; // make a copy since we will be modifying it
    size_t spacePos = workingString.find(' ');
    int cntr = 0;
    while(spacePos != std::string::npos)
    {
        std::string cur_string = workingString.substr(0,spacePos);
        workingString = workingString.substr(spacePos+1);
        mBuildPlateNormal[cntr] = std::atof(cur_string.c_str());
        cntr++;
        spacePos = workingString.find(' ');
    }
    mBuildPlateNormal[cntr] = std::atof(workingString.c_str());
    // Normalize the build plate normal in case it isn't already
    double mag = sqrt(mBuildPlateNormal[0]*mBuildPlateNormal[0] +
                      mBuildPlateNormal[1]*mBuildPlateNormal[1] +
                      mBuildPlateNormal[2]*mBuildPlateNormal[2]);
    mBuildPlateNormal[0] /= mag;
    mBuildPlateNormal[1] /= mag;
    mBuildPlateNormal[2] /= mag;


    std::vector<proc_node_map> procs;
    calculateSharingMaps(procs, mSTKMeshIn);

    std::set<uint64_t> mySupportMaterialNodes;
    std::set<uint64_t> myNewSupportMaterialNodes;
    std::vector<uint64_t> myNewNodesToCheck;
    int numProcs = mSTKMeshIn->bulk_data()->parallel_size();
    int myRank = mSTKMeshIn->bulk_data()->parallel_rank();
    bool done = false;
    size_t iterationCount = 1;
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
    {
        std::cout << "Starting to search for support structure nodes." << std::endl;
    }
    while(!done)
    {
        if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
        {
            std::cout << "Iteration " << iterationCount << ":" << std::endl;
        }
        myNewNodesToCheck.clear();
        myNewSupportMaterialNodes.clear();
        while(myNodesToCheck.size() > 0)
        {
            uint64_t curNode = myNodesToCheck.back();
            myNodesToCheck.pop_back();
            uint64_t newSupportNodeLocalId;
            stk::mesh::Entity curNodeEntity = mSTKMeshIn->getStkEntity(curNode);
            if(nodeNeedsSupport(curNodeEntity, newSupportNodeLocalId, mySupportMaterialNodes))
            {
                myNewNodesToCheck.push_back(newSupportNodeLocalId);
                mySupportMaterialNodes.insert(newSupportNodeLocalId);
                mSTKMeshIn->setSetSupportStructureFieldValue(newSupportNodeLocalId, 1.0);
                stk::mesh::Entity newSupportNodeEntity = mSTKMeshIn->getStkEntity(newSupportNodeLocalId);
                uint64_t globalNodeId = mSTKMeshIn->bulk_data()->identifier(newSupportNodeEntity);
                myNewSupportMaterialNodes.insert(globalNodeId);
            }
        }

        // Get the counts of new support structure nodes from each processsor.
        int globalNewSupportNodeCounts[numProcs];
        int localNewSupportNodeCounts[numProcs];
        for(int i=0; i<numProcs; ++i)
        {
            globalNewSupportNodeCounts[i] = 0;
            localNewSupportNodeCounts[i] = 0;
        }
        localNewSupportNodeCounts[myRank] = myNewSupportMaterialNodes.size();

        MPI_Allreduce(localNewSupportNodeCounts, globalNewSupportNodeCounts, numProcs,
                      MPI_INT, MPI_SUM, mSTKMeshIn->bulk_data()->parallel());

        int d;
        bool newSupportNodesWereFound = false;
        for(d=0; d<numProcs; ++d)
        {
            if(globalNewSupportNodeCounts[d] > 0)
            {
                newSupportNodesWereFound = true;
                break;
            }
        }

        bool printData = true;
        if(printData)
        {
            // Now actually get the ids of the new support structure nodes from each processor
            // First allocate/initialize memory for ids from all the processors
            int* localNewSupportNodeIds[numProcs];
            int* globalNewSupportNodeIds[numProcs];
            for(d=0; d<numProcs; ++d)
            {
                if(globalNewSupportNodeCounts[d] > 0)
                {
                    localNewSupportNodeIds[d] = new int [globalNewSupportNodeCounts[d]];
                    globalNewSupportNodeIds[d] = new int [globalNewSupportNodeCounts[d]];
                    for(int e=0; e<globalNewSupportNodeCounts[d]; e++)
                    {
                        localNewSupportNodeIds[d][e] = 0;
                        globalNewSupportNodeIds[d][e] = 0;
                    }
                }
                else
                {
                    localNewSupportNodeIds[d] = NULL;
                    globalNewSupportNodeIds[d] = NULL;
                }
            }
            // Initialize the node id values from my processor
            std::set<uint64_t>::iterator idIterator = myNewSupportMaterialNodes.begin();
            d=0;
            while(idIterator != myNewSupportMaterialNodes.end())
            {
                localNewSupportNodeIds[myRank][d] = *idIterator;
                idIterator++;
                d++;
            }
            // Now communicate all the data
            for(d=0; d<numProcs; ++d)
            {
                if(globalNewSupportNodeCounts[d] > 0)
                {
                    // Only doing MPI_Reduce so proc 0 has all the data.
                    MPI_Reduce(localNewSupportNodeIds[d], globalNewSupportNodeIds[d], globalNewSupportNodeCounts[d],
                               MPI_INT, MPI_SUM, 0, mSTKMeshIn->bulk_data()->parallel());
                }
            }
            if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
            {
                std::set<int> uniqueNodeIdList;
                for(d=0; d<numProcs; ++d)
                {
                    for(int f=0; f<globalNewSupportNodeCounts[d]; f++)
                    {
                        uniqueNodeIdList.insert(globalNewSupportNodeIds[d][f]);
                    }
                }
                std::cout << "Total number of new support nodes on iteration " << iterationCount << ": " << uniqueNodeIdList.size() << std::endl;
                std::set<int>::iterator nodeIter = uniqueNodeIdList.begin();
                std::cout << "Node ids: ";
                while(nodeIter != uniqueNodeIdList.end())
                {
                    std::cout << *nodeIter << " ";
                    nodeIter++;
                }
                std::cout << std::endl;
            }

            // Now free all the data we dynamically allocated
            for(d=0; d<numProcs; ++d)
            {
                if(globalNewSupportNodeCounts[d] > 0)
                {
                    delete [] localNewSupportNodeIds[d];
                    delete [] globalNewSupportNodeIds[d];
                }
            }
        }

        if(!newSupportNodesWereFound)
        {
            done = true;
        }
        else
        {
            myNodesToCheck = myNewNodesToCheck;
            communicateBoundaryInfo(procs, myNewSupportMaterialNodes, mySupportMaterialNodes, myNodesToCheck);
        }

        iterationCount++;
    }
    if(mSTKMeshIn->bulk_data()->parallel_rank() == 0)
    {
        std::cout << "Total number of support " << std::endl;
    }

    mSTKMeshIn->write_exodus_mesh(mMeshOut, mConcatenateResults);

    return return_val;
}

void SupportStructure::communicateBoundaryInfo(std::vector<proc_node_map> &procs,
                                               std::set<uint64_t> &aNewSupportNodes,
                                               std::set<uint64_t> &aAllSupportNodes,
                                               std::vector<uint64_t> &aNodesToCheck)
{
    if(procs.size() > 0)
    {
        stk::CommSparse comm_spec(mSTKMeshIn->bulk_data()->parallel());
        for(int phase=0; phase<2; ++phase)
        {
            for(size_t i=0; i<procs.size(); ++i)
            {
                int other_proc = procs[i].processor;
                // pack the number of nodes
                comm_spec.send_buffer(other_proc).pack<size_t>(procs[i].nodes.size());
                for(size_t j=0; j<procs[i].nodes.size(); ++j)
                {
                    // pack the global node id and then the value for this node
                    comm_spec.send_buffer(other_proc).pack<uint64_t>(procs[i].nodes[j]);
                    int isSupportNode=0;
                    if(aNewSupportNodes.find(procs[i].nodes[j]) != aNewSupportNodes.end())
                        isSupportNode=1;
                    comm_spec.send_buffer(other_proc).pack<int>(isSupportNode);
                }
            }
            if ( phase == 0 )
            {
                comm_spec.allocate_buffers();
            }
            else
            {
                comm_spec.communicate();
            }
        }

        for(size_t i=0; i<procs.size(); ++i)
        {
            int other_proc = procs[i].processor;
            // get the number of nodes from the sending proc
            size_t num_nodes;
            comm_spec.recv_buffer(other_proc).unpack<size_t>(num_nodes);
            for(size_t j=0; j<num_nodes; ++j)
            {
                uint64_t globalNodeId;
                int node_val;
                comm_spec.recv_buffer(other_proc).unpack<uint64_t>(globalNodeId);
                comm_spec.recv_buffer(other_proc).unpack<int>(node_val);
                stk::mesh::EntityId entityId = globalNodeId;
                stk::mesh::Entity nodeEntity = mSTKMeshIn->bulk_data()->get_entity(stk::topology::NODE_RANK, entityId);
                uint64_t localNodeId = nodeEntity.m_value;
                if(node_val > 0 && aAllSupportNodes.find(localNodeId) == aAllSupportNodes.end())
                {
                    mSTKMeshIn->setSetSupportStructureFieldValue(localNodeId, 1.0);
                    aAllSupportNodes.insert(localNodeId);
                    aNodesToCheck.push_back(localNodeId);
                }
            }
        }
    }
}

bool SupportStructure::nodeNeedsSupport(stk::mesh::Entity &aCurNode, uint64_t &aNewSupportNodeLocalId,
                                        std::set<uint64_t> &aExistingSupportMaterialNodes)
{
    double tolerance = 1e-6;
    double inCoords[3];
    double bestDot = -1.0;
    stk::mesh::Entity bestNode;
    uint64_t bestNodeLocalId;
    mSTKMeshIn->nodeCoordinates(aCurNode, inCoords);
    stk::mesh::Entity const *nodeElements = mSTKMeshIn->bulk_data()->begin_elements(aCurNode);
    int numElems = mSTKMeshIn->bulk_data()->num_elements(aCurNode);
    for(int i=0; i<numElems; ++i)
    {
        stk::mesh::Entity curElem = nodeElements[i];
        stk::mesh::Entity const *ElemNodes = mSTKMeshIn->bulk_data()->begin_nodes(curElem);
        int numNodes = mSTKMeshIn->bulk_data()->num_nodes(curElem);
        for(int j=0; j<numNodes; ++j)
        {
            stk::mesh::Entity curNode = ElemNodes[j];
            if(curNode != aCurNode)
            {
                double curCoords[3];
                mSTKMeshIn->nodeCoordinates(curNode, curCoords);
                double curVec[3];
                for(int h=0; h<3; ++h)
                    curVec[h] = inCoords[h]-curCoords[h];
                double mag = 0.0;
                for(int h=0; h<3; ++h)
                    mag += curVec[h]*curVec[h];
                mag = sqrt(mag);
                for(int h=0; h<3; ++h)
                    curVec[h] /= mag;
                double dot = 0.0;
                for(int h=0; h<3; ++h)
                    dot += curVec[h]*mBuildPlateNormal[h];
                if(dot > .707)
                {
                    if(dot > bestDot)
                    {
                        bestDot = dot;
                        bestNode = curNode;
                        bestNodeLocalId = bestNode.m_value;
                    }
                }
            }
        }
    }
    if(bestDot > 0.0)
    {
        if(aExistingSupportMaterialNodes.find(bestNodeLocalId) == aExistingSupportMaterialNodes.end())
        {
            double designVariableValue = mSTKMeshIn->getMaxNodalIsoFieldVariable(bestNodeLocalId);
            if((designVariableValue + tolerance) < mDesignFieldThresholdValue)
            {
                aNewSupportNodeLocalId = bestNodeLocalId;
                return true;;
            }
        }
    }
    return false;
}

void SupportStructure::calculateSharingMaps(std::vector<proc_node_map> &aProcs,
                                            MeshWrapper *aMesh)
{
    // Find which owned elems are on the boundary
    std::set<uint64_t> sharedBoundaryNodes;
    aMesh->get_shared_boundary_nodes(sharedBoundaryNodes);
    std::set<uint64_t>::iterator it = sharedBoundaryNodes.begin();
    while(it != sharedBoundaryNodes.end())
    {
        uint64_t curBoundaryNode = *it;
        stk::mesh::EntityId node_id = curBoundaryNode;
        stk::mesh::EntityKey key(stk::topology::NODE_RANK, node_id);
        std::vector<int> sharingProcs;
        aMesh->bulk_data()->comm_shared_procs(key, sharingProcs);
        for(size_t i=0; i<sharingProcs.size(); ++i)
        {
            int other_proc_id = sharingProcs[i];
            // if this is not me...
            if(other_proc_id != aMesh->bulk_data()->parallel_rank())
            {
                size_t j = 0;
                for(; j<aProcs.size(); ++j)
                {
                    if(aProcs[j].processor == other_proc_id)
                    {
                        aProcs[j].nodes.push_back(curBoundaryNode);
                        break;
                    }
                }
                // if we don't have an entry for this proc yet...
                if(j == aProcs.size())
                {
                    proc_node_map new_proc;
                    new_proc.processor = other_proc_id;
                    new_proc.nodes.push_back(curBoundaryNode);
                    aProcs.push_back(new_proc);
                }
            }
        }
        ++it;
    }
}

bool SupportStructure::readCommandLine( int argc, char *argv[])
{
    Teuchos::CommandLineProcessor clp;
    clp.setDocString("create_support_structure options");

    clp.setOption("input_mesh",  &mMeshIn, "in mesh file (ExodusII).", true );
    clp.setOption("output_mesh",  &mMeshOut, "out mesh file (ExodusII).", false );
    clp.setOption("design_field_name",  &mDesignFieldName, "field defining level set data.", false );
    clp.setOption("concatenate_results",  &mConcatenateResults, "specify whether to concatenate resulting mesh files.", false );
    clp.setOption("read_spread_file",  &mReadSpreadFile, "specify whether input is already decomposed.", false );
    clp.setOption("design_field_value",  &mDesignFieldThresholdValue, "specify the design variable threshold value.", false );
    clp.setOption("time_step",  &mTimeStep, "specify the time step to be read from the file.", false );
    clp.setOption("output_fields",  &mOutputFields, "specify the fields (commma separated, no spaces) to output in the output mesh.", false );
    clp.setOption("build_plate_normal",  &mBuildPlateNormalString, "specify the normal of the build plate (values separated by spaces)", false );

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

    if ( parseReturn == Teuchos::CommandLineProcessor::PARSE_SUCCESSFUL )
        return true;

    std::cout << "Failed to parse the command line arguments." << std::endl;
    return false;
}

} //namespace support_structure

} //namespace plato




