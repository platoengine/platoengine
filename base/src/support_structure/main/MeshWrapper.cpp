/*------------------------------------------------------------------------*/
/*                 Copyright 2010, 2011 Sandia Corporation.                     */
/*  Under terms of Contract DE-AC04-94AL85000, there is a non-exclusive   */
/*  license for use of this work by or on behalf of the U.S. Government.  */
/*  Export of this program may require a license from the                 */
/*  United States Government.                                             */
/*------------------------------------------------------------------------*/
//-------------------------------------------------------------------------
// Filename      : MeshWrapper.cpp
//
// Description   : a MeshWrapper is a wrapper around stk::mesh
//
// Creator       : Brett Clark
//
// Creation Date : 10/10/2018
//
// Owner         : Brett Clark
//-------------------------------------------------------------------------

#include "MeshWrapper.hpp"
#include <stk_mesh/base/MetaData.hpp>
#include <stk_mesh/base/Field.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>
#include <stk_util/parallel/ParallelReduce.hpp>
#include <stk_util/parallel/CommSparse.hpp>
#include <stk_io/StkMeshIoBroker.hpp>
#include <stk_mesh/base/Types.hpp>
#include <stk_mesh/base/FieldRestriction.hpp>
#include "Ioss_Region.h"                // for Region, NodeSetContainer, etc

namespace plato
{


MeshWrapper::MeshWrapper(stk::ParallelMachine* comm)
{
    initialize();
    mComm = comm;
}

MeshWrapper::~MeshWrapper()
{
    if ( mLocallyOwnedBulk && mBulkData )
        delete mBulkData;
    if ( mLocallyOwnedMeta && mMetaData )
        delete mMetaData;
    if(mIoBroker)
        delete mIoBroker;
}

void MeshWrapper::initialize()
{
    mMetaData = NULL;
    mBulkData = NULL;
    mIoBroker = NULL;
    mLocallyOwnedBulk = false;
    mLocallyOwnedMeta = false;
    mCoordsField = NULL;
    mTimeStep = 0;
}

bool MeshWrapper::prepare_as_source()
{
    mMetaData = new stk::mesh::MetaData;
    mLocallyOwnedMeta = true;
    mBulkData = new stk::mesh::BulkData(*mMetaData, *mComm);
    mLocallyOwnedBulk = true;
    mIoBroker = new stk::io::StkMeshIoBroker(*mComm);
    mIoBroker->set_bulk_data(*mBulkData);
    return true;
}

void MeshWrapper::get_shared_boundary_nodes(std::set<uint64_t> &aSharedBoundaryNodes)
{
    stk::mesh::Selector shared(mMetaData->globally_shared_part());
    const stk::mesh::BucketVector& sharedNodeBuckets = mBulkData->get_buckets(stk::topology::NODE_RANK,
                                                                              shared);
    for(size_t i=0; i<sharedNodeBuckets.size(); ++i)
    {
        for(size_t j=0; j<sharedNodeBuckets[i]->size(); ++j)
        {
            stk::mesh::Entity cur_node = (*(sharedNodeBuckets[i]))[j];
            aSharedBoundaryNodes.insert(mBulkData->identifier(cur_node));
        }
    }
}

void MeshWrapper::nodeCoordinates(stk::mesh::Entity &aNode, double aCoords[3])
{
    double* vals = stk::mesh::field_data(*mCoordsField, aNode);
    aCoords[0] = vals[0];
    aCoords[1] = vals[1];
    aCoords[2] = vals[2];
}

stk::mesh::Entity MeshWrapper::getStkEntity(const uint64_t &handle) const
{
    stk::mesh::Entity entity;
    entity.m_value = handle;
    return entity;
}

double MeshWrapper::getMaxNodalIsoFieldVariable(uint64_t node) const
{
    double maxVal = 0.0;
    stk::mesh::Entity nodeEntity = getStkEntity(node);
    for(size_t i=0; i<mIsoFields.size(); ++i)
    {
        double* vals = stk::mesh::field_data(*(mIsoFields[i]), nodeEntity);
        if(*vals > maxVal)
            maxVal = *vals;
    }
    return maxVal;
}

void MeshWrapper::setSetSupportStructureFieldValue(uint64_t nodeLocalId, double value)
{
    stk::mesh::Entity nodeEntity = getStkEntity(nodeLocalId);
    double* val = stk::mesh::field_data(*mSupportStructureField, nodeEntity);
    *val = value;
}

bool MeshWrapper::read_exodus_mesh( std::string &aMeshFile, std::string &aFieldName,
                                    std::string &aOutputFieldsString,
                                    int aInputFileIsSpread,
                                    int aTimeStep )
{
    mSupportStructureField = &(mMetaData->declare_field<stk::mesh::Field<double> >
    (stk::topology::NODE_RANK, "support_structure"));
    stk::mesh::put_field_on_entire_mesh(*mSupportStructureField);

    mIoBroker->set_option_to_not_collapse_sequenced_fields();
    if(!aInputFileIsSpread)
        mIoBroker->property_add(Ioss::Property("DECOMPOSITION_METHOD", "RIB"));
    mIoBroker->add_mesh_database(aMeshFile, "exodus", stk::io::READ_MESH);
    mIoBroker->create_input_mesh();

    mIoBroker->add_all_mesh_fields_as_input_fields();

    get_output_fields(aOutputFieldsString);

    mIoBroker->populate_bulk_data();

    mCoordsField = mMetaData->get_field<stk::mesh::Field<double, stk::mesh::Cartesian> >
    (stk::topology::NODE_RANK, "coordinates");

    if(!mCoordsField)
    {
        std::cout << "Failed to find nodal coordinate field." << std::endl;
        return false;
    }

    int debug_print = 0;
    if(debug_print)
    {
        export_my_mesh();
        return false;
    }

    // The fieldname string can contain multiple comma-separated names
    std::string working_string = aFieldName; // make a copy since we will be modifying it
    std::vector<std::string> parsed_strings;
    size_t comma_pos = working_string.find(',');
    while(comma_pos != std::string::npos)
    {
        std::string cur_string = working_string.substr(0,comma_pos);
        working_string = working_string.substr(comma_pos+1);
        parsed_strings.push_back(cur_string);
        comma_pos = working_string.find(',');
    }

    parsed_strings.push_back(working_string);
    for(size_t i=0; i<parsed_strings.size(); ++i)
    {
        stk::mesh::Field<double> *cur_field = mMetaData->get_field<stk::mesh::Field<double> >(
                stk::topology::NODE_RANK, parsed_strings[i]);
        if(!cur_field)
        {
            if(mBulkData && mBulkData->parallel_rank() == 0)
            {
                std::cout << "Failed to find " << parsed_strings[i] << " nodal variable." << std::endl;
                return false;
            }
        }
        else
        {
            mIsoFields.push_back(cur_field);
        }
    }

    if(aTimeStep < 1)
        aTimeStep = mIoBroker->get_input_io_region().get()->get_property("state_count").get_int();
    mIoBroker->read_defined_input_fields(aTimeStep);

    return true;
}

void MeshWrapper::export_my_mesh()
{
    int p_rank = mBulkData->parallel_rank();
    char filename[1000];
    sprintf(filename, "mesh_proc_%d.txt", p_rank);
    FILE *fp = fopen(filename, "w");
    if(fp)
    {
        fprintf(fp, "Owned Nodes\n");
        stk::mesh::Selector sel(mMetaData->locally_owned_part());
        const stk::mesh::BucketVector &node_buckets = mBulkData->get_buckets(stk::topology::NODE_RANK, sel);
        for(size_t i=0; i<node_buckets.size(); ++i)
        {
            for(size_t j=0; j<node_buckets[i]->size(); ++j)
            {
                stk::mesh::Entity cur_node = (*(node_buckets[i]))[j];
                double* vals = stk::mesh::field_data(*mCoordsField, cur_node);
                unsigned long id = mBulkData->identifier(cur_node);
                fprintf(fp, "%lu %lf %lf %lf\n", id, vals[0], vals[1], vals[2]);
            }
        }
        fprintf(fp, "Shared Nodes\n");
        stk::mesh::Selector sel2(mMetaData->globally_shared_part());
        const stk::mesh::BucketVector &node_buckets2 = mBulkData->get_buckets(stk::topology::NODE_RANK, sel2);
        for(size_t i=0; i<node_buckets2.size(); ++i)
        {
            for(size_t j=0; j<node_buckets2[i]->size(); ++j)
            {
                stk::mesh::Entity cur_node = (*(node_buckets2[i]))[j];
                double* vals = stk::mesh::field_data(*mCoordsField, cur_node);
                unsigned long id = mBulkData->identifier(cur_node);
                fprintf(fp, "%lu %lf %lf %lf\n", id, vals[0], vals[1], vals[2]);
            }
        }
        fclose(fp);
    }
}

void MeshWrapper::get_output_fields(std::string &aOutputFieldsString)
{
    std::string working_string = aOutputFieldsString; // make a copy since we will be modifying it
    std::vector<std::string> parsed_strings;
    size_t comma_pos = working_string.find(',');
    while(comma_pos != std::string::npos)
    {
        std::string cur_string = working_string.substr(0,comma_pos);
        working_string = working_string.substr(comma_pos+1);
        parsed_strings.push_back(cur_string);
        comma_pos = working_string.find(',');
    }
    if(working_string.size() > 0)
        parsed_strings.push_back(working_string);
    mNodalFields.clear();
    mElementFields.clear();
    stk::mesh::FieldVector all_fields = mMetaData->get_fields();
    for(size_t i=0; i<parsed_strings.size(); i++)
    {
        for(size_t j=0; j<all_fields.size(); ++j)
        {
            stk::mesh::FieldBase* cur_file_field = all_fields[j];
            std::string file_field_name = cur_file_field->name();
            if(parsed_strings[i] == file_field_name)
            {
                if(cur_file_field->type_is<double>())
                {
                    if(cur_file_field->entity_rank() == stk::topology::NODE_RANK)
                    {
                        mNodalFields.push_back((stk::mesh::Field<double>*)cur_file_field);
                        j = all_fields.size(); // break out
                    }
                    else if(cur_file_field->entity_rank() == stk::topology::ELEMENT_RANK)
                    {
                        mElementFields.push_back((stk::mesh::Field<double>*)cur_file_field);
                        j = all_fields.size(); // break out
                    }
                }
            }
        }
    }
    if((mNodalFields.size() + mElementFields.size()) != parsed_strings.size())
    {
        if(bulk_data()->parallel_rank() == 0)
            std::cout << "Error: Could not find all of the requested output fields!\n" << std::endl;
    }
}

void MeshWrapper::write_exodus_mesh( std::string &meshfile, int concatenate)
{
    if(concatenate)
        mIoBroker->property_add(Ioss::Property("COMPOSE_RESULTS", true));
    size_t fh = mIoBroker->create_output_mesh(meshfile, stk::io::WRITE_RESULTS);

    // Make sure to write the iso field out.
    for(size_t i=0; i<mIsoFields.size(); ++i)
    {
        mIoBroker->add_field(fh, *(mIsoFields[i]));
    }
    mIoBroker->add_field(fh, *mSupportStructureField);
    for(size_t i=0; i<mNodalFields.size(); ++i)
    {
        mIoBroker->add_field(fh, *(mNodalFields[i]));
    }
    for(size_t i=0; i<mElementFields.size(); ++i)
    {
        mIoBroker->add_field(fh, *(mElementFields[i]));
    }
    mIoBroker->begin_output_step(fh, 0);
    mIoBroker->write_defined_output_fields(fh);
    mIoBroker->end_output_step(fh);
    mIoBroker->write_output_mesh(fh);
}

} //namespace plato



