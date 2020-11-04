/*
 * MeshRefine.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: tzirkle
 */

#include "MeshRefine.hpp"

    MeshRefine::MeshRefine()
    {

    }

    void MeshRefine::refine_mesh(MeshManager &tMeshManager)
    {
        PerceptMesh & tPerceptMesh = tMeshManager.get_output_percept();

        tPerceptMesh.reopen();

        stk::mesh::PartVector parts;
        tPerceptMesh.get_parts_of_rank(tPerceptMesh.element_rank(), parts);
        int has_hex8s = 0;
        int has_tet4s = 0;
        int has_tet10s = 0;
        for(size_t i=0; i<parts.size(); ++i)
        {
            if(parts[i]->topology() == stk::topology::TET_4)
                has_tet4s = 1;
            if(parts[i]->topology() == stk::topology::HEX_8)
                has_hex8s = 1;
            if(parts[i]->topology() == stk::topology::TET_10)
                has_tet10s = 1;
        }

        int sum = has_hex8s + has_tet4s + has_tet10s;
        if(sum != 1)
        {
            std::cout << "\nERROR::refine: Unsupported or mixed elements types in mesh to be refined.\n";
            return;
        }

        if(has_hex8s)
        {

            Hex8_Hex8_8 break_hex_to_hex(tPerceptMesh);

            int scalarDimension = 0; // a scalar
            stk::mesh::FieldBase* proc_rank_field = tPerceptMesh.add_field("proc_rank", stk::topology::ELEMENT_RANK, scalarDimension);
            tPerceptMesh.commit();


            UniformRefiner breaker(tPerceptMesh, break_hex_to_hex, proc_rank_field);

            //        breaker.setGeometryFile("/ascldap/users/tzirkle/Documents/PruneTest_SFTP/PruneTestM2G.sat");
            breaker.doBreak();
        }
        else if(has_tet4s)
        {
            Tet4_Tet4_8 break_tet_to_tet(tPerceptMesh);

            int scalarDimension = 0; // a scalar
            stk::mesh::FieldBase* proc_rank_field = tPerceptMesh.add_field("proc_rank", stk::topology::ELEMENT_RANK, scalarDimension);
            tPerceptMesh.commit();


            UniformRefiner breaker(tPerceptMesh, break_tet_to_tet, proc_rank_field);

            //        breaker.setGeometryFile("/ascldap/users/tzirkle/Documents/PruneTest_SFTP/PruneTestM2G.sat");
            breaker.doBreak();
        }
        else if(has_tet10s)
        {
            Tet10_Tet10_8 break_tet_to_tet(tPerceptMesh);

            int scalarDimension = 0; // a scalar
            stk::mesh::FieldBase* proc_rank_field = tPerceptMesh.add_field("proc_rank", stk::topology::ELEMENT_RANK, scalarDimension);
            tPerceptMesh.commit();


            UniformRefiner breaker(tPerceptMesh, break_tet_to_tet, proc_rank_field);

            //        breaker.setGeometryFile("/ascldap/users/tzirkle/Documents/PruneTest_SFTP/PruneTestM2G.sat");
            breaker.doBreak();
        }
    }



