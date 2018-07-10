/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

/*
 * InputXMLChecker.cpp
 *
 *  Created on: Aug 4, 2017
 *
 */

#include <set>
#include <string>
#include <iostream>
#include "InputXMLChecker.hpp"


/******************************************************************************/
InputXMLChecker::InputXMLChecker(const std::string &interface_xml_filename,
                                 const std::string &plato_main_operations_filename,
                                 const std::vector<std::string> &physics_app_files)
/******************************************************************************/
{
  pugi::xml_parse_result result = mInterface.load_file(interface_xml_filename.c_str());
  result = mPlatoOperations.load_file(plato_main_operations_filename.c_str());
  for(size_t i = 0; i<physics_app_files.size(); ++i)
  {
      pugi::xml_document *cur_doc = new pugi::xml_document;
      result = cur_doc->load_file(physics_app_files[i].c_str());
      mPhysicsAppOperations.push_back(cur_doc);
  }
}

/******************************************************************************/
InputXMLChecker::~InputXMLChecker()
/******************************************************************************/
{
    for(size_t i=0; i<mPhysicsAppOperations.size(); ++i)
    {
        delete mPhysicsAppOperations[i];
    }
}

/******************************************************************************/
bool InputXMLChecker::check_performers()
/******************************************************************************/
{
  // Make sure we have at least two performers.
  size_t num_performers = Plato::Parse::numChildren(mInterface, "Performer");
  if(num_performers < 2)
  {
    std::cout << "Not enough performers defined for a valid plato engine run." << std::endl;
    return false;
  } 

  // Loop over performers and make sure they have all of the 
  // needed components.
  std::set<int> comm_ids;
  std::set<std::string> names;
  for(pugi::xml_node node : mInterface.children("Performer"))
  {
    // Name
    std::string value_string = node.child_value("Name");
    std::string cur_name = value_string;
    if(value_string.length() == 0)
    {
      std::cout << "Found a performer without a Name attribute specified." << std::endl;
      return false;
    }
    names.insert(value_string);

    // CommID
    value_string = node.child_value("CommID");
    if(value_string.length() == 0)
    {
      std::cout << "Found a performer without a CommID attribute specified." << std::endl;
      return false;
    }
    int cur_comm_id = atoi(value_string.c_str());
    comm_ids.insert(cur_comm_id);

    // Code
    value_string = node.child_value("Code");
    if(value_string.length() == 0)
    {
      std::cout << "Found a performer (" << cur_name << ") without a Code attribute specified." << std::endl;
      return false;
    }
  }

  // Make sure all performers have a unique comm id.
  if(num_performers != comm_ids.size())
  {
    std::cout << "Performer COMM IDs are not all unique." << std::endl;
    return false;
  }

  // Make sure all performers have a unique name.
  if(num_performers != names.size())
  {
    std::cout << "Found more than one performer with the same name." << std::endl;
    return false;
  }

  return true;
}

/******************************************************************************/
bool InputXMLChecker::check_operation(const std::string &stage_name, pugi::xml_node op_node)
/******************************************************************************/
{
    if(Plato::Parse::numChildren(op_node, "Operation") > 0)
    {
        for(pugi::xml_node child_op_node : op_node.children("Operation"))
        {
            return check_operation(stage_name, child_op_node);
        }
    }
    else
    {
        std::set<std::string> op_names;
        std::string op_name = op_node.child_value("Name");
        if(op_name.length() == 0)
        {
            std::cout << "Found an operation in Stage " << stage_name <<
                    " without a Name attribute specified." << std::endl;
            return false;
        }
        op_names.insert(op_name);

        std::string perf_name = op_node.child_value("PerformerName");
        if(perf_name.length() == 0)
        {
            std::cout << "Operation " << op_name << " in Stage " << stage_name <<
                    " does not have a PerformerName attribute specified." << std::endl;
            return false;
        }
        if(!definition_exists_in_interface("Performer", perf_name))
        {
            std::cout << "Operation " << op_name << " is performed by a performer that isn't defined." << std::endl;
            return false;
        }
        // Check operation Inputs
        for(pugi::xml_node input_node : op_node.children("Input"))
        {
            std::string shared_data_name = input_node.child_value("SharedDataName");
            if(shared_data_name.length() == 0)
            {
                std::cout << "Stage " << stage_name << " has an Operation (" << op_name <<
                        ") with an Input without a SharedDataName attribute specified." << std::endl;
                return false;
            }
            if(!definition_exists_in_interface("SharedData", shared_data_name))
            {
                std::cout << "Stage " << stage_name << " has an Input with a SharedDataName that does not match an existing SharedData." << std::endl;
                return false;
            }
            std::string argument_name = input_node.child_value("ArgumentName");
            if(argument_name.length() == 0)
            {
                std::cout << "Stage " << stage_name << " has an Operation (" << op_name <<
                        ") with an Input without an ArgumentName attribute specified." << std::endl;
                return false;
            }
        }
        // Check operation Outputs
        for(pugi::xml_node output_node : op_node.children("Output"))
        {
            std::string shared_data_name = output_node.child_value("SharedDataName");
            if(shared_data_name.length() == 0)
            {
                std::cout << "Stage " << stage_name << " has an Operation (" << op_name <<
                        ") with an Output without a SharedDataName attribute specified." << std::endl;
                return false;
            }
            if(!definition_exists_in_interface("SharedData", shared_data_name))
            {
                std::cout << "Stage " << stage_name << " has an Output with a SharedDataName that does not match an existing SharedData." << std::endl;
                return false;
            }
            std::string argument_name = output_node.child_value("ArgumentName");
            if(argument_name.length() == 0)
            {
                std::cout << "Stage " << stage_name << " has an Operation (" << op_name <<
                        ") with an Output without an ArgumentName attribute specified." << std::endl;
                return false;
            }
        }
    }
    return true;
}

/******************************************************************************/
bool InputXMLChecker::definition_exists_in_interface(const std::string &type_name, const std::string &name)
/******************************************************************************/
{
    bool found = false;
    // See if SharedData exists
    for(pugi::xml_node node : mInterface.children(type_name.c_str()))
    {
        // Name
        std::string cur_name = node.child_value("Name");
        if(cur_name.compare(name) == 0)
        {
            found = true;
            break;
        }
    }
    return found;
}
/******************************************************************************/
bool InputXMLChecker::check_stages()
/******************************************************************************/
{
    std::set<std::string> names;
    size_t num_stages = Plato::Parse::numChildren(mInterface, "Stage");

    // Loop over shared datas and make sure they are correct
    for(pugi::xml_node node : mInterface.children("Stage"))
    {
        // Name
        std::string value_string = node.child_value("Name");
        std::string stage_name = value_string;
        if(value_string.length() == 0)
        {
            std::cout << "Found a Stage without a Name attribute specified." << std::endl;
            return false;
        }
        names.insert(value_string);

        // Check stage inputs
        for(pugi::xml_node input_node : node.children("Input"))
        {
            std::string shared_data_name = input_node.child_value("SharedDataName");
            if(shared_data_name.length() == 0)
            {
                std::cout << "Stage " << stage_name << " has an Input without a SharedDataName attribute specified." << std::endl;
                return false;
            }
            if(!definition_exists_in_interface("SharedData", shared_data_name))
            {
                std::cout << "Stage " << stage_name << " has an Input with a SharedDataName that does not match an existing SharedData." << std::endl;
                return false;
            }
        }
        // Check stage outputs
        for(pugi::xml_node output_node : node.children("Output"))
        {
            std::string shared_data_name = output_node.child_value("SharedDataName");
            if(shared_data_name.length() == 0)
            {
                std::cout << "Stage " << stage_name << " has an Output without a SharedDataName attribute specified." << std::endl;
                return false;
            }
            if(!definition_exists_in_interface("SharedData", shared_data_name))
            {
                std::cout << "Stage " << stage_name << " has an Output with a SharedDataName that does not match an existing SharedData." << std::endl;
                return false;
            }
        }

        size_t num_operations = Plato::Parse::numChildren(node, "Operation");
        // Make sure there is at least one operation
        if(num_operations < 1)
        {
            std::cout << "Found a Stage (" << stage_name << ") without at least one Operation attribute specified." << std::endl;
            return false;
        }

        for(pugi::xml_node op_node : node.children("Operation"))
        {
            if(!check_operation(stage_name, op_node))
            {
                return false;
            }
        }
    }

    if(names.size() != num_stages)
    {
        std::cout << "Found more than one Stage with the same name." << std::endl;
        return false;
    }

    return true;
}

/******************************************************************************/
bool InputXMLChecker::check_shared_data()
/******************************************************************************/
{
  std::set<std::string> names;
  size_t num_shared_datas = Plato::Parse::numChildren(mInterface, "SharedData");

  // Loop over shared datas and make sure they are correct
  for(pugi::xml_node node : mInterface.children("SharedData"))
  {
    // Name
    std::string value_string = node.child_value("Name");
    std::string cur_name = value_string;
    if(value_string.length() == 0)
    {
      std::cout << "Found a SharedData without a Name attribute specified." << std::endl;
      return false;
    }
    names.insert(value_string);

    // Type
    value_string = node.child_value("Type");
    if(value_string.length() == 0)
    {
      std::cout << "Found a SharedData (" << cur_name << ") without a Type attribute specified." << std::endl;
      return false;
    }

    // Layout
    value_string = node.child_value("Layout");
    if(value_string.length() == 0)
    {
      std::cout << "Found a SharedData (" << cur_name << ") without a Layout attribute specified." << std::endl;
      return false;
    }

    // OwnerName
    value_string = node.child_value("OwnerName");
    if(value_string.length() == 0)
    {
      std::cout << "Found a SharedData (" << cur_name << ") without a OwnerName attribute specified." << std::endl;
      return false;
    }
    std::string owner_name = value_string;

    // Make sure there is only one owner.
    if(Plato::Parse::numChildren(node, "OwnerName") > 1)
    {
      std::cout << "Found a SharedData (" << cur_name << ") with more than one Owner specified." << std::endl;
      return false;
    }
  
    // Make sure the owner shows up as a performer.
    if(!definition_exists_in_interface("Performer", owner_name))
    {
      std::cout << "Found a SharedData (" << cur_name << ") with an Owner (" << owner_name << ") specified that does not exist as a performer." << std::endl;
      return false;
    }

    // UserName
    value_string = node.child_value("UserName");
    if(value_string.length() == 0)
    {
      std::cout << "Found a SharedData (" << cur_name << ") without at least one UserName attribute specified." << std::endl;
      return false;
    }

    // Loop over users of this shared data and make sure there is a 
    // corresponding performer.
    for(pugi::xml_node subnode : node.children("UserName"))
    {
      std::string user_name = subnode.child_value();
      if(!definition_exists_in_interface("Performer", user_name))
      {
        std::cout << "Found a SharedData (" << cur_name << ") with a User (" << user_name << ") specified that does not exist as a performer." << std::endl;
        return false;
      }
    }
  }
  
  if(names.size() != num_shared_datas)
  {
      std::cout << "Found more than one SharedData with the same name." << std::endl;
      return false;
  }
   

  return true;
}

/******************************************************************************/
bool InputXMLChecker::check()
/******************************************************************************/
{
    std::cout << "RESULTS:" << std::endl;
    if(!check_performers())
        return false;
    if(!check_shared_data())
        return false;
    if(!check_stages())
        return false;

    std::cout << "    No problems found with the input xml files." << std::endl;
    return true;
}


