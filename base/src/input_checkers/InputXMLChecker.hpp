/*
 * InputXMLChecker.hpp
 *
 *  Created on: Aug 4, 2017
 *
 */

#ifndef SRC_INTERFACEXMLCHECKER_HPP_
#define SRC_INTERFACEXMLCHECKER_HPP_

#include <vector>
#include "Plato_Parser.hpp"

class InputXMLChecker {

public:
  InputXMLChecker(const std::string &interface_xml_filename,
                  const std::string &plato_main_operations_filename,
                  const std::vector<std::string> &physics_app_files);
  ~InputXMLChecker();
  bool check();

private:
  pugi::xml_document mInterface;
  pugi::xml_document mPlatoOperations;
  std::vector<pugi::xml_document*> mPhysicsAppOperations;
  bool check_performers();
  bool check_shared_data();
  bool check_stages();
  bool check_operation(const std::string &stage_name, pugi::xml_node op_node);
  bool definition_exists_in_interface(const std::string &type_name, const std::string &name);

};


#endif /* SRC_INTERFACEXMLCHECKER_HPP_ */
