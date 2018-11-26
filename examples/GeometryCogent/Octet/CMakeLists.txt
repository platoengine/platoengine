get_filename_component(testDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set( testFiles "octet_hex.xml;octet_tet.xml" )
Plato_add_test_files( "${testFiles}" )

set( COMMAND1 "${COGENT_BINARY} --input=octet_hex.xml --output=model_hex.vtk" )
set( COMMAND1 "${COGENT_BINARY} --input=octet_tet.xml --output=model_tet.vtk" )
set( COMMANDS "${COMMAND1};${COMMAND2}" )

add_test(
  NAME ${testDirName}
  COMMAND ${CMAKE_COMMAND}
  -DCOMMANDS=${COMMANDS}
  -P ${CMAKE_SOURCE_DIR}/base/config/run.cmake
)
