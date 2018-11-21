set( testFiles "5Bolt.exodiff_commands;computeCogentVolume.py;5Bolt.ref.exo;platoApp.xml;platoMain.xml;background.gen;background.jou;5bolt.cogent")
Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${PYTHON_EXECUTABLE} computeCogentVolume.py" )
set( OUTPUT_MESH "volume_grad.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
