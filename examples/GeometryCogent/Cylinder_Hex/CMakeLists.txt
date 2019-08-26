set( testFiles "Cylinder_Hex.exodiff_commands;computeCogentVolume.py;Cylinder_Hex.ref.exo;platoApp.xml;platoMain.xml;background.gen;background.jou;cyl.cogent")
Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${Python2_EXECUTABLE} computeCogentVolume.py" )
set( OUTPUT_MESH "volume_grad.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
