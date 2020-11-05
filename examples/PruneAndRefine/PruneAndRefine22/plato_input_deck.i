begin service 1
    code plato_analyze
    physics mechanical
    dimensions 3
    minimum_ersatz_material_value 1e-3
    tolerance 5e-8
end service

begin output
    service 1
    output_data_to_file true
end output

begin objective
   type maximize stiffness
   load ids 1
   boundary condition ids 1
   code plato_analyze
   number processors 1
   minimum ersatz material value 1e-3
end objective
   
begin boundary conditions
   fixed displacement nodeset name ns_1 bc id 1
end boundary conditions

begin loads
    traction sideset name ss_1 value 0 -3e4 0 load id 1
end loads

begin constraint
   code PlatoMain
   type volume
   volume absolute 20
end constraint

begin block 1
   material 1
end block

begin material 1
   material_model isotropic linear elastic
   poissons_ratio .3
   youngs_modulus 1e8
end material

begin optimization parameters
   number processors 1
   filter radius absolute .6
   max iterations 5
   output frequency 5
   algorithm oc
   discretization density
   initial density value .5

   // The default field name for restarting with
   // an initial guess is "optimizationdofs". However,
   // Plato Analyze uses the word "control" for its field
   // name so we have to specify it explicitly here.
   initial guess field name control

   // When running from the GUI the name of the mesh file
   // containing the initial guess for the densities
   // is automatically generated to be "restart_<iteration#.exo".
   // Since we aren't running from the GUI we will explicitly
   // specify the filename.
   initial guess filename restart_data.exo

   // Number of extra mesh element layers to keep
   // around the actual initial guess for the design.
   // This important so that the design can still 
   // adjust without hitting the boundaries of the 
   // pruned mesh.
   number buffer layers 2

   // Specifies whether to prune the mesh or not.
   prune mesh true

   // Specifies the number of global refinements to do.
   number refines 1

   // Non-zero value indicates we are restarting from
   // an initial guess. This is typically for when you
   // are running from the GUI but at the moment
   // probably needs to be in here anyway. If you do 
   // not specify an "initial guess filename" this
   // iteration number will be used to automatically build
   // a file name based on the iteration you want to 
   // restart from.
   restart iteration 100

end optimization parameters

begin mesh
   name pa_comp_min1.exo
end mesh
   
