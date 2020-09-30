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
    traction sideset name ss_2 value 0 -3e3 0 load id 1
end loads
      
begin constraint 
   code PlatoMain
   type volume
   volume fraction .25
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
   filter radius scale 4.48
   max iterations 10 
   output frequency 1000 
   algorithm oc
   discretization density 
   initial density value .5
end optimization parameters

begin mesh
   name bolted_bracket.exo
end mesh

begin paths
code PlatoMain /ascldap/users/rvierte/plato/src/platoengine/build/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths
begin paths
code PlatoMain /ascldap/users/rvierte/plato/src/platoengine/build/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths
