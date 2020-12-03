begin service 1
  code platomain
  number processors 1
  number ranks 1
end service

begin service 2
  code plato_analyze
  number processors 1
  number ranks 1
end service

begin service 3
  code plato_analyze
  number processors 1
  number ranks 1
end service

begin criterion 1
  type compliance
  minimum_ersatz_material_value 1e-3
end criterion

begin criterion 2
  type volume
end criterion

begin scenario 1
  physics mechanical
  dimensions 3
  loads 1
  boundary_conditions 1
  material 1
  minimum_ersatz_material_value 1e-3
  tolerance 5e-8
end scenario

begin scenario 2
  physics mechanical
  dimensions 3
  loads 2
  boundary_conditions 1
  material 1
  minimum_ersatz_material_value 1e-3
  tolerance 5e-8
end scenario

begin objective
  type weighted_sum
  criteria 1 1
  services 2 3
  scenarios 1 2
  weights 1 1
end objective

begin output
    service 2
   output_data true
   data dispx dispy dispz
end output

begin boundary conditions
   fixed displacement nodeset name ns_1 bc id 1
end boundary conditions

begin loads
    traction sideset name ss_2 value 0 -3e3 0 load id 1
    traction sideset name ss_2 value 0 0 3e3 load id 2
end loads
      
begin constraint
  criterion 2
  relative_target 0.25
  type less_than
  service 1
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
   filter radius scale 4.48
   max iterations 10 
//   output frequency 1000 
   algorithm oc
   discretization density 
   initial density value .5
end optimization parameters

begin mesh
   name bolted_bracket.exo
end mesh

begin paths
code PlatoMain /ascldap/users/bwclark/spack2/platoengine/RELEASE/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths