begin service
  physics mechanical
  dimensions 3
end service

begin objective
   type maximize stiffness
   load ids 10
   boundary condition ids 11
   code lightmp
   number processors 1
   weight 1 
//   distribute objective at most 3 processors
end objective

begin boundary conditions
   fixed displacement nodeset 1 bc id 11
end boundary conditions

begin loads
    traction nodeset 2 value 0 5e4 0 load id 10
end loads
      
begin constraint 
   type volume
   code platomain
   volume fraction .5
end constraint

begin block 1
   material 1
end block

begin material 1
   material_model isotropic linear elastic 
   poissons_ratio .33
   youngs_modulus 1e9
   penalty_exponent 3
end material

begin optimization parameters
   number processors 1
   filter radius scale  2
   max iterations 3 
   output frequency 0 
    algorithm oc
   // algorithm mma 
   discretization density 
   initial density value .5
   input generator version old
end optimization parameters

begin mesh
   name input_mesh.exo
end mesh

begin paths
   code PlatoMain ../../apps/services/PlatoMain 
   code lightmp ../../apps/statics/PlatoStatics
end paths

