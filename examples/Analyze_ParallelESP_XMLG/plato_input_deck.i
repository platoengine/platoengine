begin objective
   type maximize stiffness
   load ids 1 2
   boundary condition ids 1 
   code plato_analyze
   number processors 1
   output for plotting dispx dispy dispz 
end objective

begin boundary conditions
   fixed displacement nodeset name ns_4 bc id 1
end boundary conditions

begin loads
    traction sideset name ss_2 value 0 2e3 0 load id 1
    traction sideset name ss_3 value 0 1e3 0 load id 2
end loads
      
begin constraint
   type volume
   volume absolute 17.5
end constraint

begin material 1
   poissons ratio .33
   youngs modulus 1e9
end material

begin block 1
   material 1
   element type tet4
end block
   
begin optimization parameters
   algorithm mma
   number processors 1
   filter radius scale 1.5
   max iterations 2
   output frequency 1
   optimization type shape
   prune mesh false
   number buffer layers 2
   number refines 0
   write restart file false
   csm file rocker.csm
end optimization parameters

begin paths
   code plato_analyze analyze_MPMD
   code PlatoMain PlatoMain
end paths

begin mesh
   name rocker.exo
end mesh

