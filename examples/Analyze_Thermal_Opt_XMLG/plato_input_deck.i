begin objective
   type maximize heat conduction 
   load ids 1 
   boundary condition ids 1 2  
   code plato_analyze
   number processors 1
   output for plotting dispx dispy vonmises
end objective

begin boundary conditions
   fixed temperature nodeset 1 bc id 1
   fixed temperature nodeset 2 bc id 2
end boundary conditions

begin loads
    heat flux sideset 1 value -1e2 load id 1
end loads
      
begin constraint 
   type volume
   volume fraction .2
end constraint

begin block 1
   material 1
end block
begin block 2
   material 1
end block

begin material 1
   thermal conductivity coefficient 210
   density 2703
   specific heat 900 
end material

begin optimization parameters
   number processors 1
   filter radius scale 2.48
   max iterations 30 
   output frequency 1000 
   algorithm ksal
   discretization density 
   initial density value .2
   fixed blocks 2
   al penalty parameter 1
   al penalty scale factor 1.05
end optimization parameters

begin mesh
   name tm2.exo
end mesh


