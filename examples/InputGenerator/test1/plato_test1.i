begin objective
   type maximize stiffness
   begin bcs
      fixed displacement nodeset 2 
   end bcs
   begin loads
      traction sideset 1 direction 0 1 0 scale 100
   end loads
   code salinas
   number processors 4
   weight .34
end objective

begin objective
   type maximize heat conduction
   begin bcs
      fixed temperature nodeset 2 
   end bcs
   begin loads
      heat flux sideset 1 100
   end loads
   code albany
   number processors 4
   weight .34
end objective
   
begin objective
   type match frf data
   begin bcs
      fixed displacement nodeset 2 
   end bcs
   begin loads
      force nodeset 1 direction 0 0 1 scale 100
   end loads
   code salinas
   number processors 4
   weight .32
end objective
      
begin constraint
   type volume
   weight 1
   volume fraction .5
end constraint

begin block
   id 1
   material 1
end block

begin material
   id 1
   penalty exponent 3
   poissons ratio .3
   youngs modulus 1e9
   thermal conductivity 1e5
end material

begin optimization parameters
   filter scale 1.5
   max iterations 20
   number processors 2
   output frequency 2
end optimization parameters

begin mesh
   name plato_test1.gen
end mesh

begin paths
   code albany /projects/plato/dev/albany/opt/albany/Latest/gcc-5.4.0-openmpi-1.10.2/bin
   code salinas /projects/plato/dev/plato_sd/gcc-5.4.0
   code PlatoMain /projects/plato/dev/PlatoMain/gcc-5.4.0-opt
   code lightmp /projects/plato/dev/lightmp/gcc-5.4.0-opt
end paths

   