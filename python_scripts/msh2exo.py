#!/usr/bin/env python
import os, sys

tPath = str(sys.argv[1])
if tPath[-1:] == "/":
  tmhs2osh_root = tPath + "src/msh2osh "
  tosh2exo_root = tPath + "src/osh2exo "
else:
  tmhs2osh_root = tPath + "/src/msh2osh "
  tosh2exo_root = tPath + "/src/osh2exo "

tmhs2osh_command = tmhs2osh_root + str(sys.argv[2]) + ".msh " + str(sys.argv[2]) + ".osh" 
tSuccess = os.system(tmhs2osh_command)
print('msh2osh command was executed successfuly:', tSuccess)

tosh2exo_command = tosh2exo_root + str(sys.argv[2]) + ".osh " + str(sys.argv[2]) + ".exo"
tSuccess = os.system(tosh2exo_command)
print('osh2exo command executed successfuly:', tSuccess)
