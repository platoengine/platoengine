#!/usr/bin/env python
import os, sys

tPathInput = str(sys.argv[1])
if tPathInput[-1:] == "/":
  tmhs2osh_root = tPathInput + "msh2osh "
  tosh2exo_root = tPathInput + "osh2exo "
else:
  tmhs2osh_root = tPathInput + "/msh2osh "
  tosh2exo_root = tPathInput + "/osh2exo "

tNameInput = str(sys.argv[2])
if tNameInput[-4:] == ".msh":
  tName = tNameInput[:-4]
else:
  tName = tNameInput

tmhs2osh_command = tmhs2osh_root + tName + ".msh " + tName + ".osh" 
tSuccess = os.system(tmhs2osh_command)
print('msh2osh command was successful:', tSuccess)

tosh2exo_command = tosh2exo_root + tName + ".osh " + tName + ".exo"
tSuccess = os.system(tosh2exo_command)
print('osh2exo command was successful:', tSuccess)
