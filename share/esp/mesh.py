import ESPtools
import sys

# get raw arguments from command line
inArgs = sys.argv[1:]

# unpack raw arguments into dictionary
args = {}
for inArg in inArgs:
  tokens = inArg.split('=')
  key = tokens[0]
  val = tokens[1]
  args[key] = val

# check for required input
if not args.keys().count('modelNameIn'):
  raise Exception("Required input is missing:  expected modelNameIn=model.csm")

# call mesh() with destructured dictionary
ESPtools.mesh(**args)
