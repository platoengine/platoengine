from ESPtools import *

def mesh(modelName, meshName=None, minScale=0.2, meshLengthFactor=1.0, etoName=None ):

  if meshName == None:
    dot = '.'
    tokens = modelName.split(dot)
    tokens.pop()
    meshName = dot.join(tokens) + ".exo"

  paramVals = getInitialValues(modelName)

  with redirected('csm.console'):
    updateModel(modelName, paramVals)

  with redirected('aflr.console'):
    aflr(modelName, meshName, minScale, meshLengthFactor, etoName)

  with redirected('toExo.console'):
    toExo(modelName, meshName)

