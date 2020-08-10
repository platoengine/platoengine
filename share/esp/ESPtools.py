import os
import sys
import subprocess
from shutil import copyfile
from pyCAPS import capsProblem
from contextlib import contextmanager

##############################################################################
## define functionality that redirects console output to a file
##############################################################################
@contextmanager
def redirected(to=os.devnull):
    '''
    import os

    with redirected(to=filename):
        print("from Python")
        os.system("echo non-Python applications are also supported")
    '''
    fdout = sys.stdout.fileno()
    fderr = sys.stderr.fileno()

    def _redirect_stdout(to):
        sys.stdout.close()                  # + implicit flush()
        os.dup2(to.fileno(), fdout)         # fd writes to 'to' file
        sys.stdout = os.fdopen(fdout, 'w')  # Python writes to fd

    def _redirect_stderr(to):
        sys.stderr.close()                  # + implicit flush()
        os.dup2(to.fileno(), fderr)         # fd writes to 'to' file
        sys.stderr = os.fdopen(fderr, 'w')  # Python writes to fd

    with os.fdopen(os.dup(fdout), 'w') as old_stdout, os.fdopen(os.dup(fderr), 'w') as old_stderr:
        with open(to, 'w') as file:
            _redirect_stdout(to=file)
            _redirect_stderr(to=file)
        try:
            yield # allow code to be run with the redirected stdout
        finally:
            _redirect_stdout(to=old_stdout) # restore stdout.
                                            # buffering and flags such as
                                            # CLOEXEC may be different
            _redirect_stderr(to=old_stderr) # restore stderr.
                                            # buffering and flags such as
                                            # CLOEXEC may be different


##############################################################################
## define function that compute tesselation data and body mesh
##############################################################################
def aflr(modelName, meshName, minScale=0.2, maxScale=1.0, meshLengthFactor=1.0, etoName=None):


  tokens = meshName.split('.')
  tokens.pop()
  meshBaseName = ".".join(tokens)

  # Initialize capsProblem object
  myProblem = capsProblem()

  # Load EGADS file
  filename = os.path.join(".", modelName)
  model = myProblem.loadCAPS(filename)

  # Load AFLR4 AIM
  aflr4 = myProblem.loadAIM(aim         = "aflr4AIM",
                            altName     = "aflr4",
                            analysisDir = ".")

  if etoName != None:
    # if tesselation object exists, remove it
    subprocess.call(['rm', etoName])

    tokens = etoName.split('.')
    tokens.pop()
    etoBaseName = ".".join(tokens)

    # Dump VTK files for visualization
    aflr4.setAnalysisVal("Proj_Name", etoBaseName)
    aflr4.setAnalysisVal("Mesh_Format", "ETO")

  # Scaling factor to compute AFLR4 'ref_len' parameter
  aflr4.setAnalysisVal("Mesh_Length_Factor", meshLengthFactor)
  aflr4.setAnalysisVal("min_scale", minScale)
  aflr4.setAnalysisVal("max_scale", maxScale)

  # Run AIM pre-/post-analysis
  aflr4.preAnalysis()
  aflr4.postAnalysis()

  # Load AFLR3 AIM to generate the volume mesh
  aflr3 = myProblem.loadAIM(aim         = "aflr3AIM",
                            analysisDir = ".",
                            parents     = aflr4.aimName)

  # Dump VTK files for visualization
  aflr3.setAnalysisVal("Proj_Name", meshBaseName)
  aflr3.setAnalysisVal("Mesh_Format", "SU2")

  # Run AIM pre-/post-analysis
  aflr3.preAnalysis()
  aflr3.postAnalysis()

  # Close CAPS
  myProblem.closeCAPS()


##############################################################################
## define function that converts su2 mesh to exo mesh
##############################################################################
def getInitialValues(modelName):

  strVal = subprocess.check_output(['awk', '/despmtr/{print $0}', modelName])
  params = strVal.split('\n')
  params = list(filter(None, params)) ## filter out empty strings

  initialValues = []
  for param in params:
    tokens = param.split(' ')
    tokens = list(filter(None, tokens)) ## filter out empty strings

    if len(tokens) != 9 and len(tokens) != 3:
      print "Expected line either in the form:"
      print "despmtr <name> <value> lbound <value> ubound <value> initial <value>"
      print "or in the form:"
      print "despmtr <name> <value>"
      print "got: " + param
      raise Exception("Parsing error: reading initial values failed.")

    ## first token should be 'despmtr'
    if tokens[0] != 'despmtr':
      raise Exception("unknown error: expected 'despmtr' token, got '" + tokens[0] + "'" )

    if len(tokens) == 9:
      ## eighth token should be 'initial'
      if tokens[7] != 'initial':
        raise Exception("parsing error: expected 'initial' token, got '" + tokens[7] + "'" )

      ## get current value
      initialValue = float(tokens[8])

    elif len(tokens) == 3:

      ## get current value
      initialValue = float(tokens[2])

    initialValues.append(initialValue)
  
  return initialValues


##############################################################################
## define function that converts su2 mesh to exo mesh
##############################################################################
def toExo(modelName, meshName):
  tokens = meshName.split('.')
  tokens.pop()
  meshBaseName = ".".join(tokens)

  strVal = subprocess.check_output(['awk', '/named_face/{print $2, $3, $4, $5}', modelName])
  tokens = strVal.split('\n')
  tokens = list(filter(None, tokens)) ## filter out empty strings

  callArgs = ['Su2ToExodus', meshBaseName+'.su2', meshBaseName+'.exo']

  for token in tokens:
    callArgs += token.split(' ')

  subprocess.call(callArgs)

##############################################################################
## define function that converts su2 mesh to exo mesh
##############################################################################
def updateModel(modelName, paramVals):

  response = subprocess.check_output(['awk', '/attribute/{if ($2~/capsMeshLength/) print $3}', modelName])
  capsMeshLength = str(response)

  modedName = modelName + ".tmp"

  f_in = open(modelName)
  f_out = open(modedName, 'w')

  for line in f_in:
    if line.strip().lower() == 'end':
      f_out.write("select body\n")
      f_out.write("attribute capsAIM $aflr4AIM;aflr3AIM\n")
      f_out.write("attribute capsGroup $solid_group\n")
      f_out.write("attribute capsMeshLength " + capsMeshLength + "\n")

      f_out.write("select face\n")
      f_out.write("attribute capsGroup $solid_group\n")
      f_out.write("attribute capsMeshLength " + capsMeshLength + "\n")

    f_out.write(line)

  f_out.close()


  for ip in range(len(paramVals)):
    p = paramVals[ip]
    f = open('tmp.file', "w")
    command = 'BEGIN{ip=0};{if($1~"despmtr"){if(ip=='+str(ip)+'){print $1, $2, val, $4, $5, $6, $7, $8, $9}else{print $0}ip++}else{print $0}}'
    print "command: ", command
    subprocess.call(['awk', '-v', 'val='+str(paramVals[ip]), command, modedName], stdout=f)
    f.close()
    subprocess.call(['mv', 'tmp.file', modedName])

  subprocess.call(['mv', modedName, modelName])


##############################################################################
## define function that generates exodus mesh from csm file
##############################################################################
def mesh(modelName, meshName=None, minScale=0.2, maxScale=1.0, meshLengthFactor=1.0, etoName=None ):

  if meshName == None:
    dot = '.'
    tokens = modelName.split(dot)
    tokens.pop()
    meshName = dot.join(tokens) + ".exo"

  if type(minScale) == str:
    minScale = float(minScale)

  if type(maxScale) == str:
    maxScale = float(maxScale)

  if type(meshLengthFactor) == str:
    meshLengthFactor = float(meshLengthFactor)

  paramVals = getInitialValues(modelName)

  with redirected('csm.console'):
    updateModel(modelName, paramVals)

  with redirected('aflr.console'):
    aflr(modelName, meshName, minScale, maxScale, meshLengthFactor, etoName)

  with redirected('toExo.console'):
    toExo(modelName, meshName)

