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
def toExo(meshName, groupAttrs):
  tokens = meshName.split('.')
  tokens.pop()
  meshBaseName = ".".join(tokens)

  callArgs = ['Su2ToExodus', meshBaseName+'.su2', meshBaseName+'.exo']

  # aflr writes faces w/o a capsGroup attribute to MARK 1
  callArgs.append("mark")
  callArgs.append("1")
  callArgs.append("sideset")
  callArgs.append("surface")

  for entry in groupAttrs:
    if entry["name"] != "solid_group":
      callArgs.append("mark")
      callArgs.append(entry["index"])
      callArgs.append("sideset")
      callArgs.append(entry["name"])

  subprocess.call(callArgs)

##############################################################################
## define function that converts su2 mesh to exo mesh
##############################################################################
def updateModel(modelName, paramVals):


  #
  # Add requisite body and face attributes for meshing
  #

  # find mesh size attribute 'MeshLength'
  #
  response = subprocess.check_output(['awk', '/set/{if ($2=="MeshLength") print $3}', modelName])

  ## is 'MeshLength' in the csm file?
  if response == "":
    raise Exception("Error reading CSM file: required variable, 'MeshLength', not found..")

  ## is 'MeshLength' in the csm file only once?
  tokens = response.rstrip().split("\n")
  if len(tokens) > 1:
    raise Exception("Error reading CSM file: multiple 'MeshLength' keywords found. 'MeshLength' variable should appear once.")

  MeshLength = str(response)

  modedName = modelName + ".tmp"

  f_in = open(modelName)
  f_out = open(modedName, 'w')

  for line in f_in:
    if line.strip().lower() == 'end':
      f_out.write("select body\n")
      f_out.write("attribute capsAIM $aflr4AIM;aflr3AIM\n")
      f_out.write("attribute capsGroup $solid_group\n")
      f_out.write("attribute capsMeshLength " + MeshLength + "\n")

      f_out.write("select face\n")
      f_out.write("attribute capsGroup $solid_group\n")
      f_out.write("attribute capsMeshLength " + MeshLength + "\n")

    f_out.write(line)

  f_out.close()


  #
  # If paramVals were provided, set them in the model file
  #

  for ip in range(len(paramVals)):
    p = paramVals[ip]
    print "param: " + str(p)
    f = open('tmp.file', "w")
    command = 'BEGIN{ip=0};{if($1~"despmtr"){if(ip=='+str(ip)+'){print $1, $2, val, $4, $5, $6, $7, $8, $9}else{print $0}ip++}else{print $0}}'
    print "command: ", command
    subprocess.call(['awk', '-v', 'val='+str(paramVals[ip]), command, modedName], stdout=f)
    f.close()
    subprocess.call(['mv', 'tmp.file', modedName])

  subprocess.call(['mv', modedName, modelName])

  #
  # find any face attribute assignments and copy them to the end of the file
  #

  f_in = open(modelName)
  f_out = open(modedName, 'w')

  faceAttrs = []

  bodyLine = ""
  faceLine = ""
  for line in f_in:
    if line.strip().lower() == 'end':
      for faceAttr in faceAttrs:
        for attrLine in faceAttr:
          f_out.write(attrLine)
    else:
      tokens = line.split(' ')
      tokens = list(filter(None, tokens)) ## filter out empty strings
      if bodyLine != "" and faceLine != "":
        if tokens[0] == "attribute" and tokens[1] == "capsGroup":
          faceAttrs.append([bodyLine, faceLine, line])
          bodyLine = ""
          faceLine = ""
      elif bodyLine != "":
        if tokens[0] == "select" and tokens[1] == "face":
          faceLine = line
        else:
          bodyLine = ""
      else:
        if tokens[0] == "select" and tokens[1] == "body":
          bodyLine = line

    f_out.write(line)

  f_out.close()

  subprocess.call(['mv', modedName, modelName])

##############################################################################
## define function that generates exodus mesh from csm file
##############################################################################
def mesh(modelNameIn, modelNameOut=None, meshName=None, minScale=0.2, maxScale=1.0, meshLengthFactor=1.0, etoName=None, parameters=None ):

  deleteOnExit = False
  if modelNameOut == None:
    modelNameOut = "work" + modelNameIn
    deleteOnExit = True

  if meshName == None:
    dot = '.'
    tokens = modelNameIn.split(dot)
    tokens.pop()
    meshName = dot.join(tokens) + ".exo"

  if type(minScale) == str:
    minScale = float(minScale)

  if type(maxScale) == str:
    maxScale = float(maxScale)

  if type(meshLengthFactor) == str:
    meshLengthFactor = float(meshLengthFactor)

  paramVals = []
  if type(parameters) == str:
    paramVals = [float(entry) for entry in parameters.split(',')]
  else:
    paramVals = getInitialValues(modelNameIn)

  subprocess.call(['cp', modelNameIn, modelNameOut])

  with redirected('csm.console'):
    updateModel(modelNameOut, paramVals)

  with redirected('aflr.console'):
    aflr(modelNameOut, meshName, minScale, maxScale, meshLengthFactor, etoName)

  ## get capsGroup map
  groupAttrs = []
  f_in = open('aflr.console')
  for line in f_in:
    tokens = line.split(' ')
    tokens = list(filter(None, tokens)) ## filter out empty strings
    if tokens[0] == "Mapping" and tokens[1] == "capsGroup" and tokens[2] == "attributes":
      numberLine = f_in.next()
      tokens = numberLine.split(' = ')
      tokens = list(filter(None, tokens)) ## filter out empty strings
      if tokens[0].strip() == "Number of unique capsGroup attributes":
        numLines = int(tokens[1].strip())
        for iEntry in range(numLines):
          nextLine = f_in.next()
          defs = nextLine.split(', ')
          defs = list(filter(None, defs)) ## filter out empty strings
          groupName = defs[0].split(' = ')[1].strip()
          groupIndex = defs[1].split(' = ')[1].strip()
          groupAttrs.append({"name": groupName, "index": groupIndex})
      

  with redirected('toExo.console'):
    toExo(meshName, groupAttrs)

  if deleteOnExit:
    subprocess.call(['rm', modelNameOut])
  
