import os
import sys
import subprocess
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
def aflr(modelName, meshName, etoName):

  tokens = etoName.split('.')
  tokens.pop()
  etoBaseName = ".".join(tokens)

  tokens = meshName.split('.')
  tokens.pop()
  meshBaseName = ".".join(tokens)

  # if tesselation object exists, remove it
  subprocess.call(['rm', etoName])

  # Initialize capsProblem object
  myProblem = capsProblem()

  # Load EGADS file
  filename = os.path.join(".", modelName)
  model = myProblem.loadCAPS(filename)

  # Load AFLR4 AIM
  aflr4 = myProblem.loadAIM(aim         = "aflr4AIM",
                            altName     = "aflr4",
                            analysisDir = ".")

  # Dump VTK files for visualization
  aflr4.setAnalysisVal("Proj_Name", etoBaseName)
  aflr4.setAnalysisVal("Mesh_Format", "ETO")

  # Scaling factor to compute AFLR4 'ref_len' parameter
  aflr4.setAnalysisVal("Mesh_Length_Factor", 1.0)
  aflr4.setAnalysisVal("min_scale", 0.22)

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

  for ip in range(len(paramVals)):
    p = paramVals[ip]
    f = open('tmp.file', "w")
    command = 'BEGIN{ip=0};{if($1~"despmtr"){if(ip=='+str(ip)+'){print $1, $2, val, $4, $5, $6, $7}else{print $0}ip++}else{print $0}}'
    print "command: ", command
    subprocess.call(['awk', '-v', 'val='+str(paramVals[ip]), command, modelName], stdout=f)
    f.close()
    subprocess.call(['mv', 'tmp.file', modelName])


##############################################################################
## script body
##############################################################################

modelName = sys.argv[1]
meshName  = sys.argv[2]
etoName   = sys.argv[3]

paramVals = sys.argv[4:]

print "paramVals: ", paramVals

with redirected('csm.console'):
  updateModel(modelName, paramVals)

with redirected('aflr.console'):
  aflr(modelName, meshName, etoName)

with redirected('toExo.console'):
  toExo(modelName, meshName)

