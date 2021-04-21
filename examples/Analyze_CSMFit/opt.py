import os
import sys
import nlopt
import exodus
import PlatoServices
import PlatoPython
import ctypes
from contextlib import contextmanager
from numpy import *

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


# boilerplate that dynamically loads the mpi library required by PlatoPython.Analyze
ctypes.CDLL("libmpi.so",mode=ctypes.RTLD_GLOBAL)

# create initial mesh (required for Analyze to initialized correctly)
os.system('plato-cli geometry esp --input model.csm --output-model model_opt.csm --output-mesh model.exo --tesselation model.eto')

# create global Analyze instance
analyzeAppFileName = "analyzeApp.xml"
analyzeInputFile = "analyzeInput.xml"
analyze = PlatoPython.Analyze(analyzeInputFile, analyzeAppFileName, "analyze")
analyze.initialize();

# create global PlatoServices instance
engineAppFileName = "platoApp.xml"
engineInputFile = "platoInput.xml"
engine = PlatoServices.Services(engineInputFile, engineAppFileName, "services")
engine.initialize();

#define objective
def dfdx(x, grad):

  engine.importData("Parameters", "SCALAR", x.tolist())
  engine.compute("Update Geometry on Change")

  # import current design into Analyze instance
  analyze.importData("Parameters", "SCALAR", x.tolist())
  analyze.compute("Reinitialize on Change")

  # compute cell solutions
  analyze.compute("Compute Objective")
  value = analyze.exportData("Criterion Value", "SCALAR")

  if grad.size > 0:
    # export output data from Analyze instance
    analyzeGrad = analyze.exportData("Criterion Gradient", "SCALAR", len(grad))
    print "grad:", analyzeGrad

    for iVal in range(len(analyzeGrad)):
      grad[iVal] = analyzeGrad[iVal]

  print "eval at: ", x.tolist()
  print "eval to: ", value
  return value

xinit = [0.75, 4.8]
# create nlopt instance
opt = nlopt.opt(nlopt.LD_LBFGS, len(xinit))

opt.set_min_objective(dfdx)
#opt.add_inequality_constraint(g, 1e-6)
opt.set_lower_bounds([0.6, 4.0])
opt.set_upper_bounds([0.9, 6.0])
opt.set_xtol_rel(1e-6)
opt.set_maxeval(10)

print "xinit", xinit
xopt = opt.optimize(xinit)
minf = opt.last_optimum_value()

print "optimum is: ", minf
print "optimum at: ", xopt.tolist()

# finalize mpi and kokkos.  Not essential, but you'll get warnings otherwise.
analyze.finalize()
engine.finalize()

xtarget = [0.80, 5.0]
tol = 1e-3
error = 0.0
for i in range(len(xopt)):
  error += math.fabs(xopt[i]-xtarget[i])

if error > tol:
  print "error (", error, ") exceeded tolerance"
  sys.exit(1)
else:
  print "error (", error, ") within tolerance"
  sys.exit(0)

