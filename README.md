# PLATO Engine

<p align="center"><img src="https://github.com/platoengine/platoengine/blob/master/figures/plato_logo_simple.png" width="400"/></p>

# Description
The PLATO Engine computer program serves as a collaborative testbed rich in light-weight synthesis tools for optimization-based design. PLATO Engine is a research code designed to facilitate collaboration with academia, labs and industries by providing interfaces for plug-n-play insertion of synthesis technologies in the areas of modeling, analysis and optimization. Currently, PLATO Engine offers a set of light-weight tools for finite element analysis, linear- and nonlinear-programming, and non-gradient based optimization. The PLATO Engine program is designed to run on high-performance computers.

# Getting Started

## Installation
To checkout a copy of PLATO Engine and PLATO Analyze from the command line use from the directory of your choice (i.e. `$HOME/codes`):
```javascript
git clone https://username:password@github.com/platoengine/platoengine.git (PLATO Engine)
git clone https://username:password@github.com/SNLComputation/lgrtk.git (PLATO Analyze)
```
where username and password corresponds to your github.com account. At this stage, make sure all required environment variables are set and all tpls are installed. 

For build instructions, please refer to the [PLATO Wiki](https://github.com/platoengine/platoengine/wiki/Build-Instructions)

### Branches
There are two main branches of this repository; stable and master. 
* **stable** - Most recent stable version of PLATO Engine. This may not have all of the current features but will be more tested. Checkout the stable branch if you want to use PLATO Engine to run some examples. A `stable version` is denoted by a tag. For instance, the latest release version is `v0.6.0`.
* **master** - Constantly changing and may contain bugs. Checkout the master branch if you are actively developing in PLATO.

# Application
The PLATO Engine testbed is designed to support research in the area of optimization-based design on high-performance computing systems.  The PLATO Engine testbed is used to explore interoperability with several analysis, modeling and optimization tools.  The testbed is also used to test the viability of these analysis, modeling and optimization tools for the solution of optimization-based design problems. 

# Approach
PLATO Engine is intended to serve as a collaborative testbed.  It is designed to enable intercommunication of modeling, analysis and optimization data using a Multiple Program, Multiple Data (MPMD) parallel programming model.  The MPMD model allows multiple, independent programs/executables to share data in-memory.  The optimization algorithms orchestrate the execution and communication between multiple analysis codes and aggregates their contributions to create designs that meet multiple performance criteria.  

# High Performance Computing
PLATO Engine has been designed for MPMD parallel executions.  It also targets Single Program, Multiple Data (SPMD) parallel programming model; however, the SPMD model is not heavily used by the targeted PLATO applications.  PLATO Engine is also performance-protable, which allow it to optimally perform in current and next-generation computing architectures.

# Required Libraries
Trilinos library (provides Epetra, Seacas and STK): https://github.com/trilinos/trilinos

Omega_h library (provides mesh metadata): https://github.com/SNLComputation/omega_h

Netcdf library (provides I/O libraries): https://www.unidata.ucar.edu/software/netcdf/

AMGX library (provides GPU linear solver): https://github.com/NVIDIA/AMGX

Lapack library (provides linear algebra libraries): http://www.netlib.org/lapack/

Boost library (provides C++ source libraries): https://www.boost.org/

# Hardware Requirements
Tested compilers are `g++ 4.7.2`, `g++ 5.4.0`, and `g++ 7.2.0` compilers. Tested `OS` include `Linux` and `Mac`. `RAM` requirements are problem size dependent. 

**Note: Currently, PLATO Analyze only runs on Graphics Processing Units (GPUs)**

# Contributing 
Please open a GitHub issue to ask a question, report a bug, request features, etc.  If you'd like to contribute, please fork the repository and use a feature branch.  Make sure to follow the team's [coding style policies](https://github.com/platoengine/platoengine/wiki/Coding-Style).  Pull requests are welcomed.

# User Support

Users are welcomed to submit questions via email to plato3D-help@sandia.gov.
