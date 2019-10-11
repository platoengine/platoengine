# PLATO Engine

<p align="center"><img src="https://github.com/platoengine/platoengine/blob/master/figures/Plato_Logo.jpeg" width="400"/></p>

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
* **stable** - Most recent stable version of PLATO Engine. This may not have all of the current features but will be more tested. Checkout the stable branch if you want to use PLATO Engine to run some examples. A `stable version` is denoted by a tag. For instance, the latest release version is `v0.3.0`.
* **master** - Constantly changing and may contain bugs. Checkout the master branch if you are actively developing.

# Application
The PLATO Engine testbed is designed to support research in the area of synthesis optimization on high-performance distributed memory computer architectures. The PLATO Engine testbed is being used to explore interoperability with multiple analysis, modeling and optimization numerical libraries on high-performance distributed memory computer architectures to synthesized designs. The testbed is also being used to test the viability of these analysis, modeling and optimization numerical libraries for the solution of synthesis optimization problems. 

# Approach
PLATO Engine is intended to serve as a collaborative testbed that enables the demonstration of other modeling, analysis and optimization numerical libraries for optimization-based design. It is designed to enable intercommunication of modeling, analysis and synthesis data using a Multiple Program, Multiple Data (MPMD) parallel programming model. This MPMD model allows multiple programs/executables to run independently while communicating with one another. The synthesis optimization algorithm orchestrates the execution and communication between the various analysis codes and aggregates their contributions to generate a design that meets multiple criteria.  

# High Performance Computing
PLATO Engine has been designed for the MPMD parallel programming model. It also aims to perform using a Single Program, Multiple Data (SPMD) parallel programming model. PLATO Engine is designed for MPI distributed memory calculations and aims to perform its function efficiently on current and next generation computing architectures.

# Required Libraries
Trilinos library (provides Epetra, Seacas and STK): https://github.com/trilinos/trilinos

# Hardware Requirements
Tested compilers are `g++ 4.7.2`, `g++ 5.4.0`, `g++ 7.2.0` and `intel 17.0.1` compilers. Tested `OS` include `Linux` and `Mac`. `RAM` requirements are problem size dependent. 

**Note: Currently, PLATO Analyze only runs on Graphics Processing Units (GPUs)**

# Contributing 
Please open a GitHub issue to ask a question, report a bug, request features, etc. If you'd like to contribute, please fork the repository and use a feature branch. Make sure to follow the team's [coding style policies](https://github.com/platoengine/platoengine/wiki/Coding-Style). Pull requests are welcome.

# User Support

Users are welcomed to submit questions via email to plato3D-help@sandia.gov.
