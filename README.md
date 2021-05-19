# About

Dubins is a library for control of a highly-constrained Dubins vehicle, whose state is described by the following dynamics. 

$f(x)$

The repository contains:

1. Lyapunov Control - functions for applying a Control Lyapunov Function (CLF) to our system.
2. Trajectory Optimization - functions for creating a PyDrake `MathematicalProgram` that navigates the Dubins vehicle to a desired start and end state.
3. mCLF - a custom algorithm for robust trajectory optimization, combining the concepts of a CLF with trajectory optimization to control the systme in the presence of disturbances.
4. Miscellaneous visualization tools, which were useful in writing my final report.

# Use
Explore the `demo.ipynb` Jupyter notebook for examples (and visualizations) of how this code can work!

## Miscellaneous Warnings
This code is not entirely finished. It was completed for a term project; many variables are hard-coded into my code, and the function library is not entirely clear. If you want to modify this library, please do so! I welcome any and all contributions.

# Installation Guide
First, clone this repository.
```
git clone https://github.com/deliastephens/dubins.git
```

`dubins` builds on the PyDrake libraries, which can be installed via [these instructions](http://underactuated.csail.mit.edu/drake.html). On my machine, I have to run the following in the `dubins` directory after downloading the binaries, unpacking, and setting up my PYTHONPATH. PyDrake is not compatible with Anaconda, so make sure to deactivate your conda environments before installation!


```
./install_prereqs.sh
pip3.9 install --requirement /opt/drake/share/drake/setup/requirements.txt --requirement requirements.txt
export PYTHONPATH=`pwd`:${PYTHONPATH}
```

If required, a manual `pip` install of `numpy` and `matplotlib` may be required.


To activate a Python virtualenvironment, 
```
source venv/bin/activate
```

Finally, this project is best viewed through the wonderful lens of a [Jupyter notebook](https://jupyter.org/install). To start a Jupyter notebook, simply run
```
jupyter notebook
```

If you change any of the files, you must restart the kernel.

