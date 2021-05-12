# Installation Guide

Following [this](http://underactuated.csail.mit.edu/drake.html) resource.

`TODO: Add detail`

After downloading binaries, unpacking and setting up PYTHONPATH, run the following in the `dubins` directory:

```
./install_prereqs.sh
pip3.9 install --requirement /opt/drake/share/drake/setup/requirements.txt --requirement requirements.txt
export PYTHONPATH=`pwd`:${PYTHONPATH}
```

Also need to install `numpy` and `matplotlib`