> The simulator is currently in early stages of development, if you find any bugs or would need extra functionality, please create an [issue](https://github.com/LOBSTER-Robotics/LobsterSimulator/issues)

The documentation can be found [here](https://docs.lobster-robotics.com/software/simulator).

###### Usage

The simulator can be installed using pip with the following command:  
`pip install git+https://github.com/LOBSTER-Robotics/LobsterSimulator`

This will install the latest version from the master branch. If you want to install from a specific branch/commit/tag use:  
```console
pip install git+https://github.com/LOBSTER-Robotics/LobsterSimulator@branch-name
 ```  
 ```console
pip install git+https://github.com/LOBSTER-Robotics/LobsterSimulator@commit-hash
 ```  
 ```console
pip install git+https://github.com/LOBSTER-Robotics/LobsterSimulator@tag
 ```  

Then to start the simulator from your Python project you can do the following:

``` Python
from lobster_simulator.Simulator import Simulator

s = Simulator(time_step=1/240)
```

###### Note
It is possible that during the installation of pybullet, you get an error that says that 'Microsoft Visual C++ 14.0 is
required' in this case you need to download the installer 
[here](https://visualstudio.microsoft.com/visual-cpp-build-tools/).
