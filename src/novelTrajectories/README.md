# STRANDS data tools that work with QSRlib


### Trajectory reader
`traj_data_reader.py` provides the class `Trajectory_Data_Reader` and  `Trajectory_Data_Keeper`. 
`config.ini` needs to include qsr options.

#### About the `config.ini`
Create a `config.ini` based on the following template:

``` ini
[trajectory_data_reader]
path = <path1>
; use load_from_files=True in the constructor to load from the following files
date = date
qsr = qtcb
q = 0_01
v = False
n = True
```

You can add any parameters you like in the config file, but also initiate them in your data_reader.

Just make sure that your program can find your `config.ini`. If you are not 
familiar how to do this then an easy way is to pass the directory of
`config.ini` in the constructor, e.g.:
``` python
reader = CAD120_Data_Reader(config_path=<path string to config.ini>)
```

