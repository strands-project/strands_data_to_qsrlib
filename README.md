# STRANDS Data Readers

This package aims to provide easy parsing of various datasets mainly keeping
them in QSRlib format.

It might end up being part of the QSRlib given its functionality and strong
dependency to it.

## Available readers for the following datasets
* CAD120 ([info](http://pr.cs.cornell.edu/humanactivities/data.php))

## Usage and Help
### Installation
You do need to have
[QSRlib](https://github.com/strands-project/strands_qsr_lib) somewhere installed
where it can be found by `strands_data_readers`. Easiest way is probably to 
modify your `PYTHONPATH` or if you are using an IDE then check its documentation
on how to resolve dependencies.


### CAD120
`cad120_data_reader.py` provides the class `CAD120_Data_Reader`. In most cases
it is enough to just call the constructor without any of the optional arguments,
and if you have a suitable `config.ini` then things should go smoothly.

#### About the `config.ini`
Create a `config.ini` base on the following template that tells where the CAD120
folder is. If the notion of corrected `labeling.txt` files makes no sense then
just use the same path for both `<path1>` and `<path2>`.
``` ini
[cad120_data_reader]
path = <path1>/CAD_120
corrected_labeling_path = <path2>/CAD_120
```

Just make sure that your program can find your `config.ini`. If you are not 
familiar how to do this then an easy way is to pass the directory of
`config.ini` in the constructor, e.g.:
``` python
reader = CAD120_Data_Reader(config_path=<path string to config.ini>)
```

