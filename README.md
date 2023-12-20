# ACAV

## Introduction

ACAV is an automated Python framework designed to conduct causality analysis for AV accident recordings. 

Our paper, "ACAV: A Framework for Automatic Causality Analysis in Autonomous Vehicle Accident Recordings" will be published at ICSE 2024 in April.

Please visit [ACAV website](https://acav2023.github.io/) for more information and demonstration.



## Prerequisites

- python3
- protobuf 3.19.4
- [cyber_record](https://github.com/acav2023/cyber_record#cyber_record)
- [record_msg](https://github.com/acav2023/cyber_record#2-parse-messages)
- shapely

### Install `protobuf` for ACAV
Install `protobuf 3.19.4` for ACAV, by the following command: 
```shell
pip3 install protobuf==3.19.4
```

### Install `cyber_record` for ACAV
Install `cyber_record`, a cyber record file offline parse tool, by the following command: 
```shell
pip3 install cyber_record
```

### Install `record_msg` for ACAV
To avoid introducing too many dependencies, save messages by `record_msg`.
```shell
pip3 install record_msg -U
```

### Install `shapely` for ACAV
Install `shapely` for ACAV, by the following command: 
```shell
pip3 install shapely
```



## Step by Step

### Run ACAV
Run ACAV by the following command: 
```shell
cd /root_of_ACAV_SourceCode
python3 main.py -i <the directory of the original recording file>
```
For example: 
```shell
cd /root_of_ACAV_SourceCode
python3 main.py -i record/T-2.record
```

### Setup

#### Adding a Vehicle
Currently, only the parameter file for [Lincoln 2017 MKZ](https://github.com/ApolloAuto/apollo/tree/master/modules/calibration/data/Lincoln2017MKZ_LGSVL) are included in ACAV. 
For a parameter file of a new vehicle, please add it to `/root_of_ACAV-SourceCode/vehicles/`. 

#### Adding a map
Currently, only the map file for San Francisco is included in ACAV. 
For your own map file, please add it to `/root_of_ACAV-SourceCode/maps/`



## Citing

If you use the project in your work, please consider citing the following work: 
```
```
