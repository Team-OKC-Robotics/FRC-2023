
# system identification

## Introduction

This is where we find our physics models of the robot based on experimental data.

## Setup

Prerequisites: Python 3 must be installed

1. Install `pipenv` using `pip`
```
pip install pipenv
```
2. Initialize a virtual environment
```
pipenv update
```

## How to Run

System ID can be done by running
```
pipenv run python3 main.py [module]
```
where `[module]` is the module you want to run. You can also individually test the modules by going to the appropriate subdirectory and running:
```
pipenv run python3 module_id.py
```
where `module_id.py` is the name of the module's system ID script.

## Modules

Currently we have system ID tools for the following modules and their module argument for `main.py`:
- Swerve (module: `swerve`)


