# ADS-B signal generation in Python
`dronekit` only works with Python 3.9 due to some compatibility issues.

## Install Python 3.9 virtual environment with pipenv
Use python virtual environment to avoid messing up with your local python packages. 

```bash
# If you do not have python and pip on your machine 
sudo apt-get install python-pip python3-dev

# Install pipenv using pip
pip install --user pipenv

# Add source and install python3.9
sudo add-apt-repository ppa:deadsnakes/ppa 
sudo apt update 
sudo apt install python3.9 python3.9-distutils

# Set up python 3.9 environment and install dependencies
cd simulations/sim_adsb/python
pipenv --rm
pipenv --python 3.9
pipenv shell
pipenv install
```

## Run simulation
```bash
python sim_adsb.py
```


