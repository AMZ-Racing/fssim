# Configure multiple repetitions
An example of the configuration file can be found under [automated_simulation](../fssim/config/automated_simulation.yaml). This follows exactly the same style as [simulation](../fssim/config/simulation.yaml). You can add another repetition with one more line under `repetitions` with different or same configuration. A new repetition es executed either when `kill_after` runns out or the car violated any conditions under `res/checks` (e.g. vehicle exits track).

# Launch automated test
Automated tests must be launch with a [launch.py](../fssim/scripts/launch.py) which requires two files. A configuration file (e.g. [automated_simulation](../fssim/config/automated_simulation.yaml)) and an output directory (e.g. `sim_output/`)
```
usage: launch.py [-h] [--config FILE] [--output OUTPUT]
```
And example launching code can look like this if called from within a workspace.
```
./src/fssim/fssim/scripts/launch.py --config src/fssim/fssim/config/automated_simulation.yaml --output ~/sim_output
```
