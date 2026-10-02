# Self-Balancing Robot Simulator

A robot that stands on two wheels and balances itself upright — like a mini Segway.
download the requirments in the anaconda terminal for the pybullet for smooth download.
## What Does It Do?

The robot tries to stay standing upright on its own. When you push it or tilt it, the robot automatically corrects itself and returns to standing straight.

**How to test it:**
- Run the simulation
- In the PyBullet window, move the slidebar to increase the velocity positive or negative to move forward or backward.


## Setup

1. Install the required packages:
```bash
pip install -r requirements.txt
```
(First time takes 5-15 minutes because PyBullet needs to download and compile)

2. Run the simulation:
```bash
self_balance.py
```

A window will open showing the robot standing still on a ground plane.

## How to Use It

**While the window is open:**
-move the slidebar to move the robot forward or backward. the slidebar is in the right side of the screen.

**When you close the window:**
- The simulation saves data to `balance_log.csv`
- A message appears telling you to run `plot_results.py`

## See the Results

After running the simulation, create a graph showing how well it balanced:

```bash
python plot_results.py
```

This opens a graph with two panels:
- **Top:** Shows the robot's tilt angle over time (should stay near 0 = upright)
- **Bottom:** Shows how hard the controller was working at each moment

A file called `balance_response.png` is also saved.





## Project Files

- `self_balance.py` — The actual simulation (run this one)
- `pid_controller.py` — The balancing logic
- `plot_resultsself.py` — Creates graphs from the data
- `balance_log.csv` — Data saved after you run main.py
- `balance_response.png` — Graph created by plot_results.py

## Typical Workflow

```bash
# Install packages (one time)
pip install -r requirements.txt

# Run the simulation
self_balance.py
# → Drag the robot in the window to test it
# → Close the window when done

# Make a graph of the results
python plot_resultsself.py
# → Opens a graph showing how well it balanced
```

## What If The Robot Falls Over?

If the robot tips over and doesn't balance, the simulation stops. This means the controller settings need to be adjusted.

The controller settings are at the top of `main.py`:
```python
PID_KP = 8.0
PID_KI = 0.5
PID_KD = 0.5
```


