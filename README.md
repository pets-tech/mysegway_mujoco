## Segway in mujoco

<!-- ![Image](docs/segway.png) -->
<!-- ![Image](docs/segway_side_view.png) -->
![Image](docs/segway_standup.gif)



## Install

```bash
python3 -m venv ./venv
source venv/bin/activate
pip install -r requirements.txt

sudo apt-get install libzmq3-dev
g++ controllers/lqr_controller_stab.cpp -o lqr_controller_stab -lzmq -O2

```

## Run

To compute K vector (modal control and lqr)
```bash
python model.py
```

To simulate with mujoco
```bash
python simulate.py & python rt-plotter.py
```

![Image](docs/work.png)


## Acknolagments

Thanx to [Antonov Evgeniy](https://github.com/mrclient) for model and modal control.


