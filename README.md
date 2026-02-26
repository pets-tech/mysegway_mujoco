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

```

## Run

To compute gains (K vector) (modal control and/or lqr)
```bash
python model.py
```

To simulate with mujoco (with system state vizualizetion)
```bash
python simulator.py & python rt-plotter.py
```

Examples of controllers:

1. LQR with standup

    - C++

    ```
    g++ controllers/lqr_controller_with_standup.cpp -o controllers/lqr_controller_with_standup -lzmq -O3
    ./controllers/lqr_controller_with_standup
    ```

    - Python

    ```
    python controllers/lqr_controller_with_standup.py
    ```

![Image](docs/work.png)


## Acknolagments

Thanx to [Antonov Evgeniy](https://github.com/mrclient) for model and modal control.


