# Mars Rover Simulator

A simulation of a Mars Rover based on a [4WIS4WID mobile robot model](https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/joe.2014.0241).
The rotatable distance sensor of the Mars Rover determines the distance to obstacles in the environment.
The control software is implemented in Ada and either steers the Rover autonomously or manually.
The Mars Rover and its environment is visualized in 3D using the game engine Bevy.

## Setup

- Ensure that the following dependencies are installed (only tested on Linux):
    - GNAT Pro for Rust 25 / Rust 1.77.2
    - GNAT Pro 25 (or newer)
    - Alire (configured to use GNAT Pro toolchain)
    - OS dependencies of Bevy ([Linux dependencies](https://github.com/bevyengine/bevy/blob/release-0.13.2/docs/linux_dependencies.md), see [Setup](https://bevyengine.org/learn/quick-start/getting-started/setup/) of the Quick Start Guide for other OS)
- Ensure that submodules are checked out with `git submodule update --init --recursive`
- Execute `cargo run --release` to start the demo

## Visualization

### Mouse Control

- Drag and drop of rover
- Drag and drop of obstacles

### Keyboard Control

- `R`: Reset rover position
- `T`: Toggle display of rover status text

The rover can be controlled using the keyboard.
The control software will take control if none of the following keys are pressed for a few seconds:

- `Up`/`Down`: Set velocity (overwrites controller)
- `Left`/`Right`: Set steering angle (overwrites controller)
- `Home`/`End`: Change distance sensor angle (overwrites controller)

## Design

The core business logic is encapsulated in the [`domain`](src/domain.rs) module.
It defines the `Robot` and `Environment` entities, along with the rules governing their interactions.
The `Simulator` drives the simulation by updating the `Robot`'s state based on these rules.
External control is managed by the `Controller` and the simulation is visualized by the `Visualizer`.

### Robot ([`domain::robot`](src/domain/robot.rs))

- Stores the robot's position and heading as well as the state of the wheels (steering angle, rotating angle, velocity) and the distance sensor (angle, measured distance).
- Allows updating the position based on the state of the wheels and the elapsed time.
- Allows setting the steering angle and velocity of the wheels and the angle of the distance sensor.

### Environment ([`domain::environment`](src/domain/environment.rs))

- Stores the obstacles with their position and dimensions (horizontal and vertical length).
- Allows determining the distance to the next obstacle based on a position and direction.

### Simulator ([`simulator`](src/simulator.rs))

- Requests the robot to calculate its new position based on the elapsed time.
- Updates the position of the robot as long as a position update does not lead to a collision with an obstacle.
- Requests the environment to determine the distance of the distance sensor to the next obstacle in the sensor direction and updates the measured distance in the robot.

### Controller ([`controller`](src/controller.rs))

- Runs the Ada-based control software in a separate thread.
- Receives the steering angle and velocity of the robot wheels and the angle of the robot's distance sensor from the control software.
- Provides the measured distance from the robot's distance sensor to the control software.

### Visualizer ([`visualizer`](src/visualizer.rs))

- Visualizes the robot and the environment with its obstacles.
- Initializes the robot and the environment.
- Updates the robot position if the robot is dragged and dropped.
- Updates the obstacle position if the obstacle is dragged and dropped.
