# Hector and Turtlebot3

Implementation of path planning and control of a Turtlebot3, as well as extended kalman filter and finite state machine on a hector.

![demo](demo.gif)

## Features (Turtlebot)
1. Theta\* planner
2. Quintic Hermite trajectory planner
3. BFS goal replan to avoid inaccessible goals
4. Bidirectional motion

## Features (Hector)
1. Extended Kalman Filter
- Inertia Measurement Unit (IMU)
- Global Positioning System (GPS)
- Barometer
- Sonar
- Magnetometer
2. Finite State Machine
- Movement to Turtlebot3's current location
- Movement to Turtlebot3's final goal
- Movement to Hector's initial position

## Instructions

1. Build by calling make.sh
```
./make.sh
```
2. Run the bringup.sh and run.sh scripts on the remote device on different terminal
```
./bringup.sh
```
```
./run.sh
```

## Changing parameters

- Change turtle goals and turtle and hector initial positions in src/ee4308\_bringup/worlds/world20.sh
- Change other turtle parameters in src/ee4308\_turtle/config/turtle.yaml
- Change other hector parameters in src/ee4308\_turtle/config/hector.yaml
