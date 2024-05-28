# Drone Simulator

### By Tom Shabalin, Dor Harizi and Shai Moshe

The primary objective of this project is to develop an effective solution for a small drone to navigate indoors without collisions. This project involves a fully autonomous 2D drone simulator designed to be as realistic as possible. It incorporates LIDAR sensors, a gyroscope, an optical flow sensor, and a speed sensor, with each sensor's readings including a slight amount of noise to enhance realism.

## Requirement for running
Make sure to install the next libraries using Maven:
- JGrapht library.
- Mxgraph library.

## How to run
After you installed the libraries required for the project, you can go to the SimulationWindow.java file and click the "RUN" button.

## Symbols 
- Yellow mark - mapped area.
- Black circle - his purpose to get some idea from where drone came and simply make some route that his passed.(for navigation)
- Red points - represents the wall point.
- Blue line - his whole route.

## API description
Really simple API with few buttons -
- Play/Pause button - plays and pauses the flight simulator.
- Toggle Map - allows you to hide the real map, entering to "real time" vision.
- Open Graph - opens a graph representing the actual part of the area where the drone visited.

## Screenshots from the simluator
![](./Images/Buttons.png)

![](./Images/Map1.png)

![](./Images/Map2.png)


