## Explanation of our code:
The idea is that when an object that is red (which is associated with stop in traffic) is detected within a distance of   0.15 meters, the duckiebot should start a stop sequence and will have stopped before reaching the object (to simulate the idea of realising the need to stop and having the space for braking which is crucial in traffic) 

We also added a PID controller to be able to reduce the deviation from the path that our duckiebot was experiencing

## Installation:
steps:
1-clone demo branch of the repository 'Croissant' to your local machine
```bash 
git clone --branch demo --single-branch https://github.com/Gulin534/Croissant.git
```

2-navigate to the cloned directory
```bash
cd Croissant
```

## How to set up a proper environment to be able to run the code with no problems
!!!make sure there is nothing RED around!!! (because having any red objects in the vicinity of the robot will intrefere with being able to test the code as the main purpose is for it to stop if a red object is detected.)
Please ensure that the track that will be used for the duckiebot is smooth to be able to observe the PID working.

## How to use it:
steps:
1- Build the code: 
```bash
dts devel build -f
```

2- Run PID control, to make sure that your wheel encoders are reset and aligned:
```bash
dts devel run -R (name_of_robot) -L encoder-pid 

```
3- Run the twist_mux_launcher which will run the 4 nodes needed for the code to work:
```bash
 dts devel run -R (name_of_robot) -L twist_mux_launcher -X
 ```
