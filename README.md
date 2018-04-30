# ME149: Optimal Control of Robotics Final Report ReadMe
## By Shane Rozen-Levy

## Objective

The goal of this project was to produce optimal trajectories for a RC car. We designed trajectories for the car turning around
and for the car parallel parking. 

## Code

### Top Level Functions

My code has two top level functions located in the main directory. MAIN_simpleCarParallelPark.m is the entry point for the car parallel
parking. Main_simpleCarUTurn.m is the entry point for the car making a u-turn.

### General Code Outline

The main functions setup a parameterized version of the problems. This allows for the easy changing of parameters like the road dimensions
and the if time is a decision variables. They then call simpleCarSubProblem.m located in the sub routine directory. simpleCarSubProblem
sets up the problem struct for dirColBvpTrap.m using the parameters passed to it. dirColBvpTrap.m then impliments the trapezoidal
collocation and passes the probelm to fmincon to solve.

### Dependencies

This project was written in MATLAB 2017a. All of the dependencies for the main functions are either located in the CodeLibrary directory or in sub routines directory. Both main
functions will automatically add the directories to your path assuming you do not move them around. There are no other dependencies. I
did not use the course CodeLibrary. If I needed a function from the course CodeLibrary I copied it to the CodeLibrary I used for this 
project.

### Code I Wrote/Modified

Most of the code for this project I wrote. The spline functions were taken from the course library or online and the animate.m function was
taken from the code library. I wrote the dynamics functions and the code to draw the car. I modified dirColBvpTrap.m from what I turned
in for homework to handle time as a decision variable and path constraints.
decision variable 

