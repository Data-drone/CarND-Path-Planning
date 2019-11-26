# CarND Path Planning Project

Udacity Self Driving Car

Project Repo for Planning Project

This code runs a path planning algorithm to drive the car in the udacity self driving car simulator

# Outline

The starting logic for this is built out from the official walk through
https://www.youtube.com/watch?v=7sI3VHFPP0w&feature=youtu.be


# Behaviour Planning

In this simulator there are only three lanes. The goal of the Path Planner is to 
smoothly and safely drive through the three lane highway simulator.

Smoothly and safely entails:
- drive at least 4.32 miles without incident
- stay within speed limit
- Max Accel and Jerk are not Exceeded
- No Collisions
- Stay within lane when not transitioning
- Able to change lanes

## Lane Change Decisions

In this scenarios there are three decisions that the car has to make:
- Stay in Lane
- Change Left
- Change Right

Depending on which lane we are currently, different options will be avaiable.
the function `laneOptions` takes the current lane then returns the options available for making lane change decisions.

## Decision Criteria

If the car is stuck behind a slow car and there is an adjacent lane clear then the car should switch lane. For the middle lane, we will have to create some way to decide whether to go left or right.

We decide on whether a lane is clear or not depending on whether there are any cars within 30 units of our car. See line `179` if it is not clear then the lane is taken off of the avaliable `lane_options`

When changing lanes, we simply take the first value from lane options vector. This is suboptimal, we should be looking at which lane is faster or better aligned with our goal when deciding where to go from the middle lane rather than just defaulting to the left but we have no goal lane in this scenario and a cost function is harder to tune. 

# Notes:

Starter code from https://github.com/udacity/CarND-Path-Planning-Project.git