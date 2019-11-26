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

## Decision Criteria

If the car is stuck behind a slow car and there is an adjacent lane clear then the car should switch lane. For the middle lane, we will have to create some way to decide whether to go left or right.

We should look at:
- 

# Notes:

Starter code from https://github.com/udacity/CarND-Path-Planning-Project.git