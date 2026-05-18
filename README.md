# CS 81 Final Project

## Problem Statement:
Robot A and B are exploring an unknown environment to find a goal location. The initial position of both these robots is arbitrary and their relation to each other is unknown. The robots must map the environment and find the goal location given a visual cue. Having multiple robots at different initial positions makes the search for the goal faster. The robots will return the optimal path from the goal to the closest (evaluated based on path length) starting point of one of the robots.

## Motivating Application: 
Robots could be exploring an unsafe, human-inaccessible environment (e.g. a structurally unsound building), trying to find a target (this could be a bomb or an injured person or whatever is making the building unsafe like a gas leak). The robots are put into the environment at different places to maximize efficiency of mapping. There is no time to rigorously define the robots injection locations. 
