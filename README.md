# NAAIMS

NAAIMS is a modular automated intersection management (AIM) simulator. This package aims to support different priority policies, tiling strategies, intersection shapes, counts, lane changing, and stochastic vehicle movements while maintaining otherwise consistent behavior to allow for direct comparisons between simulator setups.

This (hopefully) readable and extensible open source implementation is intended to encourage collaborative, rapid development of new ideas in autonomous intersection management.

## Getting started

Run NAAIMS by selecting one of the scenarios in `scenarios/`, which will build an intersection configuration, load it, and start the simulator.

## Roadmap

Development goals ordered loosely by priority

1. Simulate a single intersection using
   1. square tiles and the FCFS priority policy (complete)
   2. traffic signal phases
   3. a newly developed priority auction policy
   4. using a modified Carlino 2013 auction
   5. a new reservation system that allows for stochastic reservations to account for imprecise (human-controlled) trajectories
   6. conflict arc tiles
2. Scale up simulation size
   1. Simulate a small cluster of intersections (i.e., implement connected intersections and associated lane-changing behavior)
   2. Simulate a large network of intersections (i.e., city-scale)
3. Develop an extensible API to hook into other open-source traffic simulators like SUMO

## Style

This project follows PEP8 style using the following linters and formatters:

- pylint
- mypy
- pycodestyle
- autopep8
