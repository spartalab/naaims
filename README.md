# aimsim

aimsim is a modular automated intersection management (AIM) simulator. This aims to simulate several AIM schemas, varying from priority policies, tiling methods, and stochasticity. aimsim is intended to be an easily readable and readily extensible implementation that can be used to test the efficacy of new AIM control schemas and examine how well they scale.

## Roadmap

Development goals we want to hit ordered by priority

1. Simulate a single intersection using ArcAIM and the FCFS priority policy
2. Simulate a single intersection using tile-based AIM
3. Simulate alternate priority control policies
    1. Simulate a single intersection using AIM with priority auctions
    2. Simulate mixed traffic signals and AIM
4. Implement a new reservation system that allows for stochastic reservations to account for imprecise (human-controlled) trajectories
5. Scale up simulation size
    1. Simulate a small cluster of intersections (i.e., implement connected intersections and associated lane-changing behavior)
    2. Simulate a large network of intersections (i.e., city-scale)
6. Develop an extensible API to hook into other open-source traffic simulators like SUMO

## Style

This project follows PEP8 style using the following linters and formatters:

* pylint
* mypy
* pycodestyle
* autopep8
