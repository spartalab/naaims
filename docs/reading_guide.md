# NAAIMS Reading Guide

The NAAIMS core algorithm is divided into initialization and the step cycle. Initialization runs once on starting the simulation, and the step cycle continues to repeat until the simulation timeframe ends.

I recommend you start by reading the [premise and assumptions](##premise-and-assumptions), continuing linearly until you find an odd term, for which you can refer to the [glossary](##glossary) for a definition (hopefully). I'd really appreciate it if you make fixes as you read and submit them as pull requests when you're done!

The reading client is up to you. If you want a recommendation, I developed this in Visual Studio Code with the Python extension and several formatters and linters installed, which might be helpful if you want to jump around function definitions using the context menu or make style-compliant fixes to any issues you come across.

## Table of contents

1. [Glossary](##glossary)
2. [Premise and assumptions](##premise-and-assumptions)
3. [Directory structure](##directory-structure)
4. [Initialization](##initialization)
5. [Step cycle](##step-cycle)

## Glossary

| Road | One-way connections between intersections, spawners, and removers. May contain multiple parallel lanes. Need not be straight. |
| Lane | A lane can be thought of as a trajectory that can contain some number of vehicles. Both roads and intersections consist of lanes. RoadLane and IntersectionLane implementations differ, but both call back to a general Lane class for shared functionality. |
| Pathfinder | A module that finds the movements necessary for a vehicle to reach its destination. |
| IntersectionManager | The module that manages when and how vehicles can enter its specific intersection. Different managers implement different priority policies. |
| LaneChangeManager | The module that manages when and how vehicles can change lanes. Different managers implement different lane changing strategies. |
| Tiling | How an intersection is divided into discrete units that can be checked for conflicts. |
| Tile | A specific area in an intersection identified by its (x,y) area and time t. They can be reserved by vehicles traveling through an intersection. |
| Facility | An object that vehicles physically travel on, i.e., roads and intersections. |
| Upstream | Upstream objects---vehicle spawners, roads, and intersections---need to transfer vehicles to their downstream objects. |
| Downstream | Downstream objects---roads, intersections, and vehicle removers---need to accept vehicle transfers from Upstream objects. | In this AIM implementation, vehicles are mostly data types, with their movement logic controlled by the Facility they're traveling on. |

## Premise and assumptions

(Not all of them, just a few relevant ones.)

Vehicles are assumed to be rectangles with fixed length and width. When updating their position on a lane, they're split into front, center, and rear sections and tracked separately. This allows a vehicle to straddle the boundary between a road and an intersection and vice versa. Furthermore, instead of having their movement decision logic inside the Vehicle class, it's processed by the lane they're on.

On the topic of lanes, instead of using a vehicle's true position directly, lanes track their vehicles' proportional progress along the lane (AKA longitudinal movement) and use that to do all calculations. This has the advantage of reducing updates to one dimension which should make calculations simpler. If 2D movement is necessary, lanes also track a vehicle's lateral deviation away from the lane's centerline and mix that with a vehicle's progress to find its true position.

To make stopping distance guarantees, vehicles are assumed to have a braking speed of at least some minimum value specified at simulation initialization.

## Directory structure

This library is structured like a pip package, with all source files (other than `main.py`) contained in the eponymous `naaims/` directory.

Scripts to run new simulation instances will live in `scenarios/`. Documentation is, of course, in `docs/`.

Inside `naaims/`, modules are grouped into folders by the general class of object they're associated with, e.g. IntersectionManagers in `intersections/`. Some folders only contain a single module, since I anticipate their scope might grow enough to be split into multiple files as time goes on, so this will make backwards compatibility easier.

Unit tests go into `test`.

Most module names should be self explanatory and will be discussed in detail later in this reading guide, except for these: - `archtypes.py` contain interfaces that spawners, removers, intersections, and roads implement - `shared.py` is a module that holds instanced data that's shared across all classes - `util.py` contains miscellaneous utility functions

## Initialization

Start by looking in `naaims/simulator.py`, specifically at the init function for the base Simulator class. Initialization of a new naaims instance is contained to this init and the methods it calls. As such, this function's signature is as general as possible, with the ability to create as many roads, intersections, spawners, and removers as specification dicts it's provided.

The specification dicts are JSON-like packages that contain the input arguments necessary for the initialization of objects and their properties. For example, an intersection's spec dict will contain its connectivity matrix, the road objects it's connected to, and a spec dict that it will use to create its manager.

I think the init is well-commented enough that it can be read without much additional explanation. Make sure to jump into the defintions of any objects it creates and functions it calls, too!

You can ignore the inits of Simulator subclasses; I don't anticipate touching those until we're ready to run real experiments. I think of Simulator subclasses as simply convenience functions that take input parameters and convert into road, intersection, spawner, and remover specifications to provide to the base Simulator's init.

Spawners, intersections, and removers can only be connected to roads (e.g., an intersection can't dump vehicles directly into a remover), so roads have additional checks to see if they're connected to a spawner or an intersection upstream, and an intersection or a remover downstream.

## Step cycle

This cycle is defined in `Simulator.step()` in `naaims/simulator.py`. I recommend reading it linearly, jumping in and out of function definitions as needed. It consists of the following sub-steps:

1. [Update speeds](#update-speeds)
2. [Step vehicles](#step-vehicles)
3. [Process vehicle transfers](#process-vehicle-transfers)
4. [Handle logic](#handle-logic)
5. [Increment time](#increment-time)

Each run through the step cycle is a single timestep. I except there to be at least 12, maybe 30 or 60 steps per second.

### Update speeds

First, lanes contained in facilities, i.e., roads and intersections, calculate the new speed and acceleration of all vehicles in its jurisdiction and return them to the Simulator for updating. (Because a vehicle's speed update is determinant on the speed of the vehicle preceding them, the update can't happen in place.)

The abstract method `Facility.get_new_speeds()` is implemented by Road and Intersection classes. Both call `Lane.get_new_speeds()` which uses logic to determine first their new acceleration and then their new speed. The NAAIMS vehicle model assumes that vehicles have complete control over their acceleration each timestep, and uses that acceleration to calculate its new speed.

### Step vehicles

Next, upstream objects---spawners, roads, and intersections---spawn or update their internal record of vehicles in their domain and the vehicles' record of their true position for vehicles in their jurisdiction.

The abstract method `Facility.step_vehicles()` is implemented by VehicleSpawner, Road, and Intersection classes.

Vehicle spawners decide whether to spawn vehicles based on a poisson process and if the road it's going to spawn into has room. If it does decide to spawn, it prepares VehicleTransfer packets for all three sections of the vehicle. (The spawner has no way to calculate vehicle positions itself, so it has to hand the vehicle off to the road in order to fully instantiate the vehicle.)

Both Road and Intersection call `Lane.step_vehicles()` which updates their own record vehicles' proportional progress along the lane, lateral deviation, and then the vehicles' true positions if its center section is still within lane. If a vehicle section reaches or goes past the end of a lane, the lane prepares a VehicleTransfer packet for that section.

Following the spawning or position updates, all upstream objects place any VehicleTransfers they've created into the buffer of their Downstream object(s)---roads, intersections, and removers---for processing in the next substep.

### Process vehicle transfers

Downstream objects process incoming vehicle sections in their buffer using the `Downstream.process_buffer()` interface.

Roads and intersections create a new record of the newly transferred section's progress, and update the vehicle's position if the section transferring is the vehicle center. In a way, this is just `step_vehicles()` part two.

Vehicle removers let the section pass into the ether unless the exiting section is the center of the vehicle. Since a remover has no way to update a vehicle's true position, we let the exiting center section signify that the vehicle has exited the simulation, so the vehicle is returned to the simulator for logging.

### Handle logic

In this substep, facilities use the `Facility.update_schedule()` interface to handle all their logic that doesn't involve updating vehicles' positions, speed, or acceleration.

Roads will call on their lane change managers to issue permissions to the vehicles that want to and can switch lanes. This functionality is half-baked right now since I'm focusing on the single intersection case, while lane changes are only necessary for multi-intersection simulations.

Intersections call on their managers to update the state of their tiling. This removes the tiles from the last timestep, updates permitted lanes if traffic signals are used, and, if implemented, tweaks reserved tiles based on what movements the intersection saw last timestep. Once this is complete the manager decides whether to poll incoming road lanes for new reservation requests, in what order to poll them, and which eligible requests to accept. How this is done depends on how the manager subclass implements the abstract method `IntersectionManager.process_requests()`.

To check if a reservation is possible, the manager uses `Tiling.check_reservation()` in order to verify if the lane's next eligible reservation or reservations are possible. This function is especially notable because it's the single longest and most complicated method in the entire library (so far). In order to check if a reservation will work, we have to do a nested simulation, moving the vehicles requesting the reservation forward in virtual space to check which tiles they occupy each future timestep until they exit. In effect, we're recreating the above substeps of updating vehicle speeds, then positions, then transferring vehicles from road to intersection to road with slightly different logic.

The test starts just as the front of the first vehicle requesting is projected to enter the intersection and ends when the last vehicle requesting that still has a potentially possible reservation exits the intersection. On that last point, since trailing vehicles' speeds are determined by those of their predecessors, if we're approving a sequence of vehicles, we have to continue simulating leading vehicles in a sequence on the downstream road even after they've exited until the last vehicle in the sequence exits the intersection (or has its request rejected).

If the check passes, the manager can do some more logic with the viable reservation (such as comparing several potential reservations to find the best combo) before confirming them using `Tiling.confirm_reservation()`. The manager then does some cleaning up before finishing this substep.

### Increment time

They cycle finishes by incrementing the global timestep record `SHARED.t` before returning to the beginning of the cycle for the next timestep.
