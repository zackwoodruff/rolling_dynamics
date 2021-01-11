# rolling_dynamics - dynamic rolling simulations 
rolling_dynamics is a matlab library for simulating rolling dynamics between two rigid bodies. It is supplemental code to accompany the paper:
J. Z. Woodruff and K. M Lynch, "Robotic Contact Juggling" (manuscript in preparation). 

## What should it be used for? 
This code simulates rolling contacts between two rigid bodies when the surfaces can be orthogonally parameterized. 

## Installation
This code was developed and tested on MATLAB version 2018b, and there are no other dependncies. To install simply clone or download the repository. 

## Examples
The examples are contained in the folder src/examples/

### Ball on Plate
This example simulates a ball(sphere) rolling on a plate(plane) with a constant rotational velocity. This simulation has an analytical solution and we compare with results from K. Weltner, “Stable circular orbits of freely moving balls on rotating discs,” American Journal of Physics, vol. 47, no. 11, pp. 984–986, 1979.

run: src/examples/ball_on_plate/main_ball_plate_complete_example.m


### Ellipsoid on Dish
This example simulates an ellipsoid in an ellipsoidal dish. 

run: src/examples/ellipsoid_on_dish/main_ellipsoid_dish_complete_example.m
