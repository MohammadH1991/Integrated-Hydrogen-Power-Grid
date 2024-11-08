This repository contains code and models for an integrated hydrogen-electricity power grid. The model optimizes the operation of a 4-bus electrical grid and a 4-node hydrogen network with the primary objective of minimizing operational costs. The optimization problem is formulated as a Mixed-Integer Linear Programming (MILP) problem and solved using the GLPK solver.

Project Description
The project models an integrated energy system consisting of:

4-Bus Electrical Grid: Electricity is generated using hydrogen turbines and dispatched to meet demand at each bus.
4-Node Hydrogen Network: The hydrogen network supplies hydrogen fuel to turbines and manages flow using steady-state Weymouth equations.
Hydrogen Turbine-Generated Electricity: Hydrogen turbines produce electricity to supply the power grid.
DC Power Flow for Electricity: The DC power flow model is used to represent electricity flows in the grid.
Steady-State Weymouth Model for Hydrogen Flow: Hydrogen flows within the network are modeled using the steady-state Weymouth model.
The integrated model helps achieve optimal operation by minimizing fuel costs, and grid operational costs.

Problem Formulation
The optimization problem is modeled as a Mixed-Integer Linear Program (MILP) with the following characteristics:

Objective: Minimize the total operational cost of the integrated hydrogen-electricity grid.
Constraints:
Power balance and DC power flow constraints for the electrical grid.
Hydrogen flow constraints using the steady-state Weymouth model.
Operational limits for both hydrogen and electricity networks.
Generation and transmission limits for hydrogen turbines.

