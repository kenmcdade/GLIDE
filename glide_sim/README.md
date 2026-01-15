# GLIDE Simulation Core

This directory contains the core GLIDE simulation framework.

The code here implements internal physics models, numerical integration,
and visualization utilities used to explore tether-assisted orbital
momentum exchange concepts.

## Entry Point

The main runnable script is:

    python main.py

Running this executes a representative simulation using the internal
modules defined in this directory.

## Structure

- physics/ — orbital mechanics, tether dynamics, and force models  
- viz/ — plotting and animation utilities  
- main.py — example driver script tying the components together  

This framework is exploratory and intended for concept validation and
analysis, not flight-ready modeling.
