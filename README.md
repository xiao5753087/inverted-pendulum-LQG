# Cart-Pole LQG Control Simulation

A MATLAB implementation of LQG (Linear Quadratic Gaussian) control for the classic cart-pole (inverted pendulum) system.

## Overview

This project demonstrates the full LQG design pipeline:
- Linearization of the nonlinear cart-pole system at the upright equilibrium
- LQR controller design via solving the Discrete Algebraic Riccati Equation (DARE)
- Kalman Filter design for state estimation under process and measurement noise
- Closed-loop simulation with Euler integration, including an impulse disturbance at t = T/2

## Demo

https://youtu.be/yc4iB4t7oXw

## System Model

State vector: `x = [p, ṗ, θ, θ̇]`  
- `p`: cart position (m)  
- `θ`: pole angle from vertical (rad)  
- `u`: force applied to cart (N)

## Dependencies

- MATLAB R2021a or later
- Control System Toolbox (`lqr`, `care`)

## Usage

Run `LQG_practice.m` directly in MATLAB. The simulation will animate the cart-pole response and save `cart_pole_LQG.avi` to the working directory.

## Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| M | 1.0 kg | Cart mass |
| m | 0.1 kg | Pole mass |
| l | 0.5 m | Pole length |
| Q | diag([1,1,10,1]) | LQR state cost |
| R | 0.1 | LQR control cost |

## Author

Yuxiao [你的姓] — Electrical Engineering, UNSW Sydney
