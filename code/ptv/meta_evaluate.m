%% Introduction
% This is a script that will collect all of the evaluation 
% per simulation and spit out various aspects of PTV performance.
% 
% We are interested in how PTV performs when we change
% depth, size, concentration, sinking rate, shape and diversity.
% We will first run through a monotonic simulation. This means that 
% there will be only one variable per simulation. An image will have
% all the same sized particles, same falling rate at same depth.
% Each simulation will change on variable. 
%
% 'simulation_data' directory contains all of the simulation data.
% There are 180 frames (3minutes worth) of data per setup.
%

% Configure PTV tracker %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Monotonic 
% Introduction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% The goal of monotonic simulation is to find out the limit of our PTV
% algorithm. When do we start to see a decline in our data?
%
% The format of each monotonic directory is as follows:
m_data_{size}_{concentration}_{velocity}_{depth}_{sh ape}
%   size (um): diameter or major axis of the particle
%   concentration (No./m^3): particle concentration per cubic meter
%   velocity (m/day): falling rate
%   depth (mm): z-axis depth from left camera
%   shape: 's' for spheres and 'c' for cylinders
% 

% Implementation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 1. parse available simulation directories

% 3. loop through each of them to run evaluation
%   a) Run the main tracking algorithm 
%   b) save the result
%   c) run evaluation for comparison

% 4. Go through each parameter to output a graph of interest
%   a) size
%   b)concentration
%   c) velocity
%   d) depth
%   e) shape

%% Diverse
% Now, we are interested in seeing how the system performs against more
% 'realistic' situations. Over the literature, there is a wide variance
% per particle parameters that are not really easy to come up with 
% an accurate simulated data. Hence, we will provide boundaries for each
% paramters and randomly mix them. We will compare the ground truth data
% in each simulation scenario to PTV results. 
% 
% The format of diverse simulation directoy is as follows:
% d_data_{index}
%   index: index of each configuration. The configuration file is 
%          saved as a simulation_config.mat in each simulation directory

% Implementation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 1. parse available simulation directories

% 2. loop through and evaluate each simulation 

% 3. Dissect the evaluation result in terms of axis discussed in the 
%    introduction.

% Include the graphs that compare outcome of each simulation to see if 
% there has been any significantly good / poor scenarios

%% Note for later 
% May be we want to find out which configuration works the best