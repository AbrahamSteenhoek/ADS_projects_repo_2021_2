% Nathan Neihart
% Sep. 1, 2021
%
% This script uses Impedance Corrected 2x-THRU de-embedding to generate the
% the S-parameters of fixutre A and fixture B and writes the results
% to a touchstone file. This script uses the IEEE P370 standard.

clear 
clc

% Add the path to the IEEE-P370 scripts. UPDATE AS NECESSARY.
addpath('ieee-370-master/TG1/');
addpath('ieee-370-master/TG3/');

% RELATIVE path to where the different THRU measurement are stored.
% THESE MUST BE FILLED IN BY THE STUDENT
path_R0603_THRU = 'X:\EE_514\Measured_Data\EE514_Components\0603_res\0603_RES_THRU.S2P';     % THRU structure for 1206 Resistor board
path_C0603_THRU = 'X:\EE_514\Measured_Data\EE514_Components\0603_cap\0603_CAP_THRU.S2P';     % THRU structure for 1206 Capacitor board
path_L0603_THRU = 'X:\EE_514\Measured_Data\EE514_Components\0603_ind\0603_IND_THRU.S2P';     % THRU structure for 0603 Inductor board

% Load the Touchstone files into an S-parameter OBJECT
s_2xTHRU_R = sparameters( path_R0603_THRU );
s_2xTHRU_C = sparameters( path_C0603_THRU );
s_2xTHRU_L = sparameters( path_L0603_THRU );

% Use the Matlab scripts supplied by the IEEE Standard 370-2020 to extract
% the S-parameters for fixture A and fixture B from our measured THRU
% structures. This script is locaded in ../ieee-370-master/TG1/
[s_R_fixtureA, s_R_fixtureB] = IEEEP3702xThru( s_2xTHRU_R );
[s_C_fixtureA, s_C_fixtureB] = IEEEP3702xThru( s_2xTHRU_C );
[s_L_fixtureA, s_L_fixtureB] = IEEEP3702xThru( s_2xTHRU_L );

% Write the 2-Port S-parameter objects for each Fixture into a
% corresponding touchstone file.
% Resistor fixtures
rfwrite(s_R_fixtureA, 'fixtureA_R.s2p', 'FrequencyUnit', 'Hz');
rfwrite(s_R_fixtureB, 'fixtureB_R.s2p', 'FrequencyUnit', 'Hz');

% Capacitor fixtures
rfwrite(s_C_fixtureA, 'fixtureA_C.s2p', 'FrequencyUnit', 'Hz');
rfwrite(s_C_fixtureB, 'fixtureB_C.s2p', 'FrequencyUnit', 'Hz');

% Inductor fixtures
rfwrite(s_L_fixtureA, 'fixtureA_L.s2p', 'FrequencyUnit', 'Hz');
rfwrite(s_L_fixtureB, 'fixtureB_L.s2p', 'FrequencyUnit', 'Hz');


