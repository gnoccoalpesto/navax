#!/bin/bash -i

roslaunch erc_bringup erc_bringup.launch

roslaunch ground_preproc filter.launch

roslaunch mapping3d mapping.launch

roslaunch ar_track_alvar alvar_full.launch
