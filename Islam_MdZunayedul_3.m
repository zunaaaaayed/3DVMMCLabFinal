clear all; close all; clc;
warning('off', 'vision:calibrate:boardShouldBeAsymmetric');

addpath(genpath('./lib'));
res_dir = 'results/';
if ~exist(res_dir, 'dir')
    mkdir(res_dir);
end