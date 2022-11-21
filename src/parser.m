%% CSV Parser

clear
clc
close all

%% Parsing

filePath = '../data/gps.csv';
gpsTable = readtable(filePath);