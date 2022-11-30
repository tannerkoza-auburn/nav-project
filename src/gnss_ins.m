%% GNSS/INS Integration - FORDAV Data

clear
clc
close all 

%% CONFIGURABLE PARAMS

logName = "2017-08-04-V3-Log3";

%% LOAD DATA

fprintf('nav-project: loading and calculating data...\n')

rootDir = fullfile(fileparts(which(mfilename)), "..");
dataDir = fullfile(rootDir, "data", logName);
filePath = fullfile(dataDir, logName);

data = extractData(filePath);

%% PLOT RESULTS

fprintf('nav-project: plotting results...\n')

plotResults(data)


