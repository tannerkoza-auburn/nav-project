%% CSV Parser

clear
clc
close all

%% Parsing

files = dir('../data/*.csv');

for i=1:length(files)
    
    currentFilePath = strcat(files(i).folder, filesep, files(i).name);
    [~, fileName, ~] = fileparts(currentFilePath);
    data = readtable(currentFilePath);
    structData = table2struct(data,"ToScalar",true);
    tempOutputStruct.(fileName) = structData;
    
end

save('../data/fordav_sample_data','-struct','tempOutputStruct')
