%% CSV Parser

clear
clc
close all

%% Parsing

rootDir = fullfile(fileparts(which(mfilename)), "..");
dataDir = fullfile(rootDir, "data");
logs = dir(dataDir);

for i=4:length(logs)

    currentLogPath = fullfile(dataDir, logs(i).name, "*.csv");
    files = dir(currentLogPath);

    for j = 1:length(files)

        currentFilePath = fullfile(files(j).folder, files(j).name);
        [~, fileName, ~] = fileparts(currentFilePath);
        data = readtable(currentFilePath);
        structData = table2struct(data,"ToScalar",true);
        tempOutputStruct.(fileName) = structData;

    end

    outputDir = fullfile(dataDir, logs(i).name, logs(i).name);
    save(outputDir,'-struct','tempOutputStruct')
    
end


