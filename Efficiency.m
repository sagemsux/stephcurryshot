clc; clear; close all;
table = struct2array(load('table.mat'));

%Eliminate zeros from table
for i = 1:size(table,1)
    if table(i,9) == 0
        index = i-1;
        break
    end
end
tableClean = table(1:index,:);


pEnergy = zeros(size(tableClean,1),1);
pMotion = zeros(size(pEnergy));
N = zeros(size(pEnergy));


for i = 1:size(tableClean,1)
    [pEnergy(i),pMotion(i),N(i)] = calcConGrad(tableClean(i,3),tableClean(i,4),tableClean(i,5),tableClean(i,6),tableClean(i,7),tableClean(i,8),tableClean(i,9));
end

tableEff = [tableClean(:,:) pEnergy pMotion pEnergy./pMotion];

spectralRatio = max(tableEff(:,12));

for i = 1:size(tableEff,1)
    if tableEff(i,12) == spectralRatio
        index = i;
        break
    end
end

spectralRow = tableEff(i,:);