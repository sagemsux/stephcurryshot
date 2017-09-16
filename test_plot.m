clear; close all; clc;
table = struct2array(load('table.mat'));

%Eliminate zeros from table
for i = 1:size(table,1)
    if table(i,9) == 0
        index = i-1;
        break
    end
end

tableClean = table(1:index,:);
    
xr = tableClean(:,1);
yr = tableClean(:,2);

%Sort table by energy and find index where energy less than curry
tableSortE = sortrows(tableClean,9);
for i = 1:size(table,1)
    if tableSortE(i,9) > 495
        index = i-1;
        break
    end
end

tableEAllow = tableSortE(1:index,:);
set(figure, 'Position', [300, 150, 600, 500]);
hold on
plot(tableEAllow(:,1),tableEAllow(:,2),'ok','MarkerSize',1)
plot(.182,2.372,'ok','MarkerSize',5,'MarkerFaceColor','r')
txt = '   \leftarrow Curry Release Point';
text(.182,2.372,txt)
xlabel('Xe (m)')
ylabel('Ye (m)')
title('Release Points with Energy Less than Control (Curry)')
