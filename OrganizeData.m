close all; clear; clc;
table = struct2array(load('table.mat'));

%Clean up data in table
for i = 1:size(table,1)
    if table(i,9) == 0
        index = i-1;
        break
    end
end

tableClean = table(1:index,:);
minEnergy = min(tableClean(:,9));
tableSortE = sortrows(tableClean,9);
tableSortq1 = sortrows(tableClean,3);
tableq1start = tableSortq1(:,3:9);

i_optimal = 3859;
%q1_target = tableSortE(1,3);
q1_target = tableClean(i_optimal,3);
E_target = tableClean(i_optimal,9);

for i = 1:size(tableq1start,1)
    if tableq1start(i,1) == q1_target
        index_start = i;
        break
    end
end

for i = 1:size(tableq1start,1)
    if i > index_start && tableq1start(i,1) ~= q1_target
        index_end = i;
        break
    end
end

tableq1Target = sortrows(tableq1start(index_start:index_end,:),7);
q2 = tableq1Target(:,2);
q3 = tableq1Target(:,3);
E = tableq1Target(:,7);

set(figure, 'Position', [300, 150, 600, 500]);
hold on
%{
F = scatteredInterpolant(q2*180/pi,q3*180/pi,E);
[qx,qy] = meshgrid(q2*180/pi,q3*180/pi);
qz = F(qx,qy);
mesh(qx,qy,qz);
%}
c = colormap(parula(size(q2,1)));
%
for i = 1:size(q2,1)
    scatter3(q2(i)*180/pi,q3(i)*180/pi,E(i),'o','MarkerFaceColor',c(i,:),'MarkerEdgeColor',c(i,:))
end
%}
%
%}
%{
txt = {'   Curry'};
s = [1]*400;
scatter3([.505*180/pi],[1.3181*180/pi],[530.0682],s,'p','MarkerFaceColor','r')
text([.505*180/pi],[1.3181*180/pi],[530.0682],txt,'FontSize',14,'FontWeight','bold','Color','k')
%}
zlim([200 1000])
xlim([-40 40])
xlabel('q2 (deg)'); 
ylabel('q3 (deg)');
zlabel('Energy (J)');
title('Energy Surface at Optimal Jump Height','FontSize',14)
view([35+90 15])
txt = {'   Optimal','   Curry'};
s = [1,1]*400;
scatter3([tableClean(i_optimal,4)*180/pi .505*180/pi],[tableClean(i_optimal,5)*180/pi 1.3181*180/pi],[tableClean(i_optimal,9) 530.0682],s,'p','MarkerFaceColor','r')
text([tableClean(i_optimal,4)*180/pi .505*180/pi],[tableClean(i_optimal,5)*180/pi 1.3181*180/pi],[tableClean(i_optimal,9) 530.0682],txt,'FontSize',14,'FontWeight','bold','Color','k')

