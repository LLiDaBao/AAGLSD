close all
clear
% txtstr='P1020822.txt';
txtstr='P1020856.txt';
percent=1;
lines=importdata(txtstr);

lensm=(lines(:,1)-lines(:,3)).^2+(lines(:,2)-lines(:,4)).^2;
[~, idx]=sort(lensm,'descend');
lines=lines(idx(1:end*percent),:);

zz=zeros(480,640)+256;
imshow(zz);
hold on
for i=1:1:size(lines,1)
% for i=1:1:size(line_gnd,1)
%     line=line_gnd(i,:);
line=lines(i,:);
plot([line(1),line(3)],[line(2),line(4)], 'Color','k','LineWidth',2);
hold on
end
figure(2)
zz=zeros(480,640)+256;
imshow(zz);
lines=importdata([txtstr(1:end-4) 'linelet.txt']);
lenslinelet=(lines(:,1)-lines(:,3)).^2+(lines(:,2)-lines(:,4)).^2;
[~, idx]=sort(lenslinelet,'descend');
lines=lines(idx(1:end*percent),:);

hold on
for i=1:1:size(lines,1)
% for i=1:1:size(line_gnd,1)
%     line=line_gnd(i,:);
line=lines(i,:);
plot([line(1),line(3)],[line(2),line(4)], 'Color','black','LineWidth',1);
hold on
end

figure(3)
zz=zeros(480,640)+256;
imshow(zz);
lines=importdata([txtstr(1:end-4) 'lsd.txt']);
lensmcm=(lines(:,1)-lines(:,3)).^2+(lines(:,2)-lines(:,4)).^2;

hold on
for i=1:1:size(lines,1)
% for i=1:1:size(line_gnd,1)
%     line=line_gnd(i,:);
line=lines(i,:);
plot([line(1),line(3)],[line(2),line(4)], 'Color','black','LineWidth',1);
hold on
end

figure(4)
zz=zeros(480,640)+256;
imshow(zz);
lines=importdata([txtstr(1:end-4) 'ed.txt']);
lens=(lines(:,1)-lines(:,3)).^2+(lines(:,2)-lines(:,4)).^2;

hold on
for i=1:1:size(lines,1)
% for i=1:1:size(line_gnd,1)
%     line=line_gnd(i,:);
line=lines(i,:);
plot([line(1),line(3)],[line(2),line(4)], 'Color','black','LineWidth',1);
hold on
end


figure(5)
zz=zeros(480,640)+256;
imshow(zz);
lines=importdata([txtstr(1:end-4) 'ppht.txt']);
lens=(lines(:,1)-lines(:,3)).^2+(lines(:,2)-lines(:,4)).^2;

hold on
for i=1:1:size(lines,1)
% for i=1:1:size(line_gnd,1)
%     line=line_gnd(i,:);
line=lines(i,:);
plot([line(1),line(3)],[line(2),line(4)], 'Color','black','LineWidth',1);
hold on
end

figure(6)
zz=zeros(480,640)+256;
imshow(zz);
lines=importdata([txtstr(1:end-4) 'mcm.txt']);

hold on
for i=1:1:size(lines,1)
% for i=1:1:size(line_gnd,1)
%     line=line_gnd(i,:);
line=lines(i,:);
plot([line(1),line(3)],[line(2),line(4)], 'Color','black','LineWidth',1);
hold on
end