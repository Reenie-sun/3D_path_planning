clear;
clc;

latlim = [36 37];
lonlim = [127.5 128.5];

n37e128 = dteds(latlim, lonlim, 0);

samplefactor = 1;
[capeterrain, caperef] = dted('n37e128.dt0', samplefactor, latlim, lonlim);


% ls(fullfile(matlabroot, 'toolbox', 'map', 'mapdata'));
% 
% capeterrain(capeterrain == 0) = -1;
% 
% figure;
% ax = usamap(latlim, lonlim);
% 
% capecoast = shaperead('n37e128.dt0');
% 
% geoshow(ax, capecoat, 'FaceColor', 'none');