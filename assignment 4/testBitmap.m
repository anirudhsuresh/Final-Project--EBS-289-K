clear all;
global bitmap;
global rangeMax;

%lidar values
rangeMax = 200; % meters
angleSpan = pi; angleStep = angleSpan/360; 

Xmax = 150; Ymax = 150; %physical dimensions of space (m)

R = 500; C = 500; %rows and columns; discretization of physical space in a grid
map=zeros(R, C);
bitmap = 0.0* ones(R, C); %initialize as empty

%create test rectangular obstacle
Xsw=70; Ysw = 50;
Xne=Xsw + 30; Yne= Ysw + 20;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, Xmax, Ymax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, Xmax, Ymax, R, C);
map(Ine:Isw, Jsw:Jne) = 1;

for k=1:5
    Tl = SE2([10+10*k  5 pi/2]);
    p = laserScanner(angleSpan, angleStep, rangeMax, Tl.T, map, Xmax, Ymax);  
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);
        % possible infinite range is handled inside the update function
        n = updateLaserBeamBitmap(angle, range, Tl.T, R, C, Xmax, Ymax);
    end
end

imagesc(bitmap);
colorbar
