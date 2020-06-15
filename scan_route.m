% generateNursery();
angleSpan = 2*pi; angleStep = angleSpan/360; 
Xmax =42; Ymax = 52;

global bitmap
global occupancy_grid
global rangeMax;
rangeMax = 200; % meters
bitmap = flipud(bitmap);
save('bitmap.mat', 'bitmap');

occupancy_grid = 0.5*ones(R,C);

load('Q_TRUE.mat')

% Reduce the size of the total path travelled for faster processing
Q_TRUE = [Q_TRUE(1, :); Q_TRUE(2, :); Q_TRUE(3, :)];
Q_TRUE = Q_TRUE(:,1:25:end); % only use every 10th iteration

%Simulate disturbances in the bitmap like a twig or bush

% noise_threshold = 0.9999; 
% for j=1:C
%     for i=1:R
%         if rand > noise_threshold
%             bitmap(i,j) = 1; %plot tree trunk
%         end
%     end
% end

% run the laser scanner along the path
f = waitbar(0,'1','Name','Calculating scan', 'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
for k = 1:size(Q_TRUE,2)
    Tl = SE2([Q_TRUE(1,k) Q_TRUE(2,k) Q_TRUE(3,k)]);
    p = laserScannerNoisy(angleSpan, angleStep, rangeMax, Tl.T, bitmap, Xmax, Ymax); 
    p = filterNoise(p);
    waitbar(k/size(Q_TRUE,2), f, sprintf('%d/%d', k, size(Q_TRUE,2)))
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);
        % possible infinite range is handled inside the update function
        n = updateLaserBeamGrid(angle, range, Tl.T, R, C, Xmax, Ymax);
    end
    figure(1)
imagesc(occupancy_grid) %plots the probability grid
colorbar;
end
delete(f);

% display the raw occupancy grid
figure(1)
imagesc(occupancy_grid) %plots the probability grid
colorbar;

save('occupancy_grid.mat', 'occupancy_grid');

