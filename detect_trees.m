function detect_trees(img, file_name)
    R = 5000; C = 5000;
    Xmax_nurs = 42; Ymax_nurs = Xmax_nurs;
%     load('occupancy_grid.mat', 'occupancy_grid')
    % Process the occupancy grid

    figure(2)
    se = strel('disk',1,0);
    bitpos = imclose(img,se);
    BW = imbinarize(bitpos);
    subplot(1,2,1)
    imshow(img);
    title('Raw LIDAR data')
    subplot(1,2,2)
    imshow(BW);
    title('Processed Image')

    figure(3)
    [centers, radii, metric] = imfindcircles(BW,[20 60], 'Method', 'TwoStage', ... 
        'ObjectPolarity', 'bright', 'Sensitivity', 0.88);
    imshow(BW)
    hold on
    viscircles(centers, radii,'EdgeColor','b');

    % Tree detection parameter initialization

    Xmax_nurs = 42; Ymax_nurs = 42;
    Krows = 5;
    nTrees = 11;

    % This takes the tree positions from generate nursery and computes the locations in the bitmap
    for i = 1:Krows
        [~,yC(i)] = XYtoIJ(3*(i-1)+18.5,2*(i-1)+18,Xmax_nurs,Ymax_nurs,R,C); 
    end

    for i = 1:nTrees+1
        [xC(i),~] = XYtoIJ(3*(i-1)+18.5,2*(i-1)+20,Xmax_nurs,Ymax_nurs,R,C);
    end

    minX = min(xC); maxX = max(xC); minY = min(yC); maxY = max(yC);
    
    % define a search range to look for trees in
    searchrange_i = 50; 
    searchrange_j = 50; 
    
    row1 = []; row2 = []; row3 = []; row4 = []; row5 = [];
    
    % detect the positions of treees around the search ranges and append
    % them to their respective rows
    for i = 1:length(radii)
        if centers(i,1) < yC(1) + searchrange_i && centers(i,1) > yC(1) - searchrange_i
            if centers(i,2) < maxX+searchrange_j && centers(i,2) > minX-searchrange_j
                row1 = [row1 i];
            end
        end
        if centers(i,1) < yC(2) + searchrange_i && centers(i,1) > yC(2) - searchrange_i
            if centers(i,2) < maxX+searchrange_j && centers(i,2) > minX-searchrange_j
                row2 = [row2 i];
            end
        end
        if centers(i,1) < yC(3) + searchrange_i && centers(i,1) > yC(3) - searchrange_i
            if centers(i,2) < maxX+searchrange_j && centers(i,2) > minX-searchrange_j
                row3 = [row3 i];
            end
        end
        if centers(i,1) < yC(4) + searchrange_i && centers(i,1) > yC(4) - searchrange_i
            if centers(i,2) < maxX+searchrange_j && centers(i,2) > minX-searchrange_j
                row4 = [row4 i];
            end
        end
        if centers(i,1) < yC(5) + searchrange_i && centers(i,1) > yC(5) - searchrange_i
            if centers(i,2) < maxX+searchrange_j && centers(i,2) > minX-searchrange_j
                row5 = [row5 i];
            end
        end    
    end

    index_t = 1;
    
    % Calculate the centers and radii of all the trees
    for i=1:length(row1)
        [x1(i),y1(i)] = IJtoXY(centers(row1(i),2),centers(row1(i),1),Xmax_nurs,Ymax_nurs,R,C);
        c1(i) = radii(row1(i));
        c1(i) = c1(i)*(Xmax_nurs/R);
        y1(i) = Ymax_nurs-y1(i);
        trees_percieved(index_t,:) = [x1(i) y1(i) c1(i)];
        index_t = index_t+1; 
    end

    for i=1:length(row2)
        [x2(i),y2(i)] = IJtoXY(centers(row2(i),2),centers(row2(i),1),Xmax_nurs,Ymax_nurs,R,C);
        c2(i) = radii(row2(i));
        c2(i) = c2(i)*(Xmax_nurs/R);
        y2(i) = Ymax_nurs-y2(i);
        trees_percieved(index_t,:) = [x2(i) y2(i) c2(i)];
        index_t = index_t+1; 
    end

    for i=1:length(row3)
        [x3(i),y3(i)] = IJtoXY(centers(row3(i),2),centers(row3(i),1),Xmax_nurs,Ymax_nurs,R,C);
        c3(i) = radii(row3(i));
        c3(i) = c3(i)*(Xmax_nurs/R);
        y3(i) = Ymax_nurs-y3(i);
        trees_percieved(index_t,:) = [x3(i) y3(i) c3(i)];
        index_t = index_t+1; 
    end

    for i=1:length(row4)
        [x4(i),y4(i)] = IJtoXY(centers(row4(i),2),centers(row4(i),1),Xmax_nurs,Ymax_nurs,R,C);
        c4(i) = radii(row4(i));
        c4(i) = c4(i)*(Xmax_nurs/R);
        y4(i) = Ymax_nurs-y4(i);
        trees_percieved(index_t,:) = [x4(i) y4(i) c4(i)];
        index_t = index_t+1; 
    end

    for i=1:length(row5)
        [x5(i),y5(i)] = IJtoXY(centers(row5(i),2),centers(row5(i),1),Xmax_nurs,Ymax_nurs,R,C);
        c5(i) = radii(row5(i));
        c5(i) = c5(i)*(Xmax_nurs/R);
        y5(i) = Ymax_nurs-y5(i);
        trees_percieved(index_t,:) = [x5(i) y5(i) c5(i)];
        index_t = index_t+1; 
    end

    % Sort each of the trees detected based on their y position
    [x1,y1,c1] = triple_sort(x1,y1,c1);
    [x2,y2,c2] = triple_sort(x2,y2,c2);
    [x3,y3,c3] = triple_sort(x3,y3,c3);
    [x4,y4,c4] = triple_sort(x4,y4,c4);
    [x5,y5,c5] = triple_sort(x5,y5,c5);
    
    % Output to a text file
    
    fileID = fopen(file_name,'w');

    fprintf(fileID,'1 \n');
    for i = 1:length(row1)
        fprintf(fileID,'%d, %.2f, %.2f, %.2f\n',i,x1(i),y1(i),c1(i));
    end

    fprintf(fileID,'2 \n');
    for i = 1:length(row2)
        fprintf(fileID,'%d, %.2f, %.2f, %.2f\n',i,x2(i),y2(i),c2(i));
    end

    fprintf(fileID,'3 \n');
    for i = 1:length(row3)
        fprintf(fileID,'%d, %.2f, %.2f, %.2f\n',i,x3(i),y3(i),c3(i));
    end

    fprintf(fileID,'4 \n');
    for i = 1:length(row4)
        fprintf(fileID,'%d, %.2f, %.2f, %.2f\n',i,x4(i),y4(i),c4(i));
    end

    fprintf(fileID,'5 \n');
    for i = 1:length(row5)
        fprintf(fileID,'%d, %.2f, %.2f, %.2f\n',i,x5(i),y5(i),c5(i));
    end
