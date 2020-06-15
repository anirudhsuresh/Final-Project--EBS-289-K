function test_error()

%     scan_route()
%     load('bitmap.mat', 'bitmap')
%     load('occupancy_grid', 'occupancy_grid');
%     detect_trees(occupancy_grid, 'finaloutput.txt');
%     detect_trees(bitmap, 'referenceoutput.txt');
    
    f = fopen('finaloutput.txt', 'r');
    r = fopen('referenceoutput.txt', 'r');
    
    fline = fgetl(f);
    rline = fgetl(r);
    fo = []; ro = [];
    while ischar(fline) && ischar(rline)
        
        if length(fline) <= 2 || length(rline) <= 2
            fline = fgetl(f);
            rline = fgetl(r);
            continue
        end
        
        fsplit_line = strsplit(fline);
        rsplit_line = strsplit(rline);
        for i=1:length(fsplit_line)
            fsplit_line_num(i) = str2double(fsplit_line(i));
            rsplit_line_num(i) = str2double(rsplit_line(i));
        end
        
        fo = [fo; fsplit_line_num(2), fsplit_line_num(3), fsplit_line_num(4)];
        ro = [ro; rsplit_line_num(2), rsplit_line_num(3), rsplit_line_num(4)];
        
        fline = fgetl(f);
        rline = fgetl(r);
    end
    
    e = fo - ro;
    
    figure(1);
    subplot(3,1,1);
    histogram(e(:,1));
    title('X histogram')
    subplot(3,1,2);
    histogram(e(:,2));
    title('Y histogram')
    subplot(3,1,3);
    histogram(e(:,3));
    title('radius histogram')
    
    m_x = mean(e(:,1));
    m_y = mean(e(:,2));
    m_c = mean(e(:,3));
    
    s_x = std(e(:,1));
    s_y = std(e(:,2));
    s_c = std(e(:,3));
    
    r_x = rms(e(:,1));
    r_y = rms(e(:,2));
    r_c = rms(e(:,3));
    
    mn_x = min(e(:,1));
    mn_y = min(e(:,2));
    mn_c = min(e(:,3));
    
    mx_x = max(e(:,1));
    mx_y = max(e(:,2));
    mx_c = max(e(:,3));
    
    p_x = prctile(e(:,1), 95);
    p_y = prctile(e(:,2), 95);
    p_c = prctile(e(:,3), 95);
    
    e_x = [m_x; s_x; r_x; mn_x; mx_x; p_x];
    e_y = [m_y; s_y; r_y; mn_y; mx_y; p_y];
    e_c = [m_c; s_c; r_c; mn_c; mx_c; p_c];
    
    labels = {'Mean'; 'Standard Deviation'; 'RMS'; 'Minimum'; 'Maximum'; '95th Percentile'};
    varnames = {'Error', 'X', 'Y', 'Radius'};
    
    T = table(labels, e_x, e_y, e_c, 'VariableNames', varnames);
    display(T);
    
end