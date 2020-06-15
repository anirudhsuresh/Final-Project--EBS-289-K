function test_produce_bitmap()
    
    R = 5000; C = R;
    Xmax = 42; Ymax = 42;
    
    global bitmap;
    bitmap = zeros(R,C);
    
    file = fopen('finaloutput.txt','r');
    line = fgetl(file);
    while ischar(line)
        
        if length(line) <= 2
            line = fgetl(file);
            continue
        end
        
        split_line = strsplit(line);
        for i=1:length(split_line)
            split_line_num(i) = str2double(split_line(i));
        end
        
        [i, j] = XYtoIJ(split_line_num(2), split_line_num(3), Xmax, Ymax, R, C);
        draw_disc(i, j, split_line_num(4)*(R/Xmax), R, C);
        
        line = fgetl(file);
    end
    
    imagesc(bitmap);
end