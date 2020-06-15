function [sorted1, sorted2, sorted3] = triple_sort(array1, array2, array3)
% Sort three arrays simultaneously based on values in array2, given that
% each of the arrays are of the same length. Sorting algorithm is based on 
% insertion sort

n1 = length(array1);
n2 = length(array2);
n3 = length(array3);

if n1 ~= n2 && n2 ~= n3 && n3 ~= n1
    error('Arrays not of same size')
end
for i = 2:n2
    d = i;    
    while((d > 1) && (array2(d) < array2(d-1)))
        temp1 = array1(d);
        temp2 = array2(d);
        temp3 = array3(d);
        array1(d) = array1(d-1);
        array2(d) = array2(d-1);
        array3(d) = array3(d-1);
        array1(d-1) = temp1;
        array2(d-1) = temp2;
        array3(d-1) = temp3;
        d = d-1;
    end
end

sorted1 = array1;
sorted2 = array2;
sorted3 = array3;
end