function p = filterNoise(inp)
    ranges = inp(:,2);
    filteredRanges = medfilt1(ranges);
    filteredRanges = medfilt1(filteredRanges);
    p = [inp(:,1) filteredRanges];
end