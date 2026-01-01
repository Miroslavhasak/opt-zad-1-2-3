function sorted = sortFilenamesByNumericPart(fileStruct)
    paths = {};
    nums = zeros(numel(fileStruct), 1);
    count = 0;

    for i = 1:numel(fileStruct)
        if ~fileStruct(i).isdir
            count = count + 1;
            paths{count} = fullfile(fileStruct(i).folder, fileStruct(i).name);
            name = fileStruct(i).name;
            numStr = regexp(name, '\d+', 'match');
            nums(count) = ~isempty(numStr) * str2double(numStr{end});
        end
    end

    [~, idx] = sort(nums(1:count));
    sorted = paths(idx);
end