function mykeyboard(action, src, event)
keys = evalin('base','keys');
switch action
    case 'down'
        k = lower(event.Key);
        if isfield(keys,k), keys.(k) = 1; end
        assignin('base','keys',keys);
        if strcmp(k,'q'), assignin('base','keep_running',false); end
    case 'up'
        k = lower(event.Key);
        if isfield(keys,k), keys.(k) = 0; end
        assignin('base','keys',keys);
    case 'close'
        assignin('base','keep_running',false);
        delete(src);
end
end
