function [v_cmd,w_cmd,v_speed,w_speed] = control_update(keys,v_speed,w_speed,v_max,w_max,accel,friction)
if keys.w, v_speed = min(v_speed+accel,v_max); end
if keys.s, v_speed = max(v_speed-accel,-v_max); end
if ~keys.w && ~keys.s, v_speed = v_speed*friction; end
if keys.a, w_speed = min(w_speed+accel,w_max); end
if keys.d, w_speed = max(w_speed-accel,-w_max); end
if ~keys.a && ~keys.d, w_speed = w_speed*friction; end
v_cmd = v_speed;
w_cmd = w_speed;
end
