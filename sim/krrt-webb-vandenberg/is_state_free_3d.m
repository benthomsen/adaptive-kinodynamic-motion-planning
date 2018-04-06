function [ ok ] = is_state_free_3d( states, state_limits, obstacles, radius )
%IS_STATE_FREE returns true if the given state is valid
% - state is the 10 dimensional state vector
% - state_limits limits for the state variables
% - obstacles is an n by 6 matrix where each row contains one corner and
% the distance to the other
% - quad_dim contains the size of the quadcopter bounding-box

ok = true;
% max_dist = .1;

for i=1:size(states,1)
    state = states(i,:);
    for ii=1:size(state_limits, 1)
        if state(ii) < state_limits(ii, 1) || state(ii) > state_limits(ii, 2)
%             disp(['*** state ', num2str(ii), ' = ', num2str(state(ii)), ' ***']);
            ok = false;
            return;
        end
    end
    if collides(obstacles, radius, state)
        disp('*** collision ***');
        ok = false;
        return;
    end
end
end

function [coll] = collides(obstacles, radius, s)

n_obs = size(obstacles, 1);
coll = false;
for ii=1:n_obs
    
    obs = obstacles(ii,:)';
    obs(1:3) = obs(1:3)-obs(4:6)/2;
    
%     c_min_bl = s(1:3)-obs(1:3);
%     c_min_tr = s(1:3)-(obs(1:3)+obs(4:6));
    
    if s(1)>obs(1) && s(1)<obs(1)+obs(4) && s(2)>obs(2) && s(2)<obs(2)+obs(5) && s(3)>obs(3) && s(3)<obs(3)+obs(6)
        coll = true;
        return;
    end
     
    closest = min(max(s(1:3),obs(1:3)),obs(1:3)+obs(4:6));
    d = s(1:3)-closest;
    if sum(d.^2) < radius^2
        coll = true;
        return;
    end
    
    
end
            
end
