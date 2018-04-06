function [ ok ] = ball_is_state_free_new( states, state_limits, obstacles, radius )
%IS_STATE_FREE returns true if the given state is valid
% - state is the 10 dimensional state vector
% - state_limits limits for the state variables
% - obstacles is an n by 6 matrix where each row contains one corner and
% the distance to the other
% - quad_dim contains the size of the quadcopter bounding-box

ok = true;
max_dist = .1;

for i=1:length(states)
    state = states(i,:);
    for ii=1:size(state_limits, 1)
        if state(ii) < state_limits(ii, 1) || state(ii) > state_limits(ii, 2)
            ok = false;
            return;
        end
    end
    if collides(obstacles, radius, state)
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
    
    if s(1)>obs(1) && s(1)<obs(1)+obs(3) && s(2)>obs(2) && s(2)<obs(2)+obs(4)
        coll = true;
        return;
    end
    
    closest = min(max(s(1:2),obs(1:2)),obs(1:2)+obs(3:4));
    d = s(1:2)-closest;
    if sum(d.^2) < radius^2
        coll = true;
        return;
    end
    
end
            
end

