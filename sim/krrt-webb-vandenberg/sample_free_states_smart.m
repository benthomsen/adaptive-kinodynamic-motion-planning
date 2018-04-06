function [ state ] = sample_free_states_smart( sampling_limits, state_limits, obstacles, radius )

not_done = true;
while not_done

%     pos_state = rand(3, 1);
%     pos_state = pos_state.*(sampling_limits(1:3,2)-sampling_limits(1:3,1))+sampling_limits(1:3,1);
% 
%     if (size(ref_state,1) == 1) && (size(ref_state,2) ~= 1)
%         ref_state = ref_state';
%     end
% 
%     means = ref_state(4:end);
%     stddevs = 0.1 * (sampling_limits(4:end,2)-sampling_limits(4:end,1));
%     other_state = normrnd(means, stddevs, size(sampling_limits(4:end,1)));
%     state = [pos_state; other_state];
%     state = max(state, sampling_limits(:,1));
%     state = min(state, sampling_limits(:,2));
    
    state = rand(length(state_limits), 1);
    state = state.*(sampling_limits(:,2)-sampling_limits(:,1))+sampling_limits(:,1);

    not_done = ~ball_is_state_free(state, state_limits, obstacles, radius);

end


end

% sample_free_states_smart(state_limits, obstacles, radius, start)