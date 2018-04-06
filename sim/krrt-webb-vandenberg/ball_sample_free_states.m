function [ state ] = ball_sample_free_states( sampling_limits, state_limits, obstacles, radius )

not_done = true;
while not_done

    state = rand(size(sampling_limits,1), 1);
    state = state.*(sampling_limits(:,2)-sampling_limits(:,1))+sampling_limits(:,1);

    not_done = ~ball_is_state_free(state, state_limits, obstacles, radius);

end


end
