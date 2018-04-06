function [ ok ] = is_input_free_new( inputs, input_limits )
%IS_INPUT_FREE Summary of this function goes here
%   Detailed explanation goes here

ok = true;

for i=1:length(inputs)
    input = inputs(i,:);
    for ii=1:size(input_limits, 1)
        if input(ii) < input_limits(ii, 1) || input(ii) > input_limits(ii, 2)
            ok = false;
            return;
        end
    end
end

end

