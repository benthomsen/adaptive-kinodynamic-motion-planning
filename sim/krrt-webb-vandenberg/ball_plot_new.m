function [ scratch ] = ball_plot_new(scratch, obj, tree, parents, obstacles, goal_state, goal_cost, goal_parent )
%PLOT_GRAPH Summary of this function goes here
%   Detailed explanation goes here

figure(1); clf;
subplot(2,1,1)
hold on;
% if false
%     for ii=2:size(tree,2) %root node has no parents -> start with 2
%         src = tree(:,parents(ii));
%         dst = tree(:,ii);
%         draw_trajectory(obj, src, dst, 'blue', 1);
%     end
% end


for ii=1:size(obstacles,1)
    obs = obstacles(ii,:);
    line([obs(1),        obs(1)+obs(3)], [obs(2),        obs(2)],         'Color','red');
    line([obs(1)+obs(3), obs(1)+obs(3)], [obs(2),        obs(2)+obs(4)],  'Color','red');
    line([obs(1)+obs(3), obs(1)],        [obs(2)+obs(4), obs(2)+obs(4)],  'Color','red');
    line([obs(1),        obs(1)],        [obs(2)+obs(4), obs(2)],         'Color','red');
end

if goal_cost < inf
    p = goal_parent;
    h = draw_trajectory(obj, tree(:,p), goal_state, 'green', 3);
    uistack(h, 'top')
    ptplot = scatter(goal_state(1), goal_state(2)); 
    uistack(ptplot, 'top')
    c = p;
    p = parents(c);
    while p > 0
        pt = tree(:,c);
        ptplot = scatter(pt(1), pt(2));
        h = draw_trajectory(obj, tree(:,p), tree(:,c), 'green', 3);
        uistack(h, 'top')
        uistack(ptplot, 'top')
        c = p;
        p = parents(c);
    end
    pt = tree(:,c);
    ptplot = scatter(pt(1), pt(2));
    uistack(ptplot, 'top')
end

hold off;
title(['cost: ', num2str(goal_cost)]);


subplot(2,1,2)

if goal_cost < inf
    p = goal_parent;

    [~, times_i, ~, u] = calc_numerical(obj, tree(:,p), goal_state);
    U = u;
    times = times_i;
    c = p;
    p = parents(c);
    while p > 0
        [~, times_i, ~, u] = calc_numerical(obj, tree(:,p), tree(:,c));
        U = [u; U];
        times = [times_i; times+times_i(end)];
        c = p;
        p = parents(c);
    end
    hold on;
    for i=1:size(U,2)
        plot(times,U(:,i)); 
    end
    hold off;    
end
drawnow
end

function [h] = draw_trajectory(obj,x0,x1,color, thickness)

%     t = obj.evaluate_arrival_time(x0,x1);
%     [states, ~] = obj.evaluate_states_and_inputs(x0,x1);

    [~, ~, states, ~] = calc_numerical(obj, x0, x1);

    h = line(states(:,1), states(:,2), 'Color', color, 'LineWidth', thickness);
end