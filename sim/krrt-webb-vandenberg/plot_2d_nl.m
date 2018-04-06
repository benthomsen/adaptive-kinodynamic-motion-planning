function [ scratch ] = plot_2d_nl(scratch, obj, tree, parents, obstacles, goal_state, goal_cost, goal_parent )
%PLOT_GRAPH Summary of this function goes here
%   Detailed explanation goes here

figure(1); clf;
subplot(1,2,1)
hold on; grid on;
% if false
%     for ii=2:size(tree,2) %root node has no parents -> start with 2
%         src = tree(:,parents(ii));
%         dst = tree(:,ii);
%         draw_trajectory(obj, src, dst, 'blue', 1);
%     end
% end

for ii=1:size(obstacles,1)
    obs = obstacles(ii,:);
    a = obs(1:2);
    b = obs(1:2) + [obs(3), 0];
    c = obs(1:2) + [obs(3), obs(4)];
    d = obs(1:2) + [0, obs(4)];
    plot_quad(a,b,c,d); %front
end

if goal_cost < inf
    p = goal_parent;
    h = draw_trajectory(obj, tree(:,p), goal_state, 3, p);
    uistack(h, 'top')
    ptplot = scatter(goal_state(1), goal_state(2)); 
    uistack(ptplot, 'top')
    c = p;
    p = parents(c);
    while p > 0
        pt = tree(:,c);
        ptplot = scatter(pt(1), pt(2));
        h = draw_trajectory(obj, tree(:,p), tree(:,c), 3, p);
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
title(['Cost: ', num2str(goal_cost)]);
% daspect([1 1 1]);

subplot(1,2,2)
grid on;
if goal_cost < inf
    p = goal_parent;

    [~, times_i, ~, u] = calc_numerical(obj, tree(:,p), goal_state, obj.A_nodes(:,:,p), obj.B_nodes(:,:,p));
    U = u;
    times = times_i;
    c = p;
    p = parents(c);
    while p > 0
        [~, times_i, ~, u] = calc_numerical(obj, tree(:,p), tree(:,c), obj.A_nodes(:,:,p), obj.B_nodes(:,:,p));
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

function [] = plot_quad(a,b,c,d)    
    p = [a;b;c;d]; 
    fill(p(:,1),p(:,2), zeros(4,1), 'FaceColor', [0.99, 0.92, 0.8] , 'EdgeColor', [.93,.69,.13]);
end

function [h] = draw_trajectory(obj,x0,x1, thickness, p)
    [~, ~, states, ~] = calc_numerical(obj, x0, x1, obj.A_nodes(:,:,p), obj.B_nodes(:,:,p));

    h = line(states(:,1), states(:,2), 'Color', [0,0.75,0.77], 'LineWidth', thickness);
end