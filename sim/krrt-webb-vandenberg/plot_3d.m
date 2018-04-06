function [ scratch ] = plot_3d(scratch, obj, tree, parents, obstacles, goal_state, goal_cost, goal_parent )
%PLOT_GRAPH Summary of this function goes here
%   Detailed explanation goes here

figure(1); clf;
subplot(1,2,1)
view(3);
hold on; grid on; box on;
% if false
%     for ii=2:size(tree,2) %root node has no parents -> start with 2
%         src = tree(:,parents(ii));
%         dst = tree(:,ii);
%         draw_trajectory(obj, src, dst, 'blue', 1);
%     end
% end


    for ii=1:size(obstacles,1)
        obs = obstacles(ii,:);   
        obs(1:3) = obs(1:3)-obs(4:6)/2;
        
        a = obs(1:3);
        b = obs(1:3)+[obs(4),0,0];
        c = obs(1:3)+[obs(4),obs(5),0];
        d = obs(1:3)+[0,obs(5),0];
        e = obs(1:3)+[0,0,obs(6)];
        f = e+[obs(4),0,0];
        g = e+[obs(4),obs(5),0];
        h = e+[0,obs(5),0];
        
        plot_quad(a,b,c,d,'red'); %bottom
        plot_quad(e,f,g,h,'red'); %top
        plot_quad(a,b,f,e,'red'); %front
        plot_quad(d,c,g,h,'red'); %back
        plot_quad(a,d,h,e,'red'); %left
        plot_quad(b,c,g,f,'red'); %right
        
    end

if goal_cost < inf
    p = goal_parent;
    h = draw_trajectory(obj, tree(:,p), goal_state, 'green', 3);
    uistack(h, 'top')
    ptplot = scatter3(goal_state(1), goal_state(2), goal_state(3)); 
    uistack(ptplot, 'top')
    c = p;
    p = parents(c);
    while p > 0
        pt = tree(:,c);
        ptplot = scatter3(pt(1), pt(2), pt(3));
        h = draw_trajectory(obj, tree(:,p), tree(:,c), 'green', 3);
        uistack(h, 'top')
        uistack(ptplot, 'top')
        c = p;
        p = parents(c);
    end
    pt = tree(:,c);
    ptplot = scatter3(pt(1), pt(2), pt(3));
    uistack(ptplot, 'top')
end

hold off;
title(['cost: ', num2str(goal_cost)]);
% daspect([1 1 1]);

subplot(1,2,2)
grid on;
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

function [] = plot_quad(a,b,c,d, color)    
    p = [a;b;c;d]; 
    fill3(p(:,1),p(:,2),p(:,3), zeros(4,1), 'FaceColor', [.8,.8,.8], 'EdgeColor', color);
end

function [h] = draw_trajectory(obj,x0,x1,color, thickness)
    [~, ~, states, ~] = calc_numerical(obj, x0, x1);

    h = line(states(:,1), states(:,2), states(:,3), 'Color', color, 'LineWidth', thickness);
end