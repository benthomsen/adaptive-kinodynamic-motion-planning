clear; clc;

start = [2,2,8,0];
goal  = [98,98,0,-1];

m = 3; m_p = 1.5;
M = m+m_p;
radius = 1;

state_limits = ...
    [-500,500;
    0,500;
    -16,16;
    -16,16];

sampling_limits = ...
    [0,115;
    0,115;
    -12,12;
    -12,12];

input_limits = ...
    10 * [ -1, 1;
    -1, 1];

state_dims = 4;
input_dims = 2;

A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
% A = [0 0 1 0; 0 0 0 1; -0.01 0 -0.02 0; 0 -0.01 0 -0.02];
% A = [0 0 1 0; 0 0 0 1; 0 0 -0.1 0; 0 0 0 -0.1];
B = [0 0 1/M 0; 0 0 0 1/M]';
c = [0; 0; 0; 0];

R = 0.02*diag([1, 1]);

%obstacles = [40,10,20,80];
% obstacles = [60,0,10,20;
%              60,30,10,70;
%              30,0,10,70;
%              30,80,10,20];
% obstacles = [60,0,10,40;
%              70,60,10,40;
%              20,20,10,10;
%              40,90,20,20];
obstacles = [60,0,1,1;
             70,60,1,1];
% obstacles = [5,60,5,5;
%              13,45,5,5;
%              28,0,5,25;
%              28,40,5,25;
%              28,65,35,12;
%              44,20,20,25;
%              75,50,3,30;
%              70,92,15,15;
%              80,18,5,5;
%              88,38,5,5;
%              83,66,5,5;
%              0,95,50,5;
%              93,93,10,3];

disp('setting parameters');
rrt = rrtstar_v4(A, B, c, R, start', goal',1000,0.02);
disp('starting algorithm');

state_free = @(state)(ball_is_state_free_new(state, state_limits, obstacles, radius));
input_free = @(input)(is_input_free_new(input, input_limits));
sample_state = @()(ball_sample_free_states(sampling_limits, state_limits, obstacles, radius ));
display = @(scratch, obj, tree, parents, goal, goal_cost, goal_parent)(plot_2d_nl(scratch, obj, tree, parents, obstacles, goal, goal_cost, goal_parent));
% displaycost = @(dc_scratch, obj, tree, parents, goal, goal_cost, goal_parent)(ball_plot_states_and_inputs(dc_scratch, obj, tree, parents, goal, goal_cost, goal_parent));

[T, parents] = rrt.run(sample_state, state_free, input_free, start', goal', display, @(x_i)deal(A,B));

states = [];
u = [];
times = [];
for i=2:size(T,2)
    [~, times_i, states_i, u_i] = rrt.calc_numerical(T(:,i-1), T(:,i), A, B);
    states = [states; states_i];
    u = [u; u_i];
    tprev = 0;
    if i > 2
    	tprev = times(end)+0.001;
    end
    times = [times; tprev+times_i];
end
% [states, inputs] = rrt.evaluate_states_and_inputs(start', goal');

% [msgStr,msgId] = lastwarn;
% warnStruct = warning('off',msgId);
