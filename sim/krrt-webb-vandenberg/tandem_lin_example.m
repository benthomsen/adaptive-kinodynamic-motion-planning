clear; clc;

start = [2,5,0,0,0,0];
goal  = [98,98,0,0,-0.1,0];

l = 0.5; 
l_p = 0.1; 
m = 3;
m_p = 1.5;
% C_T = 1;
g = 9.81;
I = (m*l^2)/8 + m_p*l_p^2;
radius = 2.5;

state_limits = ...
    [-100,200;
    0,200;
    -pi/4,pi/4;
    -10,10;
    -10,10;
    -2,2];

sampling_limits = ...
    [-20,120;
    0,120;
    -pi/6,pi/6;
    -8,8;
    -8,8;
    -1.5,1.5];

input_limits = ...
    [-5, 5;
     -5, 5];

state_dims = 6;
input_dims = 2;


% A = [0 0 0 1 0 0;
%     0 0 0 0 1 0;
%     0 0 0 0 0 1;
%     0 0 -g 0 0 0;
%     0 0 0 0 0 0;
%     0 0 m_p*l_p*g/I 0 0 0];
A = [0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    0 0 -g 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];

B = [0 0; 0 0; 0 0; 0 0; 1/(m+m_p) 0; 0 1/I];

c = zeros(state_dims,1);
R = 0.1*diag([1,2]);

%obstacles = [40,10,20,80];
% obstacles = [60,0,10,20;
%              60,30,10,70;
%              30,0,10,70;
%              30,80,10,20];
% obstacles = [60,0,10,40;
%              70,60,10,40];
% obstacles = [60,0,1,1;
%              70,60,1,1];

% (center_x, center_y, center_z, width, length, height)
% obstacles = [65,20,20,20,40,40;
%              75,80,80,20,40,40];

% obstacles = [60,0,10,40;
%              70,60,10,40];
obstacles = [5,60,5,5;
             13,45,5,5;
             28,0,5,25;
             28,40,5,25;
             28,65,35,12;
             44,20,20,25;
             75,50,3,30;
             70,92,15,15;
             80,18,5,5;
             88,38,5,5;
             83,66,5,5;
             0,95,50,5;
             93,93,10,3];

disp('setting parameters');
rrt = rrtstar_v4_mod(A, B, c, R, start', goal', 1500, 0.1);
disp('starting algorithm');

[cost_, times_, states_, u_] = rrt.calc_numerical(start', goal', A, B);


state_free = @(state)(ball_is_state_free_new(state, state_limits, obstacles, radius));
input_free = @(input)(is_input_free_new(input, input_limits));
sample_state = @()(ball_sample_free_states(sampling_limits, state_limits, obstacles, radius ));
display = @(scratch, obj, tree, parents, goal, goal_cost, goal_parent)(plot_2d_nl(scratch, obj, tree, parents, obstacles, goal, goal_cost, goal_parent));
% display = @(scratch, obj, tree, parents, goal, goal_cost, goal_parent)(ball_plot_new(scratch, obj, tree, parents, obstacles, goal, goal_cost, goal_parent));

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

% [msgStr,msgId] = lastwarn;
% warnStruct = warning('off',msgId);