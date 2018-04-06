classdef rrtstar_v3
    %RRTSTAR Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
        A_ = [];
        B_ = [];
        c_ = [];
        R_ = [];
        best_arrival = [];
%         t_s = 9e99; %optimal arrival time
%         min_dist = 9e99;
        x0 = [];
        x1 = [];
        timings = [];

%         t = sym('t', 'positive');
%         x = sym('x', 'positive'); %used for integration
%         t_s = sym('t_s', 'real'); %optimal arrival time
%         min_dist = sym('d', 'real');

%         states_eq = [];
%         control_eq = [];
%         cost_eq = [];
%         distance_eq = [];

%         eval_arrival_internal;
%         eval_cost_internal;
%         eval_states_internal;
%         eval_control_internal;
%         eval_time_for_distance_internal;

        %termination conditions (per segment)
        max_it = 300;
        max_time = 0;
        gamma = 1000;
        max_radius = 1000;
    end
    
    methods
        function obj = rrtstar_v3(A, B, c, R, start, goal) % this will only work for linear systems

            if rank(ctrb(A,B)) ~= size(A,1)
                disp('System is not controllable - aborting');
                return;
            end

            obj.A_ = A;
            obj.B_ = B;
            obj.c_ = c;
            obj.R_ = R;

            if (size(start,1) == 1) && (size(start,2) ~= 1)
                obj.x0 = start';
                obj.x1 = goal';
            else
                obj.x0 = start;
                obj.x1 = goal;
            end        
        end

        function [obj] = set_termination_conditions(obj, iterations, time)
            obj.max_it = iterations;
            obj.max_time = time;
        end

        
        function [cost, times, states, u] = calc_numerical(obj, x0, x1)
            dim = length(obj.A_);
            opts = odeset('AbsTol',1e-3,'RelTol',1e-3,'Events',@termEvents);
            [t, X, ~, ~, ~] = ode45(@sys_dynamics, 0:0.1:200, [zeros(dim^2,1); x0], opts, obj.A_, obj.B_, obj.c_, obj.R_, x1);
            tau_star = t(end);
            X_fin = X(end,:);
            G_array = reshape(X_fin(1:dim^2), [dim, dim]);
            x_bar_vec = X_fin(dim^2+1:end)';
            cost = tau_star + ((x1 - x_bar_vec)'/G_array) * (x1 - x_bar_vec);
            d_star = G_array \ (x1 - x_bar_vec);
            
            opts2 = odeset('AbsTol',1e-3,'RelTol',1e-3);
            [times_backwards, X2] = ode45(@state_solution, tau_star:-0.1:0, [x1; d_star], opts2, obj.A_, obj.B_, obj.c_, obj.R_);
            times = flip(times_backwards);
            states = flip(X2(:,1:dim));
            u = zeros(length(times),size(obj.B_,2));
            for i = 1:length(times)
                y = flip(X2(i,dim+1:end))';
                u(i,:) = (obj.R_ \ obj.B_') * y;
            end
%             u = (obj.R_ \ obj.B_') * X2(dim+1:end,:)
            [msgStr,msgId] = lastwarn;
            warnStruct = warning('off',msgId);

            function dXdt = sys_dynamics(~, X, A, B, c, R, x1)
                dim_lin = length(reshape(A,[],1));
                G_vec   = X(1:dim_lin);
                x_bar   = X(dim_lin+1:dim_lin+length(A));
                dXdt   = zeros(dim_lin+length(A),1);

                G = reshape(G_vec,[length(A), length(A)]);
                G_dot = A*G + G*A' + B/R*B';
                dXdt(1:dim_lin)                     = reshape(G_dot, [], 1);
                dXdt(dim_lin+1:dim_lin+length(A))   = A*x_bar + c;
            end
            
            function [value, isterminal, direction] = termEvents(~, X, A, B, c, R, x1)
                dim_lin = length(reshape(A,[],1));
                G = reshape(X(1:dim_lin),[length(A), length(A)]);
                d = G \ (x1 - X(dim_lin+1:dim_lin+length(A)));
                value = 1 - 2*(A * x1 + c)'*d - d'*B/R*B'*d;
                isterminal = 1; % i.e. condition terminates execution
                direction = 1; 
            end
            
            function dXdt = state_solution(~, X, A, B, c, R)
                dXdt = [A, B/R*B'; zeros(length(A)), -A']*X + [c; zeros(length(A),1)];
            end

        end
        
        % rrtstar.run:
        function [path_states, closest_end_state] = run(obj, sample_free_state, is_state_free, is_input_free, start, goal, display, max_distance)
            T = [start]; % tree
            costs = [0];
            times = [0];
            parents = [-1];
            children = {[]};
            is_terminal = [false];
            cost_to_goal = [inf];
            time_to_goal = 0;

            goal_cost = inf;
            goal_parent = 0;
            goal_time = inf;

            it_limit = obj.max_it; % maximum number of iterations

            
            [cost, ~, states, u] = calc_numerical(obj, start, goal);
%             time = times_i(end);
            
            % these two functions are arguments for rrtstar.run
            % for dbl_int, see "ball_is_state_free" and "is_input_free"
            if is_state_free(states) && is_input_free(u)
                disp('goal is reachable from the start node (optimal solution)');
                it_limit = 0;
                goal_parent = 1;
                display(-1, obj, start, -1, goal, cost, 1);
            end

            display_scratch = -1;
            
            % start iterating if there isn't a free path from start to goal
            it = 0;
            while it<it_limit && goal_time > obj.max_time
                it = it+1;
                disp(['iteration ',num2str(it), ' of ', num2str(it_limit), ' (time:',num2str(goal_time),')']);
                sample_ok = false;
                tic;
                
                obj.max_radius = obj.gamma * log(it+1)/it;
                disp(['*** max radius: ', num2str(obj.max_radius), ' ***']);
                % sample points until there are no collisions and no input
                % saturation (go through all existing nodes to check - BFS)
                
                inner_i = 0;
                while ~sample_ok
                    inner_i = inner_i+1;
                    if ~mod(inner_i,50)
                        inner_i
                    end
                    x_i = sample_free_state(); % sample a random state within bounds and outside of obstacles
%                     x_i = sample_free_state(obj.x0); % sample a random state within bounds and outside of obstacles
                    min_idx = 1;
                    min_cost = inf;
                    min_time = inf;

                    queue = 1;
                    while ~isempty(queue)
                        node_idx = queue(1);
                        queue = queue(2:end); %switch with stack to perform DFS instead of BFS
                        node = T(:,node_idx); % start at root of tree
                        if is_terminal(node_idx)
                           continue;
                        end

                        [cost, times_i, states, u] = calc_numerical(obj, node, x_i);
                        time = times_i(end);
                        if cost < obj.max_radius && costs(node_idx)+cost < min_cost && costs(node_idx)+cost < goal_cost
                            free_state = is_state_free(states);
%                             if free_state
%                                 disp('*** free state ***'); states
%                             end
                            free_input = is_input_free(u);
%                             if free_input
%                                 disp('*** free input ***'); u
%                             end
                            
                            if  free_state && free_input
                                sample_ok = true;
                                min_idx = node_idx;
                                min_cost = costs(node_idx)+cost;
                                min_time = times(node_idx)+time;
                                continue;
                            end
                        end
                        %if we arrive here it means there is no trajectory from node to new x_i
                        %however child nodes might be able to form a connection
                        queue = [queue, children{node_idx}];
                    end
                end

                parents = [parents, min_idx]; % add parent of newly added node to list of parents
                children{min_idx} = [children{min_idx},it+1]; % nodes referred to by the iteration they were added?
                children{it+1} = [];
                costs = [costs, min_cost]; % add cost to new node to list of costs
                times = [times, min_time];
                T = [T,x_i]; % add new node to tree
                is_terminal = [is_terminal, false]; % this new node isn't a terminal node
                cost_to_goal = [cost_to_goal, inf];
                time_to_goal = [time_to_goal, inf];

                %update the tree with new shorter paths (the root node is checked here even though it's not necessary)
                stack(1).index = 1;
                stack(1).improvement = 0;
                stack(1).time_improvement = 0;

                % try and find cheaper paths through the new node
                while ~isempty(stack)
                    node = stack(end);
                    stack = stack(1:end-1);
                    state = T(:,node.index);

                    costs(node.index) = costs(node.index)-node.improvement;
                    times(node.index) = times(node.index)-node.time_improvement;
                    if is_terminal(node.index)
                        if costs(node.index)+cost_to_goal(node.index) < goal_cost
                            goal_cost = costs(node.index)+cost_to_goal(node.index);
                            goal_time = times(node.index)+time_to_goal(node.index);
                            goal_parent = node.index;
                            disp('*** better path? (1)***');
                        end
                        continue;
                    end
                    diff = node.improvement;
                    time_diff = node.time_improvement;

                    [cost, ~, states, u] = calc_numerical(obj, x_i, state);
%                     time = times_i(end);
                    if cost < obj.max_radius && costs(end)+cost < costs(node.index)
                        if is_state_free(states) && is_input_free(u)

                            old_cost = costs(node.index);
                            old_time = times(node.index);
                            old_parent = parents(node.index);
                            costs(node.index) = costs(end)+cost;
                            parents(node.index) = size(parents,2);
                            ch = children{old_parent};
                            children{old_parent} = ch(ch~=node.index);

                            diff = old_cost-costs(node_idx);
                            time_diff = old_time-times(node_idx);
                            disp('*** rewiring ***');
                        end
                    end

                    for jj=1:length(children{node.index})
                       stack(end+1).index = children{node.index}(jj);
                       stack(end).improvement = diff;
                       stack(end).time_improvement = time_diff;
                    end

                end

                % check from new point to goal state
                [cost, times_i, states, u] = calc_numerical(obj, x_i, goal);
                time = times_i(end);
%                 [cost, time] = evaluate_cost(obj, x_i, goal);
                if costs(end)+cost < goal_cost
%                     [states, u] = evaluate_states_and_inputs(obj, x_i,goal,time);
                    if is_state_free(states) && is_input_free(u)
                        goal_cost = costs(end)+cost;
                        goal_time = times(end)+time;
                        goal_parent = it+1;
                        is_terminal(end) = true;
                        cost_to_goal(end) = cost;
                        disp('*** better path? (2)***');
                    end
                end

                obj.timings = [obj.timings, toc()]; % computation time
                display_scratch = display(display_scratch, obj, T, parents, goal, goal_cost, goal_parent);
                disp(['*** goal cost: ', num2str(goal_cost), ' ***']);
                
                if mod(length(obj.timings),50) == 1 && length(obj.timings) > 50
                   disp(['time for the last 50 iterations: ', num2str(sum(obj.timings(end-50:end))),' seconds']); 
                end
                
            end

            % by here, goal should be found
            % climb tree from goal to root
            next = goal_parent;
            path_states = goal;
            while next ~= -1
                path_states = [T(:,next) ,path_states];
                next = parents(next);
            end

            
            % either made it to goal or failed, in which case find closest
            % state to goal
            if ~exist('max_distance','var') || isempty(max_distance)
                closest_end_state = goal;
            else
                src = T(:,goal_parent);
                best_t = obj.evaluate_arrival_time(src, goal);
                close_t = get_time_for_distance_equal(obj, src, goal, best_t, max_distance);
                [s, ~] = evaluate_states_and_inputs(obj, src, goal, best_t);
                closest_end_state = s(close_t);
            end

        end

        % this is only used when there are multiple waypoints (i.e. for
        % quad)
        function [path, time] = find_path(obj, sample_free_state, is_state_free, is_input_free, waypoints, display, max_distance)
            time = 0;
            path = [waypoints(:,1)];
            for ii=2:size(waypoints,2)
                figure('name',[num2str(ii-1),' to ', num2str(ii)]);
                [path_states, closest_end_state] = obj.run(sample_free_state, is_state_free, is_input_free, path(:,end), waypoints(:,ii), display, max_distance);
                path = [path, path_states(:,2:end)];
                if ii ~= size(waypoints,2)
                    path(:,end) = closest_end_state;
                end
            end

            for ii=2:size(path,2)
                time = time + obj.evaluate_arrival_time(path(:,ii-1), path(:,ii));
            end
        end
    end
end

