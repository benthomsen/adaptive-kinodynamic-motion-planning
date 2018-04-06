% Adaptive sliding hierarchical controller for tandem-rotor sys
% Ben Thomsen
clear
clc

%% setup and variables
% load sample_data4.mat states u times;
% xd_ts = timeseries([states(:,1), states(:,3), u(:,1)], times);
% zd_ts = timeseries([states(:,2), states(:,4), u(:,2)], times);
% 
times = 0:0.01:300;

xd = zeros(length(times),3);
zd = zeros(length(times),3);
T = 10;
T2 = 1.25;
for i = 1:length(times)
    t = i/100;
    
    xd(i,1) = 10 * cos(pi*t/T)              + 0.2 * cos(pi*t/T2);
    xd(i,2) = -(10*pi/T) * sin(pi*t/T)      - 0.2 * (pi/T2) * sin(pi*t/T2);
    xd(i,3) = -(10*pi^2)/(T^2) * cos(pi*t/T)- 0.2 * (pi/T2)^2 * cos(pi*t/T2);
    
    zd(i,1) = 10 * sin(pi*t/T)              + 0.2 *  cos(pi*t/T2);
    zd(i,2) = (10*pi/T) * cos(pi*t/T)       - 0.2 *  (pi/T2) * sin(pi*t/T2);
    zd(i,3) = -(10*pi^2)/(T^2) * sin(pi*t/T)- 0.2 *  (pi/T2)^2 * cos(pi*t/T2);
end

xd_ts = timeseries(xd, times);
zd_ts = timeseries(zd, times);

plan_mass = 1;

tfin = times(end);

l = 0.5; 
l_p = 0.4; 
m = 3;
m_p = 2;
c_t = 1;
g = 9.81;
I = (m*l^2)/8 + m_p*l_p^2;
Tmax = 80;
tau = 0.05; s = tf('s'); tf_act = 1/(tau*s+1);
% tf_act = 1;
% c_d = 1;
% c_d = 0.5 * 1.225 * 0.32;
c_d = 4 * 1.225 * 0.32;

var_rnd = 6; % process noise variance
% var_rnd = 0;

m_p2 = 1;
l_p2 = 0.3;
c_t2 = 0.92;
I2 = (m*l^2)/8 + m_p2*l_p2^2;

ts = times(floor(length(times)/2));
switchind = find(times>=ts,1);

m_p_ts = timeseries([m_p * ones(switchind,1); m_p2 * ones(switchind+1,1)], times);
l_p_ts = timeseries([l_p * ones(switchind,1); l_p2 * ones(switchind+1,1)], times);
I_ts   = timeseries([I * ones(switchind,1); I2 * ones(switchind+1,1)], times);
c_t_ts = timeseries([c_t * ones(switchind,1); c_t2 * ones(switchind+1,1)], times);


x0 = xd(1,1);
z0 = zd(1,1);
th0 = -atan2(xd(1,3),zd(1,3)+g);

dx0 = xd(1,2);
dz0 = zd(1,2);

lambda = 3;
lambda_in = 6;
Gamma_out =  0.2 * diag([1, 1]);
Gamma = 2 * diag([1, 1]);

Phi_th = 0.1;
Phi_z  = 0.4;
Phi_x  = 0.4;

k_th = 0.2; % gain on PD part of control - makes adaptation harder
k_z = 6;
k_x = 6;

ahat_outer_t0 = 0.5*[(m+m_p); c_d];

ahat_in1 = 2*I/l;
ahat_in2 = -2*m_p*l_p/l;

ahat_inner_t0 = 0.5*[ahat_in1; ahat_in2];

%% simulation
options = simset('SrcWorkspace','current');
sim('sim_v2',[],options)

%% output
tracking_error = ( ex.^2 + ez.^2 ).^(1/2);

figure; plot(x(:,1), z(:,1)); hold on; plot(xd_ts.Data(:,1), zd_ts.Data(:,1)); title('Trajectory'); grid on;
scatter(x(1,1), z(1,1));
scatter(x(end,1), z(end,1));
scatter(xd_ts.Data(switchind,1), zd_ts.Data(switchind,1));

% obstacles = [5,60,5,5;
%              13,45,5,5;
%              29,0,5,25;
%              29,40,5,25;
%              29,65,35,12;
%              44,20,20,25;
%              75,50,3,30;
%              70,92,15,15;
%              80,18,5,5;
%              88,38,5,5;
%              83,66,5,5;
%              0,95,50,5;
%              93,93,10,3];
% for ii=1:size(obstacles,1)
%     obs = obstacles(ii,:);
%     a = obs(1:2);
%     b = obs(1:2) + [obs(3), 0];
%     c = obs(1:2) + [obs(3), obs(4)];
%     d = obs(1:2) + [0, obs(4)];
%     p = [a;b;c;d]; 
%     fill(p(:,1),p(:,2), zeros(4,1), 'FaceColor', [0.99, 0.92, 0.8] , 'EdgeColor', [.93,.69,.13]);
% %     plot_quad(a,b,c,d); %front
% end

figure; 
subplot(4,1,1); plot(tout, TLf); hold on; plot(tout, TRf); grid on; legend('T_{L,f}', 'T_{R,f}');

subplot(4,1,2); plot(tout, tracking_error); title('Tracking error');grid on;

subplot(4,1,3); plot(tout, ahat_outer(:,1)); hold on; plot(tout, ahat_outer(:,2)); plot(tout, ahat_inner(1,:)); plot(tout, ahat_inner(2,:)); title('Adaptive parameters'); grid on;

subplot(4,1,4); plot(tout, sth); hold on; plot(tout, sz); plot(tout, sx), title('s'); legend('s_\theta','s_z','s_x'); grid on;


%{
figure; subplot(3,2,1); plot(x(:,1), z(:,1)); hold on; plot(xd_ts.Data(:,1), zd_ts.Data(:,1)); title('Trajectory'); grid on;

% subplot(3,2,2); plot(tout,u1,'-.'); hold on; plot(tout,u2,'-.'); plot(tout, u1f); plot(tout, u2f); title('Control inputs'); grid on;
subplot(3,2,2); plot(tout, TLf); hold on; plot(tout, TRf); grid on; legend('T_{L,f}', 'T_{R,f}');

subplot(3,2,3); plot(tout, ahat_outer); hold on; plot(tout, ahat_inner(1,:)); plot(tout, ahat_inner(2,:)); title('Adaptive parameters'); grid on;

subplot(3,2,4); plot(tout, sth); hold on; plot(tout, sz); plot(tout, sx), title('s'); legend('s_\theta','s_z','s_x'); grid on;

subplot(3,2,5); plot(tout, sth_del); hold on; plot(tout, sz_del); plot(tout, sx_del); title('s_\delta'); legend('s_{\Delta \theta}','s_{\Delta z}','s_{\Delta x}'); grid on;

subplot(3,2,6); plot(tout, tracking_error); title('Tracking error');grid on;
%}
% figure; plot(tout, TLf); hold on; plot(tout, TRf); grid on; legend('T_{L,f}', 'T_{R,f}');
