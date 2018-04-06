% clear
% clc

% sliding_controller_2d;
% sliding_controller_hierarchical_v4;

% make an animation of simulation
% figure('Position',[100,100,480,425])
figure(1)
F(length(t)) = struct('cdata',[],'colormap',[]);
filename = 'example2.gif';

L_sim = 3*L;
l_sim = 3*l;
% quad_len = 0;

obstacles = [5,60,5,5;
             13,45,5,5;
             31,0,5,25;
             31,40,5,25;
             31,65,35,12;
             44,20,20,25;
             75,50,3,30;
             70,92,15,15;
             80,18,5,5;
             88,38,5,5;
             83,66,5,5;
             0,95,50,5;
             93,93,10,3];

plot(t_ws,u_ws1(:)); hold on
plot(t_ws,u_ws2(:))

for i = 1:10:length(t)
    u_ind = find(t_ws>=t(i), 1);
    ur = 0.5 * (u_ws1(max([1,u_ind-1]):u_ind+1) + u_ws2(max([1,u_ind-1]):u_ind+1));
    ul = 0.5 * (u_ws1(max([1,u_ind-1]):u_ind+1) - u_ws2(max([1,u_ind-1]):u_ind+1));
    % scatter(x,y, marker size, colors)
%     center_x = out(i,8);
    center_x = out(i,7);
    left_x = center_x - L_sim*cos(out(i,3));
    right_x = center_x + L_sim*cos(out(i,3));
    
    center_z = out(i,1);
    left_z = center_z - L_sim*sin(out(i,3));
    right_z = center_z + L_sim*sin(out(i,3));
    
    payload_x = center_x + l_sim*sin(out(i,3));
    payload_z = center_z - l_sim*cos(out(i,3));
    
    X = [left_x; right_x];
    Z = [left_z; right_z];

%     quad_len(end+1) = sqrt((right_x-left_x)^2 + (right_z-left_z)^2);
%     subplot(1,2,1)
%     plot(out(1:i,8), out(1:i,1));     
    pbaspect([1 1 1])
    plot(out(1:i,7), out(1:i,1),'LineWidth',1.5);     
    hold on; grid on;
    plot(X_d(:,2),X_d(:,1), '--')
    
    for ii=1:size(obstacles,1)
        obs = obstacles(ii,:);
        a = obs(1:2);
        b = obs(1:2) + [obs(3), 0];
        c = obs(1:2) + [obs(3), obs(4)];
        d = obs(1:2) + [0, obs(4)];
        plot_quad(a,b,c,d); %front
    end
    
    scatter(center_x,center_z,48,2)
    scatter(payload_x,payload_z,24,2,'MarkerFaceColor',[0.47, 0.67, 0.19],'MarkerEdgeColor',[0.47, 0.67, 0.19])
    %ezplot(xRef,zRef,[0,tf]);
    plot(X,Z,'linewidth',1,'color',[0.0, 0.45, 0.74])
    plot([center_x, payload_x],[center_z, payload_z],'linewidth',1,'color',[0.47, 0.67, 0.19])
    scatter(left_x,left_z,24,1,'MarkerFaceColor',[0.0, 0.45, 0.74],'MarkerEdgeColor',[0.0, 0.45, 0.74])
    scatter(right_x,right_z,24,1,'MarkerFaceColor',[0.0, 0.45, 0.74],'MarkerEdgeColor',[0.0, 0.45, 0.74])
    hold off
    xlabel('x (m)')
    ylabel('z (m)')
    title(['Tandem rotor motion at t = ' num2str(t(i)) 's']);
%     ylim([0, max(out(:,1))]);
    ylim([0, 120]);
%     xlim([min(out(:,8)), max(out(:,8))]);
%     xlim([min(out(:,7)), max(out(:,7))]);
    xlim([-10, 110]);
    set(gcf,'color','w'); % set figure background to white
    drawnow;
%     
%     subplot(1,2,2)
% %     c = categorical({'u_L(t)', 'u_R(t)'});
%     bar([ul; ur]);
%     ylim([0 30]);
%     title(['Left and right thrusts at t = ' num2str(t_ws(u_ind),'%.1f') 's']);
%     drawnow;
%     
    F(i) = getframe(1); % save scatter plot as a frame for animation
    
%     % save animation to .gif file
    im = frame2im(F(i));
    [A,map] = rgb2ind(im,256); 
    if i == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.02);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.02);
    end
end

function [] = plot_quad(a,b,c,d)    
    p = [a;b;c;d]; 
    fill(p(:,1),p(:,2), zeros(4,1), 'FaceColor', [0.99, 0.92, 0.8] , 'EdgeColor', [.93,.69,.13]);
end