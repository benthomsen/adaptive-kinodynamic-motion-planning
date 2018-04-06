function xdot = RossEoM(t,X)
    global u

%         State Vector:
%         X(1) = x-position
%         X(2) = y-position
%         X(3) = z-position
%         X(4) = heading angle (xy plane)
%         X(5) = planar velocity
%         X(6) = curvature

%         Input Vector:
%         U(1) = curvature change rate
%         U(2) = z-velocity
%         U(3) = planar acceleration

%     x       = X(1);
%     y       = X(2);
%     z       = X(3);

    theta	= X(4);
    v       = X(5);
    curvature = X(6);
    
    x_dot   = v*cos(theta);
    y_dot   = v*sin(theta);
    z_dot   = u(2);
    th_dot  = v*curvature;
    v_dot   = u(3);
    cur_dot = u(1);
    
    xdot	= [x_dot; y_dot; z_dot; th_dot; v_dot; cur_dot];
end