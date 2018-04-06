function xdot = TandemEoM(t,X)
    global m m_p I l l_p g u

%         State Vector:
%         X(1) = x-position
%         X(2) = z-position
%         X(3) = pitch angle
%         X(4) = x-velocity
%         X(5) = z-velocity
%         X(6) = pitch rate

%         Input Vector:
%         U(1) = summed thrust
%         U(2) = differential thrust
        
    x          = X(1);
    z          = X(2);
    theta      = X(3);
    x_dot      = X(4);
    z_dot      = X(5);
    theta_dot  = X(6);

%     xdot        = zeros(6,1);
    
    theta_ddot  = u(2)*l/(2*I) + m_p*l_p*sin(theta)/I;
    x_ddot      = u(1)*sin(theta)/(m+m_p);
    z_ddot      = u(1)*cos(theta)/(m+m_p) - g;
    
    xdot        =	[x_dot; z_dot; theta_dot; x_ddot; z_ddot; theta_ddot];
end