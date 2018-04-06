function [A, B] = linstate_tandem(x_i)

    global m m_p I l l_p g 
    m = 2.5;
    m_p = 1;
    l = 0.5;
    l_p = 0.25;
    g = 9.81;
    I = (m*l^2)/8 + m_p*l_p^2;

    if (size(x_i,1) == 1) && (size(x_i,2) ~= 1)
        x_i = x_i';
    end
    x_dim = size(x_i,1);
    u_dim = 2;
    u_i = [0; (m+m_p)*g/cos(x_i(3))]; % should this be trim condition?

    thresh	=	0.1*ones(x_dim+u_dim,1); % how low should thresh be?
    xj		=	[x_i; u_i];
    ti      =   0;
    xdotj		=	TandemModel(ti,xj);
    [dFdX,~]	=	numjac('TandemModel',ti,xj,xdotj,thresh,[],0);
    A           =	dFdX(1:x_dim,1:x_dim);
    B           =	dFdX(1:x_dim,x_dim+1:x_dim+u_dim);
end