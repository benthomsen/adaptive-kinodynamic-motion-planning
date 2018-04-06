function [A, B] = Ross_linstate(x_i)
    
    if (size(x_i,1) == 1) && (size(x_i,2) ~= 1)
        x_i = x_i';
    end
    x_dim = size(x_i,1);
    u_dim = 3;
    u_i = [0; 0; 0]; % should this be trim condition?

    thresh	=	0.1*ones(x_dim+u_dim,1); % how low should thresh be?
    xj		=	[x_i; u_i];
    ti      =   0;
    xdotj		=	RossModel(ti,xj);
    [dFdX,~]	=	numjac('RossModel',ti,xj,xdotj,thresh,[],0);
    A           =	dFdX(1:x_dim,1:x_dim);
    B           =	dFdX(1:x_dim,x_dim+1:x_dim+u_dim);
end