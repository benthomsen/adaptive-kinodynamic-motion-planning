function [A, B] = linstate12d(x_i)
    u_i = [0; 0; 0; 0.1855; 0; 0; -0.0399];
%             u_i = zeros(7,1);
    if (size(x_i,1) == 1) && (size(x_i,2) ~= 1)
        x_i = x_i';
    end

    x_i_mod = [x_i(4:6); x_i(1:3); x_i(7:end)];
    thresh	=	[.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1;.1];
    xj		=	[x_i_mod; u_i];
    ti      =   0;
    xdotj		=	LinModel(ti,xj);
    [dFdX,~]	=	numjac('LinModel',ti,xj,xdotj,thresh,[],0);
    Fmodel		=	dFdX(1:12,1:12);
    Gmodel		=	dFdX(1:12,13:19);

    c123 = Fmodel(:,1:3); c456 = Fmodel(:,4:6); Fmodel2 = [c456, c123, Fmodel(:,7:end)];
    r123 = Fmodel2(1:3,:); r456 = Fmodel2(4:6,:); 
    A = [r456; r123; Fmodel2(7:end,:)];

    Gr123 = Gmodel(1:3,1:4);
    Gr456 = Gmodel(4:6,1:4);
%             B_allu = [Gr456; Gr123; Gmodel(7:end,:)];
%             c = B_allu * u0; % trim condition
    B = [Gr456; Gr123; Gmodel(7:end,1:4)];
end