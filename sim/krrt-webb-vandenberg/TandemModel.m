function xdotj = TandemModel(tj,xj)
    
    global u    

	x		=	xj(1:6);
	u		=	xj(7:8);

	xdot	=	TandemEoM(tj,x);
	xdotj	=	[xdot;0;0];
    
end