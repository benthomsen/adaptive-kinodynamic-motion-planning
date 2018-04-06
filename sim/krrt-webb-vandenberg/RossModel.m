function xdotj = RossModel(tj,xj)
    
    global u    

	x		=	xj(1:6);
	u		=	xj(7:9);

	xdot	=	RossEoM(tj,x);
	xdotj	=	[xdot;0;0;0];
    
end