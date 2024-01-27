
t = out.logsout{1}.Values.Time;
posx = squeeze(out.logsout{1}.Values.Data(:,1));
posy = squeeze(out.logsout{1}.Values.Data(:,2));
psi  = squeeze(out.logsout{1}.Values.Data(:,3));

% plotShip(t,posx,posy,psi,no_ship,color,alpha,Ecolor,EWidth)
plotShip(t,posx,posy,psi,50,'r',0.1,'k',1);