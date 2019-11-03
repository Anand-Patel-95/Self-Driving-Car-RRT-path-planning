function h = circle2(x,y,r)
% hold on
pos = [(x-r) (y-r) 2*r 2*r]; 
h = rectangle('Position',pos,'Curvature',[1 1],'FaceColor','k');


% th = 0:pi/50:2*pi;
% xunit = r * cos(th) + x;
% yunit = r * sin(th) + y;
% h = plot(xunit, yunit);
% hold off