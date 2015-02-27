% x1 = dlmread('ode.dat',' ');
x2 = dlmread('data2/sus.dat',' ');
x3 = dlmread('data2/imp.dat',' ');

x1 = zeros(size(x3));

% max_r = 5;
% min_r = 4.1;
% 
% x1 = x1(x1(:,3)<=max_r & x1(:,3)>=min_r,:);
% x2 = x2(x2(:,3)<=max_r & x2(:,3)>=min_r,:);
% x3 = x3(x3(:,3)<=max_r & x3(:,3)>=min_r,:);
clf;
% plot(x1(:,5),x1(:,6),'bo','MarkerSize',5);
% hold on;
% plot(x2(:,5),x2(:,6),'ro','MarkerSize',5);
% plot(x3(:,5),x3(:,6),'go','MarkerSize',5);
% scatter(x2(:,3).*cos(x2(:,4)),x2(:,3).*sin(x2(:,4)),'ok');
% axis([-5 5 -5 5 0 2])
hold on;
% plot3(x1(:,5),x1(:,6),x1(:,2),'b.','MarkerSize',5);
plot3(x2(:,5),x2(:,6),x2(:,2),'r.','MarkerSize',5);
plot3(x3(:,5),x3(:,6),x3(:,2),'g.','MarkerSize',5);
scatter3(x2(:,3).*cos(x2(:,4)),x2(:,3).*sin(x2(:,4)),zeros(size(x2(:,3))),'ok');

title('Contact velocity of ball over 2 seconds')
ylabel('y velocity');
xlabel('x velocity');
zlabel('time (seconds)')
legend('Sustained','Impulse (nk=16)','Initial Vel')