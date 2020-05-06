clear
close all

r = 1;
angle = 45; 
%bottom point
po = [1;0.15;3.7];
% po = [0;0.15;3.7];
%center point
c_y = po(2)+r*cosd(angle);
d = sqrt((r)^2-(c_y-po(2))^2);
p = [po(1);c_y;d+po(3)];
%upper point
c_y = p(2)+r*cosd(angle);
d = sqrt(sqrt(r)-(c_y-p(2))^2);
p1 = [p(1);c_y;d+p(3)];


v1 = p1-p;
v1 = v1/norm(v1);
v2 = cross(p1-p,po-p);
v2 = v2/norm(v2);


% t = linspace(0,2*pi);
t = pi:0.1:4*pi;
for i=1:length(t)
    c(:,i) = p+r*cos(t(i))*v1+r*sin(t(i))*v2;
end
figure(1)
plot3(c(1,:),c(2,:),c(3,:))
hold on 
plot3(p(1),p(2),p(3),'o')


% t = linspace(0,2*pi);
x = r*cos(t)+p(1);
y = r*sin(t)+p(2);
z = 0*t+p(3);
figure(2)
plot3(x,y,z);
hold on 
plot3(p(1),p(2),p(3),'o')


% n = input('Enter a number: ');
% 

n = 'c';
switch n
    case 'c'
        disp('negative one')
    otherwise
        disp('other value')
end

% e = 35;
% 
% if e<=15
%     disp('hola hola hola')
% elseif 15<e && e<=30
%     dip('')
% else 
%     disp('lalalala')
% end