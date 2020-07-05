% bezier curve test program

clear;
clc;

pt1 = [ 5; -10];
pt2 = [45; 15];

placelabel(pt1,'pt_1');
placelabel(pt2,'pt_2');
xlim([0 50])
axis equal

t = linspace(0, 1, 101);

pts = kron((1-t),pt1) + kron(t,pt2);

hold on
plot(pts(1,:),pts(2,:))
hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cla
line([pt1(1), pt2(1)],[pt1(2), pt2(2)])

pt1 = [ 5;-10];
pt2 = [ 5; 10];
pt3 = [25; 10];

line1_x = [pt1(1), pt2(1)];
line1_y = [pt1(2), pt2(2)];

line2_x = [pt2(1), pt3(1)];
line2_y = [pt2(2), pt3(2)];

cla
placelabel(pt1,'pt_1');
placelabel(pt2,'pt_2');
placelabel(pt3,'pt_3');
xlim([0 50])
axis equal
grid on

pts = kron((1-t).^2,pt1) + kron(2*(1-t).*t,pt2) + kron(t.^2,pt3);

curve_length_all = 0;

for i = 1 : length(t)-1
    
    curve_length = sqrt((pts(1,i) - pts(1, i+1))^2 + (pts(2, i) - pts(2, i+1))^2);
    
    curve_length_all = curve_length_all + curve_length;
    
end

curve_length_all
%length = int(kron((1-t).^2,pt1) + kron(2*(1-t).*t,pt2) + kron(t.^2,pt3), t, t(1), t(end))

figure(1)
hold on
plot(pts(1,:),pts(2,:));

plot(line1_x, line1_y, 'r');
plot(line2_x, line2_y, 'r');


















