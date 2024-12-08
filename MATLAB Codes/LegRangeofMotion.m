close all;
r1l=0.12;
r2l=0.23;
r4l=0.36;
r3l=0.23;
r5l=0.36;
rw=0.0325;
m=2;%2kg of mass for the robot.
theta2v(2)=zeros();
theta3v(2)=zeros();
theta4v(2)=zeros();
theta5v(2)=zeros();

MList1(2)=zeros();
MList2(2)=zeros();
MList3(2)=zeros();

xT(2)=zeros();
yT(2)=zeros();
T=35*9.81/100;%convert 35kgcm torque to Nm

%Solve for theta2 theta3 theta 4 theta 5
%Fixing theta3
theta3=pi;
theta2=linspace(-pi/3,pi/3,19);

for i=1:1:19
syms x y; 
eqn1=cos(pi-x)*r4l-cos(y)*r5l==cos(theta3)*r3l-0.12-cos(theta2(i))*r2l;
eqn2=sin(pi-x)*r4l-sin(y)*r5l==-sin(theta2(i))*r2l+sin(theta3)*r3l;
[theta4Sims,theta5Sims]=solve([eqn1,eqn2],[x,y]);
if double(theta4Sims(1))>0
theta4v(i)=pi-double(theta4Sims(1));
else
theta4v(i)=pi-double(theta4Sims(2));
end

if double(theta5Sims(1))>0
theta5v(i)=double(theta5Sims(1));
else
theta5v(i)=double(theta5Sims(2));
end

xT(i)=-0.06-cos(theta4v(i))*r4l-cos(theta2(i))*r2l;
yT(i)=-sin(theta4v(i))*r4l-sin(theta2(i))*r2l;
end
figure(1);  
set(1,'WindowStyle','Docked');
xlim([-0.4 0.4]);
ylim([-0.4 0.3]);

for i=1:1:19

MList1(i)=9.81*m*xT(i)*rw/(yT(i)+rw);

TwoFourx=-cos(theta2(i))*r2l-0.06;
TwoFoury=-sin(theta2(i))*r2l;
ThreeFivex=-cos(theta3)*r3l+0.06;
ThreeFivey=-sin(theta3)*r3l;

plot([0.06,-0.06],[0,0],'LineWidth',3);
xlim([-0.4 0.4]);
ylim([-0.4 0.3]);
hold on;
plot([-0.06,TwoFourx],[0,TwoFoury],'LineWidth',3);
plot([TwoFourx,xT(i)],[TwoFoury,yT(i)],'LineWidth',3);
plot([0.06,ThreeFivex],[0,ThreeFivey],'LineWidth',3);
plot([ThreeFivex,xT(i)],[ThreeFivey,yT(i)],'LineWidth',3);

plot(0.06, 0, 'bo','MarkerFaceColor','w');
plot(-0.06, 0, 'bo','MarkerFaceColor','w');

plot(TwoFourx, TwoFoury, 'bo','MarkerFaceColor','w');

plot(ThreeFivex, ThreeFivey, 'bo','MarkerFaceColor','w');

plot(xT(i), yT(i), 'bo','MarkerFaceColor','w');
hold off;

if i==1 || i==10 || i==19
pause(1);
else
pause(0.1);
end

end
MaxAccelTorque=max(MList1)
MaxDecelTorque=min(MList1)

LeftLimit=min(xT)
RightLimit=max(xT)

%Solve for theta2 theta3 theta 4 theta 5
%Fixing theta2

theta3=linspace(4*pi/3,2*pi/3,19);
theta2=0;

for i=1:1:19
syms x y; 
eqn1=cos(pi-x)*r4l-cos(y)*r5l==cos(theta3(i))*r3l-0.12-cos(theta2)*r2l;
eqn2=sin(pi-x)*r4l-sin(y)*r5l==-sin(theta2)*r2l+sin(theta3(i))*r3l;
[theta4Sims,theta5Sims]=solve([eqn1,eqn2],[x,y]);
if double(theta4Sims(1))>0
theta4v(i)=pi-double(theta4Sims(1));
else
theta4v(i)=pi-double(theta4Sims(2));
end

if double(theta5Sims(1))>0
theta5v(i)=double(theta5Sims(1));
else
theta5v(i)=double(theta5Sims(2));
end

xT(i)=-0.06-cos(theta4v(i))*r4l-cos(theta2)*r2l;
yT(i)=-sin(theta4v(i))*r4l-sin(theta2)*r2l;
end
figure(2);  
set(2,'WindowStyle','Docked');
xlim([-0.4 0.4]);
ylim([-0.4 0.3]);

for i=1:1:19
MList2(i)=9.81*m*xT(i)*rw/(yT(i)+rw);

TwoFourx=-cos(theta2)*r2l-0.06;
TwoFoury=-sin(theta2)*r2l;
ThreeFivex=-cos(theta3(i))*r3l+0.06;
ThreeFivey=-sin(theta3(i))*r3l;

plot([0.06,-0.06],[0,0],'LineWidth',3);
xlim([-0.4 0.4]);
ylim([-0.4 0.3]);
hold on;
plot([-0.06,TwoFourx],[0,TwoFoury],'LineWidth',3);
plot([TwoFourx,xT(i)],[TwoFoury,yT(i)],'LineWidth',3);
plot([0.06,ThreeFivex],[0,ThreeFivey],'LineWidth',3);
plot([ThreeFivex,xT(i)],[ThreeFivey,yT(i)],'LineWidth',3);

plot(0.06, 0, 'bo','MarkerFaceColor','w');
plot(-0.06, 0, 'bo','MarkerFaceColor','w');

plot(TwoFourx, TwoFoury, 'bo','MarkerFaceColor','w');

plot(ThreeFivex, ThreeFivey, 'bo','MarkerFaceColor','w');

plot(xT(i), yT(i), 'bo','MarkerFaceColor','w');
hold off;

if i==1 || i==10 || i==19
pause(1);
else
pause(0.1);
end

end

MaxAccelTorque=max(MList2)
MaxDecelTorque=min(MList2)

LeftLimit=min(xT)
RightLimit=max(xT)
%Solve for theta2 theta3 theta 4 theta 5
% Constant Relation


theta2=linspace(-pi/3,pi/3,19);
theta3=linspace(2*pi/3,4*pi/3,19);

for i=1:1:19
syms x y; 
eqn1=cos(pi-x)*r4l-cos(y)*r5l==cos(theta3(i))*r3l-0.12-cos(theta2(i))*r2l;
eqn2=sin(pi-x)*r4l-sin(y)*r5l==-sin(theta2(i))*r2l+sin(theta3(i))*r3l;
[theta4Sims,theta5Sims]=solve([eqn1,eqn2],[x,y]);
if double(theta4Sims(1))>0
theta4v(i)=pi-double(theta4Sims(1));
else
theta4v(i)=pi-double(theta4Sims(2));
end

if double(theta5Sims(1))>0
theta5v(i)=double(theta5Sims(1));
else
theta5v(i)=double(theta5Sims(2));
end

xT(i)=-0.06-cos(theta4v(i))*r4l-cos(theta2(i))*r2l;
yT(i)=-sin(theta4v(i))*r4l-sin(theta2(i))*r2l;
end
figure(3);  
set(3,'WindowStyle','Docked');
xlim([-0.4 0.4]);
ylim([-0.4 0.3]);

for i=1:1:19
MList3(i)=9.81*m*xT(i)*rw/(yT(i)+rw);
TwoFourx=-cos(theta2(i))*r2l-0.06;
TwoFoury=-sin(theta2(i))*r2l;
ThreeFivex=-cos(theta3(i))*r3l+0.06;
ThreeFivey=-sin(theta3(i))*r3l;

plot([0.06,-0.06],[0,0],'LineWidth',3);
xlim([-0.4 0.4]);
ylim([-0.4 0.3]);
hold on;
plot([-0.06,TwoFourx],[0,TwoFoury],'LineWidth',3);
plot([TwoFourx,xT(i)],[TwoFoury,yT(i)],'LineWidth',3);
plot([0.06,ThreeFivex],[0,ThreeFivey],'LineWidth',3);
plot([ThreeFivex,xT(i)],[ThreeFivey,yT(i)],'LineWidth',3);

plot(0.06, 0, 'bo','MarkerFaceColor','w');
plot(-0.06, 0, 'bo','MarkerFaceColor','w');

plot(TwoFourx, TwoFoury, 'bo','MarkerFaceColor','w');

plot(ThreeFivex, ThreeFivey, 'bo','MarkerFaceColor','w');

plot(xT(i), yT(i), 'bo','MarkerFaceColor','w');
hold off;

if i==1 || i==10 || i==19
pause(1);
else
pause(0.1);
end

end

MaxAccelTorque=max(MList3)
MaxDecelTorque=min(MList3)

LeftLimit=min(xT)
RightLimit=max(xT)