close all;
r1l=0.12;
r2l=0.23;
r4l=0.36;
r3l=0.23;
r5l=0.36;
m=2;%2kg of mass for the robot.
V(2)=zeros();
H(2)=zeros();
theta2v(2)=zeros();
theta3v(2)=zeros();
theta4v(2)=zeros();
theta5v(2)=zeros();
T=35*9.81/100;%convert 35kgcm torque to Nm

%Define time increment to be 0.01;
dt=0.001;

theta2=0;
%Calculate theta 3 by relation that theta 3 = 180 degree + theta 2 as
%defined in constrains
theta3=theta2+pi;
%Solve for theta2 theta3 theta 4 theta 5
%Since hoint 45 should remain in the middle during the hymp, thus
%cos(theta2)r2l+cos(theta4)r4l+6=0; Solve for theta 4
theta4=acos((-0.06-(cos(theta2)*r2l))/r4l);
%cos(theta3)r3l+cos(theta5)r5l-6=0; Solve for theta 5
theta5=acos((0.06-(cos(theta3)*r3l))/r5l);
H(1)=sin(theta4)*r4l;

i=1;
while theta2<pi/3
RForce=sin(theta4-theta2-pi/2)*r2l;
%Torque Balance on Element 2
F24=T/RForce;
Fy=F24*sin(theta4);
Fy=Fy*2;
a=(Fy-m*9.81)/m;
%Observe that the system is always in symetry, and F45x=F54x, thus in the y
%direction, Fytotal to Fy*2
V(i+1)=V(i)+a*dt;
H(i+1)=H(i)+V(i)*dt;
%Store Angles in the vectors
theta2v(i)=theta2;
theta3v(i)=theta3;
theta4v(i)=theta4;
theta5v(i)=theta5;

%Compute new theta 2 and 3 angles with new height.
syms x y;
eqn1 = sin(x)*r2l+sin(y)*r4l == H(i+1);
eqn2 = cos(x)*r2l+cos(y)*r4l == -0.06;

[theta2Sims,theta4Sims]=solve([eqn1,eqn2],[x,y]);
%compare with the previous result to see consistency, choose the one that
%causes the least change in angle
if abs(double(theta2Sims(2))-theta2v(i))>abs(double(theta2Sims(1))-theta2v(i))
theta2=double(theta2Sims(1));
else
theta2=double(theta2Sims(2));
end

if abs(double(theta4Sims(2))-theta4v(i))>abs(double(theta4Sims(1))-theta4v(i))
theta4=double(theta4Sims(1));
else
theta4=double(theta4Sims(2));
end

%Verify angular velocity within accepted velocity.
AngV=(theta2-theta2v(i))/dt;
%Angular velocity 0.11sec/60deg = (pi/3rad)/0.11sec
if AngV>((pi/3)/0.11)
%If angular velocity exceeds, force correlare theta 2 new using angular
%velocity.
theta2=theta2v(i)+dt*(pi/3)/0.11;
theta4=acos((-0.06-(cos(theta2)*r2l))/r4l);
T=0;
disp('Force Correlation')
else
T=35*9.81/100;
end
i=i+1;
end

theta2v(i)=theta2;
figure(1);
plot(theta2v,H,'r',theta2v,V,'b');
legend('Height of Bot', 'Velocity of Bot');
xlabel('Theta2(Rad)');
ylabel('Vertical Velocity(m/s) / Height of Mass M of Robot(m)')
[MaxDepartVelocity,index]=max(V);
MaxDepartVelocity=MaxDepartVelocity
HeightofMassatDept=H(index)