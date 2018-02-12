function [DD]=inertia(T1,T2,T3);
t1=sym('t1','real');
t2=sym('t2','real');
t3=sym('t3','real');

g=9.81;
d=10^4;   %density
l1=0;l2=0.2;l3=0.2;
r1=0.1;r2=0.02;r3=0.02;

q=[t1;t2;t3];       % angle of each link
r=[r1;r2;r3];    % radius of link
l=[l1;l2;l3];     % length of link
m=pi*r.^2.*l.*d;    % mass of link d is the density

I3=[1/2*m(3)*r3^2,0,0;
    0,1/3*m(3)*l3^2+1/4*m(3)*r3^2,0;
    0,0,1/3*m(3)*l3^2+1/4*m(3)*r3^2];

I2=[1/2*m(2)*r2^2,0,0;
    0,1/3*m(2)*l2^2+1/4*m(2)*r2^2,0;
    0,0,1/3*m(2)*l2^2+1/4*m(2)*r2^2];

I1=[0,0,0;0,0,0;0,0,0];

H23=[cos(t3), sin(t3), 0, l2;
            -sin(t3), cos(t3), 0, 0;
            0, 0, 1, 0;
            0, 0, 0, 1];
H2=[1,0,0,0;
    0,cosd(90),-sind(90),0;
    0,sind(90),cosd(90),0;
    0,0,0,1];

H12=[cos(t2),sin(t2), 0, 0;
         -sin(t2), cos(t2), 0, 0;
         0, 0, 1, 0;
         0, 0, 0, 1];

H12=H2*H12;

H01=[cos(t1), sin(t1), 0, 0;
         -sin(t1),cos(t1),0,0;
         0, 0, 1, 0;
         0,0,0,1];
     
% rotation matrix
R03=H01(1:3,1:3)*H12(1:3,1:3)*H23(1:3,1:3);
R02=H01(1:3,1:3)*H12(1:3,1:3);
R01=H01(1:3,1:3);

P3=[l3/2;0;0;1];
P2=[l2/2;0;0;1];
P1=[0;0;0;1];

P0_3 = H01*H12*H23*P3;
P0_2 = H01*H12*P2;
P0_1 = H01*P1;

% tx3=atan2(R03(3,2),R03(3,3));
% ty3=atan2(-R03(3,1),(R03(2,1)^2+R03(1,1)^2)^(1/2));
% tz3=atan2(R03(2,1),R03(1,1));
% 
% tx2=atan2(R02(3,2),R02(3,3));
% ty2=atan2(-R02(3,1),(R02(2,1)^2+R02(1,1)^2)^(1/2));
% tz2=atan2(R02(2,1),R02(1,1));
% 
% tx1=atan2(R01(3,2),R01(3,3));
% ty1=atan2(-R01(3,1),(R01(2,1)^2+R01(1,1)^2)^(1/2));
% tz1=atan2(R01(2,1),R01(1,1));
% 
% f_3 = [P0_3(1:3);tx3;ty3;tz3];
% f_2 = [P0_2(1:3);tx2;ty2;tz2];
% f_1 = [P0_1(1:3);tx1;ty1;tz1];

% J1 = [diff(f_1,t1),diff(f_1,t2),diff(f_1,t3)];    % diff of f via theta1
% J2 = [diff(f_2,t1),diff(f_2,t2),diff(f_2,t3)];      % Jacobian
% J3 = [diff(f_3,t1),diff(f_3,t2),diff(f_3,t3)];  

f_3 = [P0_3(1:3);0;0;t1+t2+t3];
f_2 = [P0_2(1:3);0;0;t1+t2];
f_1 = [P0_1(1:3);0;0;t1];

J1 = [diff(f_1,t1),diff(f_1,t2),diff(f_1,t3)];    % diff of f via theta1
J2 = [diff(f_2,t1),diff(f_2,t2),diff(f_2,t3)];      % Jacobian
J3 = [diff(f_3,t1),diff(f_3,t2),diff(f_3,t3)];  

d1=m(1)*J1(1:3,:)'*J1(1:3,:)+J1(4:6,:)'*R01*I1*R01'*J1(4:6,:);
d2=m(2)*J2(1:3,:)'*J2(1:3,:)+J2(4:6,:)'*R02*I2*R02'*J2(4:6,:);
d3=m(3)*J3(1:3,:)'*J3(1:3,:)+J3(4:6,:)'*R03*I3*R03'*J3(4:6,:);
D=d1+d2+d3;

t1=T1;t2=T2;t3=T3;

DD=[];
for i=1:3
    for j=1:3
        try
            DD(i,j)=double(subs(D(i,j)));
        catch
            DD(i,j)=0;
        end
    end
end



