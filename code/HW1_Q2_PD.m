d=10^4;   %density
l1=0;l2=0.2;l3=0.2;
r1=0.1;r2=0.02;r3=0.02;
r=[r1;r2;r3];    % radius of link
l=[l1;l2;l3]; 
m=pi*r.^2.*l.*d;    % mass of link d is the density
m0=3;

I3=[1/2*m(3)*r3^2,0,0;
    0,1/12*m(3)*l3^2+1/4*m(3)*r3^2,0;
    0,0,1/12*m(3)*l3^2+1/4*m(3)*r3^2];

I3l=[1/2*m(3)*r3^2,0,0;
    0,1/12*m(3)*l3^2+1/4*m(3)*r3^2+m0*(l3/2)^2,0;
    0,0,1/12*m(3)*l3^2+1/4*m(3)*r3^2+m0*(l3/2)^2];

I2=[1/2*m(2)*r2^2,0,0;
    0,1/12*m(2)*l2^2+1/4*m(2)*r2^2,0;
    0,0,1/12*m(2)*l2^2+1/4*m(2)*r2^2];

R1=[0,0,0];
R2=[0.1,0,0];
R3l=[0.1+0.05441,0,0];

L1=Link('d',0,'a',0,'alpha',pi/2);
L2=Link('d',0,'a',0.2,'alpha',0,'r',R2,'m',2.5133,'I',I2);
L3=Link('d',0,'a',0.2,'alpha',0,'r',R2,'m',2.5133+3,'I',I3l);

botl=SerialLink([L1 L2 L3],'name','3');   % robot with payload

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% traj of moving from 1-2-3-4-1
T1=transl(0.2,0.2,0);   % starting point
T2=transl(0.1,0.2,0);  % destination
T3=transl(0.1,-0.2,0);
T4=transl(0.2,-0.2,0);

a1 = botl.ikunc(T1);          % moving from 1 -->2
a2 = botl.ikunc(T2);
a3 = botl.ikunc(T3);
a3=[a3(1),-a3(2),-a3(3)];
a4 = botl.ikunc(T4);
a4=[a4(1),-a4(2),-a4(3)];

dq0=[0,0,0];
dqdes=[0,0,0];


[T,q1,dq1]=botl.fdyn(20,@newtorqfun,a1,dq0,a2,dqdes);     %p1 is p start and p2 is p reference
[T,q2,dq2]=botl.fdyn(20,@newtorqfun,a2,dq0,a3,dqdes);
[T,q3,dq3]=botl.fdyn(20,@newtorqfun,a3,dq0,a4,dqdes);
[T,q4,dq4]=botl.fdyn(20,@newtorqfun,a4,dq0,a1,dqdes);

q=[q1;q2;q3;q4];
botl.plot(q);

T=botl.fkine(q);
X=[];
Y=[];
for i = 1: length(T(1,4,:))
    X=[X,T(1,4,i)];
    Y=[Y,T(2,4,i)];
end

figure
plot(X,Y)
xlabel('x')
ylabel('y')
title('trajectory of end effector')


figure
anim = Animate('Q2 f');
for i=1:length(q)
    botl.plot(q(i,:))
    title('Q2 f box')
    anim.add();
end

