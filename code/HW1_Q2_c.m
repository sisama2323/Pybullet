d=10^4;   %density
l1=0;l2=0.2;l3=0.2;
r1=0.1;r2=0.02;r3=0.02;
r=[r1;r2;r3];    % radius of link
l=[l1;l2;l3]; 
m=pi*r.^2.*l.*d;    % mass of link d is the density
m0=3;

I3=[1/2*m(3)*r3^2,0,0;
    0,1/3*m(3)*l3^2+2/4*m(3)*r3^2,0;
    0,0,1/3*m(3)*l3^2+1/4*m(3)*r3^2];

I2=[1/2*m(2)*r2^2,0,0;
    0,1/3*m(2)*l2^2+1/4*m(2)*r2^2,0;
    0,0,1/3*m(2)*l2^2+1/4*m(2)*r2^2];

I1=[0,0,0;0,0,0;0,0,0];

R1=[0,0,0];
R2=[0.1,0,0];
R3=[0.1,0,0];

L1=Link('d',0,'a',0,'alpha',0,'convention', 'modified');
L2=Link('d',0,'a',0,'alpha',pi/2,'r',R2,'m',2.5133,'I',I2, 'convention', 'modified');
L3=Link('d',0,'a',0.2,'alpha',0,'r',R3,'m',2.5133,'I',I3, 'convention', 'modified');
bot=SerialLink([L1 L2 L3],'name','2');

q0=[0,-pi/6,pi/3];
dq0=[0,0,0];

[T,q,qd]=bot.fdyn(5,@mytorqfun,q0,dq0);

%% part e
R3l=[0.1,0,0];
I3l=[1/2*m(3)*r3^2,0,0;
    0,1/3*m(3)*l3^2+1/4*m(3)*r3^2+m0*(l3/2)^2,0;
    0,0,1/3*m(3)*l3^2+1/4*m(3)*r3^2+m0*(l3/2)^2];
L3l=Link('d',0,'a',0.2,'alpha',0,'r',R3l,'m',2.5133+3,'I',I3l,'convention', 'modified');
botl=SerialLink([L1 L2 L3l],'name','3');
[T,q,qd]=botl.fdyn(10,@mytorqfun,q0,dq0);


%% display use
L4=Link('d',0,'a',0.2,'alpha',0,'m',0,'convention', 'modified');
botdis=SerialLink([L1 L2 L3 L4],'name','2');   % robot with 

zz=zeros(length(q),1);
qdis=[q,zz];
botdis.plot(qdis)

figure
anim = Animate('Q2 d with gravity compensation');
for i=1:length(qdis)
    botdis.plot(qdis(i,:))
    title('Q2 d with gravity compensation')
    anim.add();
end


