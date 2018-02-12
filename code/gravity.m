function G=gravity(T1,T2,T3);
t1=sym('t1','real');
t2=sym('t2','real');
t3=sym('t3','real');

g=9.81;
d=10^4;   %density
l1=0;l2=0.2;l3=0.2;
r1=0.1;r2=0.02;r3=0.02;

r=[r1;r2;r3];    % radius of link
l=[l1;l2;l3];     % length of link
m=pi*r.^2.*l.*d;    % mass of link d is the density
P=sin(t2)*l2/2*m(2)*g+(sin(t2+t3)*l3/2+sin(t2)*l2)*m(3)*g;  

g1=diff(P,t1);
g2=diff(P,t2);
g3=diff(P,t3);
G=[g1;g2;g3];

t1=T1;t2=T2;t3=T3;

for j=1:3
        try
            G(j)=double(subs(G(j)));
        catch
            G(j)=0;
        end
end
G=double(G);

