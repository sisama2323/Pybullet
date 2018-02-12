function TAU = mytorqfun(bot,t,q,dq);
G=gravity(q(1),q(2),q(3));
% G=bot.gravload(q);
% c=bot.coriolis(q,dq)*dq';
% D=bot.inertia(q)
c=centrifugal(q(1),q(2),q(3),dq(1),dq(2),dq(3));

%%%%%%%%%%%%%%%%%%% torque comp for the payload
t1=sym('t1','real');
t2=sym('t2','real');
t3=sym('t3','real');

l1=0;l2=0.2;l3=0.2;

P=(sin(t2+t3)*l3+sin(t2)*l2)*3*9.8;  

g1=diff(P,t1);
g2=diff(P,t2);
g3=diff(P,t3);
Gcomp=[g1;g2;g3];

t1=q(1);t2=q(2);t3=q(3);

for j=1:3
        try
            Gcomp(j)=double(subs(Gcomp(j)));
        catch
            Gcomp(j)=0;
        end
end
Gcomp=double(Gcomp);

TAU=c'; % assume smooth movement 