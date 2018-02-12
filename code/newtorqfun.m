function TAU = newtorqfun(botl,t,q,dq,qref,dqref);
G=botl.gravload(q);
c=botl.coriolis(q,dq)*dq';
% g=gravity(q(1),q(2),q(3));
TAU=G+0.1*(qref-q)+c'+0.5*(dqref-dq);
