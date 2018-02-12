startup_rvc

L1=Link([0,0,0.5,0,0],'alpha',pi/2);
L2=Link([0,0,0.5,0,0],'alpha',pi/2);
L3=Link([0,0,0.2,0,0],'alpha',pi/2);

bot=SerialLink([L1 L2 L3],'name','robot');

figure
bot.plot([0 0 pi/2])
title('Figure 1. \theta 1=0, \theta2=0,\theta 3=pi/2')

figure
bot.plot([0 pi 0])
title('Figure 2. \theta 1=0, \theta2=pi, \theta 3=0')

figure
bot.plot([pi/2 pi/2 pi/4])
title('Figure 3. \theta 1=pi/2, \theta2=pi/2,\theta 3=pi/4')

figure
bot.plot([pi/3 pi/2 0])
title('Figure 4. \theta 1=pi/3, \theta2=pi/2,\theta 3=0')

T1=transl(1.2,0,0);   % starting point
T2=transl(0.5,0.5,0);  % destination
T3=transl(0.5,-0.5,0);
T4=transl(0.4,0,0);

t1 = ctraj(T1,T2,50);   % in 50 steps
t2 = ctraj(T2,T3,50);
t3 = ctraj(T3,T4,50);
t4 = ctraj(T4,T1,50);


q0=[1.2,0,0];
q1 = bot.ikunc(t1,q0);
q2 = bot.ikunc(t2,q1(end,:));
q3 = bot.ikunc(t3,q2(end,:));
q4 = bot.ikunc(t4,q3(end,:));

q=vertcat(q1,q2,q3,q4);
T=bot.fkine(q);

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
title('path of end effector')

figure
anim = Animate('1st movie');
for i=1:length(q)
    bot.plot(q(i,:))
    anim.add();
end
% ffmpeg -r 60 -s 1920x1080 -i %04d.png out.avi
