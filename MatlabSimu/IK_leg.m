function Q=IK_leg(foot,side)
%foot=[foot_x,foot_y,foot_z]; coordenadas del pie con respecto al SC0 de la
%side=1 para pierna izquierda; side=0 para pierna derecha
%pierna
if side==1
    flag=1;
elseif side==0
    flag=-1;
else
    a='Flag error'
end
ds=45; %Distancia del hombro
l1=100;
l2=100;
%IK pierna frontal izquierda
d=sqrt(((foot(1))^2)+((foot(2))^2));
h=sqrt((d^2)-(ds^2));

alpha=atan2(h,ds);
betha=atan2(foot(2),foot(1));
q1=alpha+betha;

r=sqrt((h^2)+((foot(3))^2));

P=((r^2)-(l1^2)-(l2^2))/(2*l1*l2);
q3=atan2(flag*sqrt(1-(P^2)),P);

gamma=atan2(-foot(3),h);
sigma=atan2((l2*sin(q3)),(l1+(l2*cos(q3))));
q2=gamma+sigma;

Q=[q1;q2;q3];