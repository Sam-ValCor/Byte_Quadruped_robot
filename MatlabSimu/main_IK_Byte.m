close all
clear all 

%Traslación deseada
dx=0;
dy=0;
dz=-10;
%Orientación deseada
roll=deg2rad(0);
pitch=deg2rad(0);
yaw=deg2rad(0);

%Pose deseada
pose_d=[dx;dy;dz;roll;pitch;yaw];

% Posición del pie izquierdo frontal
foot_lf=[100;70;0;1]; %uno para que sea homogenea
foot_rf=[100;-70;0;1]; %uno para que sea homogenea
foot_lb=[-100;70;0;1]; %uno para que sea homogenea
foot_rb=[-100;-70;0;1]; %uno para que sea homogenea
figure; %Graficamos el origen del mundo y la posición deseada del pie
plot3(0,0,0,'*k'); %origen w=mundo
xlim([-250,250]);
ylim([-250,250]);
zlim([0,220]);
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
% view(179.99,180); %Vista lateral
% view(90,0); %Vista Frontal
% view(180,0); %Vista lateral positiva
grid on
hold on
plot3(foot_lf(1),foot_lf(2),foot_lf(3),'dg');
plot3(foot_rf(1),foot_rf(2),foot_rf(3),'dg');
plot3(foot_lb(1),foot_lb(2),foot_lb(3),'dg');
plot3(foot_rb(1),foot_rb(2),foot_rb(3),'dg');

 Q_calc=calc_ik_byte_main(pose_d,foot_lf,foot_rf,foot_lb,foot_rb);

%Pierna frontal izquierda
% q1_lf=deg2rad(0); %motor roll left front
% q2_lf=deg2rad(0); %motor pitch1 left front
% q3_lf=deg2rad(90); %motor pitch2 left front
% %Angulos pierna izquierda frontal calulados
q1_lf=Q_calc(1); %motor roll left front
q2_lf=Q_calc(2); %motor pitch1 left front
q3_lf=Q_calc(3); %motor pitch2 left front
q_lf=[q1_lf;q2_lf;q3_lf];

%Pierna frontal derecha
% q1_rf=deg2rad(0); %motor roll right front
% q2_rf=deg2rad(0); %motor pitch1 right front
% q3_rf=deg2rad(0); %motor pitch2 right front
%Angulos pierna derecha frontal calulados
q1_rf=Q_calc(4); %motor roll right front
q2_rf=Q_calc(5); %motor pitch1 right front
q3_rf=Q_calc(6); %motor pitch2 right front
q_rf=[q1_rf;q2_rf;q3_rf];

%Pierna trasera izquierda
% q1_lb=deg2rad(0); %motor roll left back
% q2_lb=deg2rad(0); %motor pitch1 left back
% q3_lb=deg2rad(0); %motor pitch2 left back
%Angulos pierna izquierda frontal calulados
q1_lb=Q_calc(7); %motor roll left back
q2_lb=Q_calc(8); %motor pitch1 left back
q3_lb=Q_calc(9); %motor pitch2 left back
q_lb=[q1_lb;q2_lb;q3_lb];

%Pierna trasera derecha
% q1_rb=deg2rad(0); %motor roll right back
% q2_rb=deg2rad(0); %motor pitch1 right back
% q3_rb=deg2rad(0); %motor pitch2 right back
%Angulos pierna derecha frontal calulados
q1_rb=Q_calc(10); %motor roll right back
q2_rb=Q_calc(11); %motor pitch1 right back
q3_rb=Q_calc(12); %motor pitch2 right back
q_rb=[q1_rb;q2_rb;q3_rb];

%Configuración deseada
Q_d=[q_lf;q_rf;q_lb;q_rb];

draw_byte(pose_d,Q_d);
