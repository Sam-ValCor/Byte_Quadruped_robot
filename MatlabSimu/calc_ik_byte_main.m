function Q_d=calc_ik_byte_main(pose_d,foot_lf,foot_rf,foot_lb,foot_rb)
%pose=posición y orientación, pose_d=[dx;dy;dz;roll;pitch;yaw] (con respecto al mundo
%foot_ij=[foot_ijx;foot_ijy;foot_ijz;1] (con respecto al mundo)
D1=200; %Distancia entre ejes de pitch de las piernas
D2=90; %Distancia entre ejes de rollo de las piernas
Zh=120; %Distancia del piso en home

%Orientación deseada radianes
roll=pose_d(4);
pitch=pose_d(5);
yaw=pose_d(6);

R_roll=rota_x(roll);
R_pitch=rota_y(pitch);
R_yaw=rota_z(yaw);
Rr=R_roll*R_pitch*R_yaw;   

%Traslación deseada
dx=pose_d(1);
dy=pose_d(2);
dz=pose_d(3);
t_dz=dz+Zh;
Tr_x=tras_x(dx);
Tr_y=tras_y(dy);
Tr_z=tras_z(t_dz);
Tr=Tr_x*Tr_y*Tr_z;

%Transformación total deseada
Hw_r=Rr*Tr;

%Sistemas de referencia de las piernas con respecto al mundo

Hr_lf=tras_y(D2/2)*tras_x(D1/2)*rota_z(pi/2)*rota_x(pi/2);
Hr_rf=tras_y(-D2/2)*tras_x(D1/2)*rota_z(-pi/2)*rota_x(pi/2);
Hr_lb=tras_y(D2/2)*tras_x(-D1/2)*rota_z(pi/2)*rota_x(pi/2);
Hr_rb=tras_y(-D2/2)*tras_x(-D1/2)*rota_z(-pi/2)*rota_x(pi/2);

%Sistemas Transformados de las piernas-hombros
Hw_llf=Hw_r*Hr_lf;
Hw_lrf=Hw_r*Hr_rf;
Hw_llb=Hw_r*Hr_lb;
Hw_lrb=Hw_r*Hr_rb;

%Posición pie izquierdo frontal conr especto al hombro
foot_llf=pinv(Hw_llf)*foot_lf;
%Posición pie derecho frontal conr especto al hombro
foot_lrf=pinv(Hw_lrf)*foot_rf;
%Posición pie izquierdo trazero con respecto al hombro
foot_llb=pinv(Hw_llb)*foot_lb;
%Posición pie derecho trazero conr especto al hombro
foot_lrb=pinv(Hw_lrb)*foot_rb;

Q_lf=IK_leg(foot_llf,1);
Q_rf=IK_leg(foot_lrf,0);
Q_lb=IK_leg(foot_llb,1);
Q_rb=IK_leg(foot_lrb,0);
Q_d=[Q_lf;Q_rf;Q_lb;Q_rb];
