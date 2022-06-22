function draw_byte(pose_d,Q_d) 
%pose=posición y orientación, pose_d=[dx;dy;dz;roll;pitch;yaw]
%Q_d=[q_lf;q_rf;qlb;q_rb]
%q_ij=[q1_ij;q2_ij;q3_ij] q1_ij=roll q2_ij=pitch_1 q3_ij=pitch_2

D1=200; %Distancia entre ejes de pitch de las piernas
D2=90; %Distancia entre ejes de rollo de las piernas
Zh=120; %Distancia del piso en home

%Valores de las uniones en piernas
%Pierna frontal izquierda radianes
q1_lf=Q_d(1); %motor roll left front
q2_lf=Q_d(2); %motor pitch1 left front
q3_lf=Q_d(3); %motor pitch2 left front
%Pierna frontal derecha radianes
q1_rf=Q_d(4); %motor roll right front
q2_rf=Q_d(5); %motor pitch1 right front
q3_rf=Q_d(6); %motor pitch2 right front
%Pierna trasera izquierda radianes
q1_lb=Q_d(7); %motor roll left back
q2_lb=Q_d(8); %motor pitch1 left back
q3_lb=Q_d(9); %motor pitch2 left back
%Pierna trasera derecha radianes
q1_rb=Q_d(10); %motor roll right back
q2_rb=Q_d(11); %motor pitch1 right back
q3_rb=Q_d(12); %motor pitch2 right back

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

%Origen de coordenadas del robot (centro geométrico en home);
O_robot=Hw_r(1:3,4);

%Orígenes de los sitemas de las Hpiernas
Ofl=Hw_llf(1:3,4);
Ofr=Hw_lrf(1:3,4);
Obl=Hw_llb(1:3,4);
Obr=Hw_lrb(1:3,4);

plot3(O_robot(1),O_robot(2),O_robot(3),'*r'); %origen del robot 
plot3(Ofl(1),Ofl(2),Ofl(3),'*b');
plot3(Ofr(1),Ofr(2),Ofr(3),'*b');
plot3(Obl(1),Obl(2),Obl(3),'*b');
plot3(Obr(1),Obr(2),Obr(3),'*b');
line([Ofl(1),Ofr(1)],[Ofl(2),Ofr(2)],[Ofl(3),Ofr(3)],'color','red');
line([Obl(1),Obr(1)],[Obl(2),Obr(2)],[Obl(3),Obr(3)],'color','red');
line([Ofl(1),Obl(1)],[Ofl(2),Obl(2)],[Ofl(3),Obl(3)],'color','red');
line([Ofr(1),Obr(1)],[Ofr(2),Obr(2)],[Ofr(3),Obr(3)],'color','red');

%%%%%%%%%%%%%%%%%%Denavit Hartenberg%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%Variables%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L1=100; %Femur mm
L2=100; %tibia mm
ds=45; %distancia shoulder mm

%Parámetros DH pierna frontal izquierda
a1_lf=0; alpha1_lf=-pi/2; d1_lf=0; th1_lf=q1_lf-(pi/2);
a1s_lf=0; alpha1s_lf=0; d1s_lf=ds; th1s_lf=0;
a2_lf=L1; alpha2_lf=pi; d2_lf=0; th2_lf=q2_lf;
a3_lf=L2; alpha3_lf=pi; d3_lf=0; th3_lf=q3_lf;

%Parámetros DH pierna frontal derecha
a1_rf=0; alpha1_rf=-(pi/2); d1_rf=0; th1_rf=q1_rf-(pi/2);
a1s_rf=0; alpha1s_rf=0; d1s_rf=ds; th1s_rf=0;
a2_rf=L1; alpha2_rf=pi; d2_rf=0; th2_rf=q2_rf;
a3_rf=L2; alpha3_rf=pi; d3_rf=0; th3_rf=q3_rf;

%Parámetros DH pierna trasera izquierda
a1_lb=0; alpha1_lb=-pi/2; d1_lb=0; th1_lb=q1_lb-(pi/2);
a1s_lb=0; alpha1s_lb=0; d1s_lb=ds; th1s_lb=0;
a2_lb=L1; alpha2_lb=pi; d2_lb=0; th2_lb=q2_lb;
a3_lb=L2; alpha3_lb=pi; d3_lb=0; th3_lb=q3_lb;

%Parámetros DH pierna trasera derecha
a1_rb=0; alpha1_rb=-(pi/2); d1_rb=0; th1_rb=q1_rb-(pi/2);
a1s_rb=0; alpha1s_rb=0; d1s_rb=ds; th1s_rb=0;
a2_rb=L1; alpha2_rb=pi; d2_rb=0; th2_rb=q2_rb;
a3_rb=L2; alpha3_rb=pi; d3_rb=0; th3_rb=q3_rb;

%Matrices de transformación frontal izquierda
A1_lf=calc_Ai(a1_lf,alpha1_lf,d1_lf,th1_lf);
A1s_lf=calc_Ai(a1s_lf,alpha1s_lf,d1s_lf,th1s_lf);
A2_lf=calc_Ai(a2_lf,alpha2_lf,d2_lf,th2_lf);
A3_lf=calc_Ai(a3_lf,alpha3_lf,d3_lf,th3_lf);

%Matrices de transformación frontal derecha
A1_rf=calc_Ai(a1_rf,alpha1_rf,d1_rf,th1_rf);
A1s_rf=calc_Ai(a1s_rf,alpha1s_rf,d1s_rf,th1s_rf);
A2_rf=calc_Ai(a2_rf,alpha2_rf,d2_rf,th2_rf);
A3_rf=calc_Ai(a3_rf,alpha3_rf,d3_rf,th3_rf);

%Matrices de transformación trasera izquierda
A1_lb=calc_Ai(a1_lb,alpha1_lb,d1_lb,th1_lb);
A1s_lb=calc_Ai(a1s_lb,alpha1s_lb,d1s_lb,th1s_lb);
A2_lb=calc_Ai(a2_lb,alpha2_lb,d2_lb,th2_lb);
A3_lb=calc_Ai(a3_lb,alpha3_lb,d3_lb,th3_lb);

%Matrices de transformación trasera derecha
A1_rb=calc_Ai(a1_rb,alpha1_rb,d1_rb,th1_rb);
A1s_rb=calc_Ai(a1s_rb,alpha1s_rb,d1s_rb,th1s_rb);
A2_rb=calc_Ai(a2_rb,alpha2_rb,d2_rb,th2_rb);
A3_rb=calc_Ai(a3_rb,alpha3_rb,d3_rb,th3_rb);

%Matrices de transformación individuales con respecto al sistema SC0 de la pierna 

%Pierna frontal izquierda
A01_lf=A1_lf;
A01s_lf=A1_lf*A1s_lf;
A02_lf=A1_lf*A1s_lf*A2_lf;
A03_lf=A1_lf*A1s_lf*A2_lf*A3_lf;

%Pierna frontal derecha
A01_rf=A1_rf;
A01s_rf=A1_rf*A1s_rf;
A02_rf=A1_rf*A1s_rf*A2_rf;
A03_rf=A1_rf*A1s_rf*A2_rf*A3_rf;

%Pierna trazera izquierda
A01_lb=A1_lb;
A01s_lb=A1_lb*A1s_lb;
A02_lb=A1_lb*A1s_lb*A2_lb;
A03_lb=A1_lb*A1s_lb*A2_lb*A3_lb;

%Pierna trazera derecha
A01_rb=A1_rb;
A01s_rb=A1_rb*A1s_rb;
A02_rb=A1_rb*A1s_rb*A2_rb;
A03_rb=A1_rb*A1s_rb*A2_rb*A3_rb;

%Matrices de transformación individuales con respecto al sistema SCw del mundo

%Pierna frontal izquierda
Aw1_lf=Hw_llf*A01_lf;
Aw1s_lf=Hw_llf*A01s_lf;
Aw2_lf=Hw_llf*A02_lf;
Aw3_lf=Hw_llf*A03_lf;

%Pierna frontal derecha
Aw1_rf=Hw_lrf*A01_rf;
Aw1s_rf=Hw_lrf*A01s_rf;
Aw2_rf=Hw_lrf*A02_rf;
Aw3_rf=Hw_lrf*A03_rf;

%Pierna trazera izquierda
Aw1_lb=Hw_llb*A01_lb;
Aw1s_lb=Hw_llb*A01s_lb;
Aw2_lb=Hw_llb*A02_lb;
Aw3_lb=Hw_llb*A03_lb;

%Pierna trazera derecha
Aw1_rb=Hw_lrb*A01_rb;
Aw1s_rb=Hw_lrb*A01s_rb;
Aw2_rb=Hw_lrb*A02_rb;
Aw3_rb=Hw_lrb*A03_rb;


%Tomar los ultimos valores vector matriz homogenea pierna izquierda frontal
SC0_lf=Hw_llf(1:3,4);
SC1_lf=Aw1_lf(1:3,4);
SC1s_lf=Aw1s_lf(1:3,4);
SC2_lf=Aw2_lf(1:3,4);
SC_foot_lf=Aw3_lf(1:3,4);

%Tomar los ultimos valores vector matriz homogenea pierna derecha frontal
SC0_rf=Hw_lrf(1:3,4);
SC1_rf=Aw1_rf(1:3,4);
SC1s_rf=Aw1s_rf(1:3,4);
SC2_rf=Aw2_rf(1:3,4);
SC_foot_rf=Aw3_rf(1:3,4);

%Tomar los ultimos valores vector matriz homogenea pierna izquierda trazera
SC0_lb=Hw_llb(1:3,4);
SC1_lb=Aw1_lb(1:3,4);
SC1s_lb=Aw1s_lb(1:3,4);
SC2_lb=Aw2_lb(1:3,4);
SC_foot_lb=Aw3_lb(1:3,4);

%Tomar los ultimos valores vector matriz homogenea pierna derecha trazera
SC0_rb=Hw_lrb(1:3,4);
SC1_rb=Aw1_rb(1:3,4);
SC1s_rb=Aw1s_rb(1:3,4);
SC2_rb=Aw2_rb(1:3,4);
SC_foot_rb=Aw3_rb(1:3,4);

%Gráficar joints 
%Pierna izquierda frontal
plot3(SC0_lf(1),SC0_lf(2),SC0_lf(3),'*m');
plot3(SC1_lf(1),SC1_lf(2),SC1_lf(3),'*m');
plot3(SC1s_lf(1),SC1s_lf(2),SC1s_lf(3),'*g');
plot3(SC2_lf(1),SC2_lf(2),SC2_lf(3),'*m');
plot3(SC_foot_lf(1),SC_foot_lf(2),SC_foot_lf(3),'*r');
line([SC0_lf(1),SC1_lf(1)],[SC0_lf(2),SC1_lf(2)],[SC0_lf(3),SC1_lf(3)],'color','blue');
line([SC1_lf(1),SC1s_lf(1)],[SC1_lf(2),SC1s_lf(2)],[SC1_lf(3),SC1s_lf(3)],'color','blue');
line([SC1s_lf(1),SC2_lf(1)],[SC1s_lf(2),SC2_lf(2)],[SC1s_lf(3),SC2_lf(3)],'color','blue');
line([SC2_lf(1),SC_foot_lf(1)],[SC2_lf(2),SC_foot_lf(2)],[SC2_lf(3),SC_foot_lf(3)],'color','blue');

%Lineas auxiliares
% line([SC1s_lf(1),SC_foot_lf(1)],[SC1s_lf(2),SC_foot_lf(2)],[SC1s_lf(3),SC_foot_lf(3)],'color','magenta');

%Pierna derecha frontal
plot3(SC0_rf(1),SC0_rf(2),SC0_rf(3),'*m');
plot3(SC1_rf(1),SC1_rf(2),SC1_rf(3),'*m');
plot3(SC1s_rf(1),SC1s_rf(2),SC1s_rf(3),'*g');
plot3(SC2_rf(1),SC2_rf(2),SC2_rf(3),'*m');
plot3(SC_foot_rf(1),SC_foot_rf(2),SC_foot_rf(3),'*r');
line([SC0_rf(1),SC1_rf(1)],[SC0_rf(2),SC1_rf(2)],[SC0_rf(3),SC1_rf(3)],'color','blue');
line([SC1_rf(1),SC1s_rf(1)],[SC1_rf(2),SC1s_rf(2)],[SC1_rf(3),SC1s_rf(3)],'color','blue');
line([SC1s_rf(1),SC2_rf(1)],[SC1s_rf(2),SC2_rf(2)],[SC1s_rf(3),SC2_rf(3)],'color','blue');
line([SC2_rf(1),SC_foot_rf(1)],[SC2_rf(2),SC_foot_rf(2)],[SC2_rf(3),SC_foot_rf(3)],'color','blue');
%Lineas auxiliares
% line([SC1s_rf(1),SC_foot_rf(1)],[SC1s_rf(2),SC_foot_rf(2)],[SC1s_rf(3),SC_foot_rf(3)],'color','blue');

%Gráficar joints 
%Pierna izquierda trazera
plot3(SC0_lb(1),SC0_lb(2),SC0_lb(3),'*m');
plot3(SC1_lb(1),SC1_lb(2),SC1_lb(3),'*m');
plot3(SC1s_lb(1),SC1s_lb(2),SC1s_lb(3),'*g');
plot3(SC2_lb(1),SC2_lb(2),SC2_lb(3),'*m');
plot3(SC_foot_lb(1),SC_foot_lb(2),SC_foot_lb(3),'*r');
line([SC0_lb(1),SC1_lb(1)],[SC0_lb(2),SC1_lb(2)],[SC0_lb(3),SC1_lb(3)],'color','blue');
line([SC1_lb(1),SC1s_lb(1)],[SC1_lb(2),SC1s_lb(2)],[SC1_lb(3),SC1s_lb(3)],'color','blue');
line([SC1s_lb(1),SC2_lb(1)],[SC1s_lb(2),SC2_lb(2)],[SC1s_lb(3),SC2_lb(3)],'color','blue');
line([SC2_lb(1),SC_foot_lb(1)],[SC2_lb(2),SC_foot_lb(2)],[SC2_lb(3),SC_foot_lb(3)],'color','blue');

%Pierna derecha trazera
plot3(SC0_rb(1),SC0_rb(2),SC0_rb(3),'*m');
plot3(SC1_rb(1),SC1_rb(2),SC1_rb(3),'*m');
plot3(SC1s_rb(1),SC1s_rb(2),SC1s_rb(3),'*g');
plot3(SC2_rb(1),SC2_rb(2),SC2_rb(3),'*m');
plot3(SC_foot_rb(1),SC_foot_rb(2),SC_foot_rb(3),'*r');
line([SC0_rb(1),SC1_rb(1)],[SC0_rb(2),SC1_rb(2)],[SC0_rb(3),SC1_rb(3)],'color','blue');
line([SC1_rb(1),SC1s_rb(1)],[SC1_rb(2),SC1s_rb(2)],[SC1_rb(3),SC1s_rb(3)],'color','blue');
line([SC1s_rb(1),SC2_rb(1)],[SC1s_rb(2),SC2_rb(2)],[SC1s_rb(3),SC2_rb(3)],'color','blue');
line([SC2_rb(1),SC_foot_rb(1)],[SC2_rb(2),SC_foot_rb(2)],[SC2_rb(3),SC_foot_rb(3)],'color','blue');
