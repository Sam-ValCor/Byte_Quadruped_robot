clear all
close all
C=webcam('HP Wide Vision FHD Camera','Resolution','640x480','ExposureMode','auto','WhiteBalanceMode','auto', 'Hue','1');
figure;
while 1
img=snapshot(C);

[fil,col,c]=size(img);
img_YCbCr=rgb2ycbcr(img);
img_HSV=rgb2hsv(img);
r1=img_YCbCr(:,:,1)<img_YCbCr(:,:,3);
r1=double((r1));
r2=img_HSV(:,:,1)>img_HSV(:,:,2)&img_HSV(:,:,1)>img_HSV(:,:,3);
r2=double((r2));
r3=img(:,:,1)>img(:,:,2)&img(:,:,1)>img(:,:,3)&img(:,:,3)>img(:,:,2)&img(:,:,1)>=135&img(:,:,1)<=240&img(:,:,2)<=130&img(:,:,2)>=60&img(:,:,3)>=100&img(:,:,3)<=190;
r3=double((r3));
r4=img_HSV(:,:,1)>=0.915&img_HSV(:,:,1)<0.98&img_HSV(:,:,2)<0.65&img_HSV(:,:,2)>0.36&img_HSV(:,:,3)<0.86&img_HSV(:,:,3)>=0.5;
r4=double((r4));
Rt=r1.*r2.*r3.*r4;
%Rj=uint8(255*Rt);
img_out_rgb=double(img).*Rt;
img_out_gray=rgb2gray(uint8(img_out_rgb));
img_out=imbinarize(img_out_rgb,'adaptive');
img_out=255*double(img_out);
img_out=double(img_out);
se_C = strel('disk',10);
se_O = strel('disk',2);
im_byte=fliplr(imopen((imclose(img_out,se_C)),se_O));
s = regionprops(im_byte,'centroid');
img_rgb=fliplr(img);
centroids = cat(1,s.Centroid)
TF=double(isempty(centroids))
if TF==1
    centroids(:,1)=(col/2);
    centroids(:,2)=(fil/2);
end


subplot(1,2,1);
imshow(img_rgb);
hold on
subplot(1,2,1);
plot(centroids(:,1),centroids(:,2),'b*')
errorX=(centroids(end,1))-(col/2)
errorY=(fil/2)-(centroids(end,2))
k1=-0.1;
k2=-0.1;
yaw_d=k1*errorX;
pitch_d=k2*errorY;

%Traslación deseada
dx=0;
dy=0;
dz=-10;
%Orientación deseada
% roll=deg2rad(0);
% pitch=deg2rad(0);
% yaw=deg2rad(0);

%Orientación deseada
roll=deg2rad(0);
pitch=deg2rad(pitch_d);
yaw=deg2rad(yaw_d);

%Pose deseada
pose_d=[dx;dy;dz;roll;pitch;yaw];

% Posición del pie izquierdo frontal
foot_lf=[100;70;0;1]; %uno para que sea homogenea
foot_rf=[100;-70;0;1]; %uno para que sea homogenea
foot_lb=[-100;70;0;1]; %uno para que sea homogenea
foot_rb=[-100;-70;0;1]; %uno para que sea homogenea

%Graficamos el origen del mundo y la posición deseada del pie
drawnow;
subplot(1,2,2)
plot3(0,0,0,'*k'); %origen w=mundo
xlim([-250,250]);
ylim([-250,250]);
zlim([0,220]);
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
% view(179.99,180); %Vista lateral
% view(90,0); %Vista Frontal
grid on
hold on
subplot(1,2,2)
plot3(foot_lf(1),foot_lf(2),foot_lf(3),'dg');
subplot(1,2,2)
plot3(foot_rf(1),foot_rf(2),foot_rf(3),'dg');
subplot(1,2,2)
plot3(foot_lb(1),foot_lb(2),foot_lb(3),'dg');
subplot(1,2,2)
plot3(foot_rb(1),foot_rb(2),foot_rb(3),'dg');

 Q_calc=calc_ik_byte_main(pose_d,foot_lf,foot_rf,foot_lb,foot_rb);

%Pierna frontal izquierda
% q1_lf=deg2rad(0); %motor roll left front
% q2_lf=deg2rad(0); %motor pitch1 left front
% q3_lf=deg2rad(0); %motor pitch2 left front
%Angulos pierna izquierda frontal calulados
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
Q_d_deg = [ rad2deg(q1_lf), rad2deg(q2_lf), rad2deg(q3_lf); 
            rad2deg(q1_rf), rad2deg(q2_rf), rad2deg(q3_rf);
            rad2deg(q1_lb), rad2deg(q2_lb), rad2deg(q3_lb);
            rad2deg(q1_rb), rad2deg(q2_rb), rad2deg(q3_rb)]
%Configuración deseada
Q_d=[q_lf;q_rf;q_lb;q_rb];

% writePosition(s,q1_ticks_i);
subplot(1,2,2)
draw_byte(pose_d,Q_d);

hold off
end

    