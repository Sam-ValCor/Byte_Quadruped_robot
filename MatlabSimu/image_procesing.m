clear all
close all
C=webcam('c922 Pro Stream Webcam','Resolution','640x480','ExposureMode','manual','WhiteBalanceMode','auto', 'FocusMode','manual');

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
centroids = cat(1,s.Centroid);
TF=double(isempty(centroids));
if TF==1
    centroids(:,1)=(col/2);
    centroids(:,2)=(fil/2);
end


%imshow(uint8(im_byte));
imshow(img_rgb);
hold on
plot(centroids(:,1),centroids(:,2),'b*')
hold off
errorX=(centroids(end,1))-(col/2);
errorY=(fil/2)-(centroids(end,2));
k1=0.1;
k2=0.1;
yaw_d=k1*errorX;
pitch_d=k2*errorY;

end

    