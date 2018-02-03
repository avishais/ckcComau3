clear all
clc

%%

I1 = imread('start.ppm');
I2 = imread('goal.ppm');

I1 = imrotate(I1, 180);
I2 = imrotate(I2, 180);
I1 = flipdim(I1, 2);
I2 = flipdim(I2, 2);

% Environment VI
I1 = imcrop(I1, [40 100 992-40 660-100]);
I2 = imcrop(I2, [40 100 992-40 660-100]);
% 
I1 = imresize(I1, [340 580]);
I2 = imresize(I2, [340 580]);

imshow(I1)
%%

for j = 0:3
    for i = 1:size(I1,2)
        I1(1+j,i,:) = [0 0 0];
        I1(end-j,i,:) = [0 0 0];
        I2(1+j,i,:) = [0 0 0];
        I2(end-j,i,:) = [0 0 0];
    end
    
    for i = 1:size(I1,1)
        I1(i,1+j,:) = [0 0 0];
        I1(i,end-j,:) = [0 0 0];
        I2(i,1+j,:) = [0 0 0];
        I2(i,end-j,:) = [0 0 0];
    end
end

I = [I1 I2];

I = insertText(I,[0 -10],'(f)','FontSize',60,'BoxColor',...
    'w','BoxOpacity',0,'TextColor','black');

imshow(I);

imwrite(I, 'envVI.png');

