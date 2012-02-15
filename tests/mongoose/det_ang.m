A=imread('angulo3.jpg');
imshow(A)
[X,Y]=ginput(3);
a=sqrt((X(2)-X(3))^2+(Y(2)-Y(3))^2);
b=sqrt((X(2)-X(1))^2+(Y(2)-Y(1))^2);
c=sqrt((X(3)-X(1))^2+(Y(3)-Y(1))^2);
gamma=180/pi*acos((a^2+b^2-c^2)/(2*a*b))