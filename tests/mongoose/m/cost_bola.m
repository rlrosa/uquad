function f = cost_bola(x)


U =[ 0.00473160006403247     -2.18916898319836e-05      0.000309423482503981;
                         0       0.00455025410014059      7.83170752308679e-05;
                         0                         0       0.00534686558080686];
                     
c=      [23.3152609806586;
         -126.624459617958;
          19.0429011162953];                    

[a,w,m,b,t]=mong_read('bola',0);
for i=1:length(m(:,1));
mc(:,i)=(U*(m(i,:)'-c));
end


f=zeros(length(mc(:,1)),1);

for i=1:length(mc(1,:))
    f(i)=(mc(1,i)-x(1))^2+(mc(2,i)-x(2))^2+(mc(3,i)-x(3))^2-x(4)^2;
    
end
% 
% [a,w,m,b,t]=mong_read('bola',0);
% M=load('mag','X');
% kmx=M.X(1);
% kmy=M.X(2);
% kmz=M.X(3);
% bmx=M.X(4);
% bmy=M.X(5);
% bmz=M.X(6);
% mayza=M.X(7);
% mazya=M.X(8);
% maxza=M.X(9);
% mazxa=M.X(10);
% maxya=M.X(11);
% mayxa=M.X(12);
% % MAGNETOMETRO
% Km=[kmx 0 0;        
%     0 kmy 0;
%     0 0 kmz];
% 
% bm=[bmx; bmy; bmz];
% 
% Tm=[1    -mayza mazya;
%     maxza 1    -mazxa;
%    -maxya mayxa 1];
% 
% mconv=zeros(size(m));
% for i=1:length(m(:,1))
%     auxm=Tm*(Km^(-1))*(m(i,:)'-bm);
%     mconv(i,:)=auxm';
% end
% maximo= max([max(abs(mconv(:,1))),max(abs(mconv(:,2))),max(abs(mconv(:,3)))]);
% mc = [mconv(:,1)/maximo mconv(:,2)/maximo mconv(:,3)/maximo];
%  f=zeros(length(mc(:,1)),1);
% 
% for i=1:length(mc(:,1))
%     f(i)=(mc(i,1)-x(1))^2+(mc(i,2)-x(2))^2+(mc(i,3)-x(3))^2-x(4)^2;
%     
% end