function [wconv] = mong_conv_gyro(w)

G=load('gyro','X');
kgx=G.X(1);
kgy=G.X(2);
kgz=G.X(3);
bgx=G.X(4);
bgy=G.X(5);
bgz=G.X(6);
gayza=G.X(7);
gazya=G.X(8);
gaxza=G.X(9);
gazxa=G.X(10);
gaxya=G.X(11);
gayxa=G.X(12);

Kg=[kgx 0 0;
    0 kgy 0;
    0 0 kgz];

bg=[bgx; bgy; bgz];

Tg=[1    -gayza gazya;
    gaxza 1    -gazxa;
   -gaxya gayxa 1];

wconv=zeros(size(w));
for i=1:length(w(:,1))
    auxg=Tg*(Kg^(-1))*(w(i,:)'-bg);
    wconv(i,:)=auxg';
end