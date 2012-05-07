function [aconv] = mong_conv_acc(a)

A=load('acc','X');
kax=A.X(1);
kay=A.X(2);
kaz=A.X(3);
bax=A.X(4);
bay=A.X(5);
baz=A.X(6);
ayza=A.X(7);
azya=A.X(8);
axza=A.X(9);
azxa=A.X(10);
axya=A.X(11);
ayxa=A.X(12);

Ka=[kax 0 0;
    0 kay 0;
    0 0 kaz];

ba=[bax; bay; baz];

Ta=[1 -ayza azya;
   axza 1 -azxa;
   -axya ayxa 1];

Ka_1 = (Ka^-1);

aconv=zeros(size(a));
for i=1:length(a(:,1))
    aux=Ta*(Ka_1)*(a(i,:)'-ba);
    aconv(i,:)=aux';
end