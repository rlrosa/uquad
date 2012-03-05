%test círculos
clear all
close all
clc

X0=zeros(1,12);
X0(1)=5;
X0(8)=0;
ti=0;
tf=120;
setpoint= [1 1/5];
modo='cir';

[t,X,Y]=sim_lazo_cerrado(ti,tf,X0,setpoint,modo);

figure;plot(X(:,1),X(:,2));
figure;plot(t,X(:,3))