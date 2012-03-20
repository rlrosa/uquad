function K=uquad_dlqr(A,B,Q,R)

rows=length(B(1,:));
cols=length(B(:,1));
Ts=5e-2;
syms s;
A=expm(A*Ts);
gamma=expm(A*s);
B=int(gamma,0,Ts)*B;
B=eval(B);
P=Q;
k=0;
K=ones(rows,cols);


while norm(K-k)>10e-5
    k=K;
    K=-(R+B'*P* B)^(-1)*B'*P*A; 
    P=Q+K'*R*K+(A+B*K)'*P*(A+B*K);
end
K=-K;

end