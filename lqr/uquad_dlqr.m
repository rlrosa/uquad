function K=uquad_dlqr(A,B,Q,R)

rows=length(B(:,1));
cols=length(B(1,:));
Ts=evalin('base','Ts');
%Ts = 10e-3;
%syms s;

%Pique de wikipedia http://en.wikipedia.org/wiki/Discretization
D = uquad_exp([A B;zeros(4,28)]*Ts);
A = D(1:rows,1:rows);
B = D(1:rows,rows+1:end);

P=Q;
k=0;
K=ones(cols,rows);


while norm(K-k)>10e-5
    k=K;
    K=-(R+B'*P* B)\B'*P*A; 
    P=Q+K'*R*K+(A+B*K)'*P*(A+B*K);
end
K=-K;

end