function K=uquad_lqr(A,B,Q,R)
%Solves the continues Riccati algebraic ecuation
%A'*P+P*A-P*B*R^(-1)*B'*P+Q ussing shur vector method


F=A;
G=B*R^(-1)*B';
H=Q;

%Hamiltonian matrix
Z=[F -G; -H -F'];

%[V,D]=eig(Z);
%[V,H]=hess(Z)
[U,T]=schur(Z);
U11=T(1:12,1:12);
U21=T(13:24,1:12);

P=U11\U21;

K=R^(-1)*B'*P;
