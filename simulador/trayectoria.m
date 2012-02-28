%N waypoints

N=5;
K1=1;
K2=1;
x0=100*rand;
xN=100*rand;
y0=100*rand;
yN=100*rand;
x=100*rand([N-2 1]);
y=100*rand([N-2 1]);

vr=sym('vr',[N-1 1] );
xr=sym('xr',[N-1 1] );
yr=sym('yr', [N-1 1] );
phir=sym('phir',[N-1 1] );
lambda=sym('lambda',2*(N-2) );

%f=K1*sum(vr)+K2

gradfk=[K1*ones(N-1,1); 2*K2*(xr-[x0;x]);2*K2*(yr-[y0;y]);zeros(size(phir))];

Ck=[xr(1)-x0;
    yr(1)-y0;
    xr(2:N-1)-xr(1:N-2)-vr(1:N-2).*cos(phir(1:N-2));
    yr(2:N-1)-yr(1:N-2)-vr(1:N-2).*sin(phir(1:N-2));
    xN-xr(N-1)-vr(N-1)*cos(phir(N-1));
    yN-yr(N-1)-vr(N-1)*cos(phir(N-1));
    ]

