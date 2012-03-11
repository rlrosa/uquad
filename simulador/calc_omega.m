function w=calc_omega(F)

syms x
val=solve(3.5296e-5*x^2-4.9293e-4*x-F);
val_hov=eval(val);
        
if (val_hov(1)>=0)
     w=val_hov(1);
else w=val_hov(2);
end