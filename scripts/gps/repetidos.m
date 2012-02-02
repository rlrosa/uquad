function repetidos(easting, northing, elevation, log_level)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function repetidos(easting, northing, elevation, log_level)
%
% Se observo que el GPS a veces da info repetida, no estamos seguros si es
% culpa del GPS o del software en el medio.
% Ese script analiza la informacion que provino del GPS, buscando cuando,
% entre datos sucesivos, se repiten uno, dos o los tres campos.
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

if(nargin < 4)
  log_level = 2; %0,1,2
end

eq = zeros(3,1);

seg = zeros(3,1);

enc = zeros(3,1);

for i=1:length(elevation)-1
  if(easting(i) == easting(i+1))
    if(log_level > 2)
      fprintf('UNO en:\t%d\n',i)
    end
    eq(1) = eq(1) + 1;
    enc(1) = enc(1) + 1;
    if(enc(1) > seg(1))
      seg(1) = enc(1);
    end
    if(northing(i) == northing(i+1))
      if(log_level > 2)
        fprintf('DOS en:\t%d\n',i)
      end
      eq(2) = eq(2) + 1;
      enc(2) = enc(2) + 1;
      if(enc(2) > seg(2))
        seg(2) = enc(2);
      end
      if(elevation(i) == elevation(i+1))
        eq(3) = eq(3) + 1;
        enc(3) = enc(3) + 1;
        if(enc(3) > seg(3))
          seg(3) = enc(3);
        end
        if(log_level > 2)
          fprintf('TRES en:\t%d\n',i)
        end
      else
        if(log_level > 1)
          fprintf('seguido\tTRES:\t%d\n',enc(3))
        end
        enc(3) = 0;
      end
    else
      if(log_level > 1)
        fprintf('seguido\tDOS:\t%d\n',enc(2))
      end
      enc(2) = 0;
    end
  else
    if(log_level > 1)
      fprintf('seguido\tUNO:\t%d\n',enc(1))
    end
    enc(1) = 0;
  end  
end

if(log_level > 0)
  fprintf('\n%d\t%d\t%d\n',seg(1), seg(2), seg(3));
end
