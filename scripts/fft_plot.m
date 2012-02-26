function [ Y,f ] = fft_plot( y, Fs, plot_color )
%--------------------------------------------------------------------------
%   function [ Y,f ] = fft_plot( y, Fs, plot_color  )
%
%   Calculates the FFT of the secuence 'y', assumed to have been sampled at
%   'Fs'.
%   If no output arguments are recieved, then the FFT is plotted.
%
%   Output:
%       - Y: single-sided FFT(y)
%       - f: Frequency vector corresponding to Y.
%       - plot_color: Color to use in plot.
%
%--------------------------------------------------------------------------

L=length(y);
NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Y = fft(y,NFFT);
Y=Y(1:NFFT/2+1); % single-sided spectrum
f = Fs/2*linspace(0,1,NFFT/2+1);

if(~exist('plot_color','var'))
  plot_color = 'b';
end

if (nargout < 1)
    % Plot single-sided amplitude spectrum.
    plot(f,abs(Y),plot_color) 
    title('Single-Sided Amplitude Spectrum of y(t)')
    xlabel('Frequency (Hz)')
    ylabel('|Y(f)|')
    grid on
end

end

