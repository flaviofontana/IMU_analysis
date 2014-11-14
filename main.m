close all
clear all
clc
format long g;

% TODO
% * Load info struct and fill all the legends and the labels with the
% appropriate strings.

%% Settings %
%%% LIST OF EXPERIMENTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                  HW LPF   SW LPF      Fs
% SET 0  : exp 0 - ACCELEROMETERS                [ 42 Hz,	30 Hz,	200 Hz ]
% SET 1  : exp 0 - GYROSCOPES                    [ 42 Hz,	30 Hz,	200 Hz ]
% SET 2  : exp 1 - SHAKING IMU - ACCELEROMETERS  [ 42 Hz,	30 Hz,	200 Hz ]
% SET 3  : exp 1 - SHAKING IMU - GYROSCOPES      [ 42 Hz,	30 Hz,	200 Hz ]
% SET 4  : exp 2 - ACCELEROMETERS                [ 10 Hz,	30 Hz,	200 Hz ]
% SET 5  : exp 2 - GYROSCOPES                    [ 10 Hz,	30 Hz,	200 Hz ]
% SET 6  : exp 3 - ACCELEROMETERS                [ 10 Hz,	NO,     200 Hz ]
% SET 7  : exp 4 - ACCELEROMETERS                [ 10 Hz,	NO,     500 Hz ]
% SET 8  : exp 5 - ACCELEROMETERS                [ 98 Hz,	30 Hz,	200 Hz ]
% SET 9  : exp 5 - GYROSCOPES                    [ 98 Hz,	NO,     200 Hz ]
% SET 10 : exp 6 - ACCELEROMETERS                [ 98 Hz,	NO,     200 Hz ]
% SET 11 : exp 6 - GYROSCOPES                    [ 98 Hz,	NO,     200 Hz ]
% SET 12 : exp 7 - MAGNETOMETERS                 [ UNKNWN,	UNKNWN,	80  Hz ]
% SET 13 : exp 7 - GPS - ECEF (few samples)      [ UNKNWN,	UNKNWN,	5   Hz ]
% SET 14 : exp 8 - GPS - ECEF                    [ UNKNWN,	UNKNWN,	5   Hz ]
% SET 15 : exp 8 - GPS - RADIANS                 [ UNKNWN,	UNKNWN,	5   Hz ]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

selector = 15;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[meas, meas_, timeline, Ts, Fs, HW_cutoff, SW_cutoff, info] = load_measures(selector);

% Remove average value
remove_average = true;

% Color map
clr = [ 0       0.447	0.741;	...   % BLUE
        0.85	0.325	0.098;	...   % RED
        0.929	0.694	0.125 ];      % YELLOW

%% Init %

meas_no = size(meas,2);
meas_length = size(meas,1);

%% Compute statistics

meas_AVG = mean(meas);

if remove_average == true
    meas = meas_;
end

meas_RMS = rms(meas);
meas_VAR = var(meas);

%% Plot histograms
figure

for kk = 1:meas_no
sp_hist(kk) = subplot(1,meas_no,kk);
bins = min(meas(:,kk)):((max(meas(:,kk))-min(meas(:,kk)))/50):max(meas(:,kk));
n = hist(meas(:,kk),bins);
bar (bins,n,'FaceColor',clr(kk,:),'EdgeColor','none');
f = (   1/(meas_RMS(kk)*sqrt(2*pi))   )  *  exp(-0.5*((bins-mean(meas(:,kk)))/meas_RMS(:,kk)).^2  );
f = f*sum(n)/sum(f);
hold on; plot (bins,f, 'r', 'LineWidth', 2);
legend('Meas Histogram','Best fitting Normal dist');
xlabel('measure values');
ylabel('# of occurrences');
end

if ~(strcmp(info.type,'gps_ecef') || strcmp(info.type,'gps_rad')) 
    linkaxes(sp_hist, 'xy');
end

%% Plot time history
figure

for kk = 1:meas_no
    sp_time(kk) = subplot(meas_no,1,kk);
    plot(timeline, meas(:,kk),'color',clr(kk,:));
    xlabel('time [s]');
    ylabel('[]');
    grid on;
end

if ~(strcmp(info.type,'gps_ecef') || strcmp(info.type,'gps_rad'))
linkaxes(sp_time);
end

%% FFT - Fast Fourier Transform
figure

% Working on power of 2 makes the FFT algorithm more efficient. 
% If NFFT > size(signal)The matlab fft() function automatically "zero-pads"
% the signal. This helps the visualization of the FFT with no effects on
% the actual frequency content of the signal.
NFFT = 2^nextpow2(meas_length); % Next power of 2 from length of y

Y = fft(meas,NFFT)/meas_length;
f = Fs/2*linspace(0,1,NFFT/2+1);

% Plot single-sided amplitude spectrum.

% Since the amplitude components in the FFT are split among negative and
% positive frequencies, if we want to preserve the amplitude information 
% when plotting the single-sided spectrum we need to multiply all the
% components - execpt the continuos 0Hz one - by a factor 2. In fact 0Hz 
% is unique and should not multiplied by 2. 
% !!!!!! This is not done here, rember this when you read the plot !!!!!!
for kk = 1:meas_no
    sp_fft(kk) = subplot(1,meas_no,kk);
    plot(f,2*abs(Y(1:NFFT/2+1,kk)),'color',clr(kk,:)); 
    xlabel('Hz');
    ylabel('[]');
    axis square;
    grid on;
end

if ~(strcmp(info.type,'gps_ecef') || strcmp(info.type,'gps_rad')) 
linkaxes(sp_fft, 'xy');
end

pause(2);

%% PSD - Power Spectral Density
figure

[Pxx,F] = pwelch(meas,hamming(meas_length),0,meas_length,Fs);

for kk = 1:meas_no
    sp_psd(kk) = subplot(1,meas_no,kk);
    plot(F,10*log10(Pxx(:,kk)),'color',clr(kk,:));
    xlabel('Hz'),
    ylabel('dB');
    axis square;
end

if ~(strcmp(info.type,'gps_ecef') || strcmp(info.type,'gps_rad'))
linkaxes(sp_psd, 'xy');
end

HW_str = sprintf('HW cut-off\n %f Hz', HW_cutoff);
SW_str = sprintf('SW cut-off\n %f Hz', SW_cutoff);

cutoff_F = min(HW_cutoff,SW_cutoff);

%Get the index of the F baseline @ cutoff frequency
tmp = abs(F-cutoff_F);
[idx, idx] = min(tmp); %index of closest value

for kk = 1:meas_no
    PSD(kk) = mean(Pxx(1:idx,kk));
    subplot(sp_psd(kk));
    PSD_str = sprintf('Average Power Spectral Density @ %f Hz: %f', cutoff_F, PSD(kk));
    
    l_h(1) = vline(HW_cutoff,'r:', HW_str);
    l_h(2) = vline(SW_cutoff,'b-', SW_str);
    l_h(3) = hline(10*log10(PSD(kk)),'k:', PSD_str);
    
    l_h(1).LineWidth = 2;
    l_h(2).LineWidth = 2;
    l_h(3).LineWidth = 2;
    
    grid on;
end

meas_NSD = sqrt(PSD);


%% Display results
clc
fprintf('\n‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\n\n');
fprintf('Data from experiment #%d\n',selector);
fprintf('Type: __TODO__ (%d datasets)\n',meas_no);
fprintf('Acqusition time: %f s\n',timeline(end));
fprintf('\n_________________________________________________________\n\n');
fprintf('Samples: %d\n',meas_length);
fprintf('Sampling Frequency: %d Hz\n', floor(Fs));
fprintf('Low Pass Filter cut-off: %d Hz\n', cutoff_F);
fprintf('\n_________________________________________________________\n\n');
fprintf('Average:\n');
fprintf('\t%1.10f\t%1.10f\t%1.10f\n\n',meas_AVG(1),meas_AVG(2),meas_AVG(3));
fprintf('Root Mean Square:\n');
fprintf('\t%1.10f\t%1.10f\t%1.10f\n\n',meas_RMS(1),meas_RMS(2),meas_RMS(3));
fprintf('Noise Spectral Densities:\n');
fprintf('\t%1.10f\t%1.10f\t%1.10f\n\n',meas_NSD(1),meas_NSD(2),meas_NSD(3));
fprintf('\n_________________________________________________________\n\n');