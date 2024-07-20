clc; clear; close all;

% Main script to compare BER of BPSK and BFSK systems

% Parameters
fs = 100e3;               % Sampling frequency
t = 0:1/fs:1;             % Time vector
fm = 3e3;                 % Message frequency
fc = 20e3;                % Carrier frequency for BPSK and BFSK
SNR_dB = 0:5:30;          % SNR range in dB
msg_bits = randi([0 1], 1, length(t));  % Random binary message

% Modulate signals
bpsk_mod = bpsk_modulate(msg_bits);
bfsk_mod = bfsk_modulate(msg_bits, fc, t);

% Calculate BER for different SNR values
berBPSK = calculate_ber(bpsk_mod, msg_bits, SNR_dB);
berBFSK = calculate_ber(bfsk_mod, msg_bits, SNR_dB, fc, t);

% Plot BER vs SNR
plot_ber(SNR_dB, berBPSK, berBFSK);

%% Function to modulate BPSK
function bpsk_signal = bpsk_modulate(bits)
    bpsk_signal = 2*bits - 1;  % BPSK: 0 -> -1, 1 -> 1
end

%% Function to modulate BFSK
function bfsk_signal = bfsk_modulate(bits, fc, t)
    bfsk_signal = cos(2*pi*fc*t + pi*bits);  % BFSK: 0 -> cos(2*pi*fc*t), 1 -> cos(2*pi*fc*t + pi)
end

%% Function to calculate BER
function ber = calculate_ber(mod_signal, original_bits, SNR_dB, fc, t)
    ber = zeros(length(SNR_dB), 1);
    for i = 1:length(SNR_dB)
        SNR = SNR_dB(i);
        rx_signal = awgn(mod_signal, SNR, 'measured');  % Add AWGN noise

        if nargin == 5  % For BFSK demodulation
            demod_bits = cos(2*pi*fc*t) .* rx_signal > 0;
        else  % For BPSK demodulation
            demod_bits = rx_signal > 0;
        end

        ber(i) = sum(original_bits ~= demod_bits) / length(original_bits);
    end
end

%% Function to plot BER vs SNR
function plot_ber(SNR_dB, berBPSK, berBFSK)
    figure;
    semilogy(SNR_dB, berBPSK, '-o', 'LineWidth', 2);
    hold on;
    semilogy(SNR_dB, berBFSK, '-s', 'LineWidth', 2);
    grid on;
    title('BER vs SNR for BPSK and BFSK');
    xlabel('SNR (dB)');
    ylabel('Bit Error Rate (BER)');
    legend('BPSK', 'BFSK');
    hold off;
end
