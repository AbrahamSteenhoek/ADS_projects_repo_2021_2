% Copyright 2017 The Institute of Electrical and Electronics Engineers, Incorporated (IEEE).
%
% This work is licensed to The Institute of Electrical and Electronics
% Engineers, Incorporated (IEEE) under one or more contributor license
% agreements.
%
% See the NOTICE file distributed with this work for additional
% information regarding copyright ownership. Use of this file is
% governed by a BSD-style license, the terms of which are as follows:
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
%
%   * Redistributions of source code must retain the above copyright
%   notice, this list of conditions, the following disclaimer, and the
%   NOTICE file.
%
%   * Redistributions in binary form must reproduce the above
%   copyright notice, this list of conditions, the following
%   disclaimer in the documentation and/or other materials provided
%   with the distribution, and the NOTICE file.
%
%   * Neither the name of The Institute of Electrical and Electronics
%   Engineers, Incorporated (IEEE) nor the names of its contributors
%   may be used to endorse or promote products derived from this
%   software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
% COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%
% SPDX-License-Identifier: BSD-3-Clause

function [time_domain_matrix, time_] = getTimeDomainMatrix(sp_matrix, freq, port_num, data_rate, rise_time_per, pulse_shape)

N = length(freq);
%Add Gaussian Filter
f_cut = 3*data_rate/2*1e9;
sigma = 1/2/pi/f_cut;
rise_time = 1/data_rate*1000*rise_time_per;
f0 = 320/rise_time*1e9;
for k=1:N
    filter(k) = 1;
    if (pulse_shape == 2)
        filter(k) = 1/(1+1j*freq(k)/f0);
    end
    if (pulse_shape == 3)
        filter(k) = exp(-2*pi*pi*freq(k)*freq(k)*sigma*sigma);
    end
end

% figure(111)
% plot(freq,gaussina_filter);

for i=1:port_num
    for j=1:port_num
        sij = squeeze(sp_matrix(i,j,:));
        sij =sij.*filter.';
        sij(1)=real(sij(1));
        %Extend to negative frequencies
        sij_conj = add_conj(sij);

        df = freq(2) - freq(1);
        dt = 1/(2*freq(end)+df);
        if(pulse_shape == 1)
            pulse = getGaussianPulse(dt,data_rate,2*N-1,rise_time_per);
        else
            pulse = getpulse(dt,data_rate,2*N-1,1.4*rise_time_per);
        end
        for k=1:length(pulse)
            pulse_response_freq(k) = pulse(k)*sij_conj(k);
        end
        time_domain_matrix(i,j,:) = ifft(pulse_response_freq);
        time_ = dt*(0:(length(pulse)-1));
    end
end
