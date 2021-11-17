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

function delay = get_delay_time(freq,original_function,phase_causal, data_rate, rise_time_per)

N = length(freq);
df = freq(2) - freq(1);
dt = 1/(2*freq(end)+df);

%Add Gaussian Filter
f_cut = 3*data_rate/2*1e9;
sigma = 1/2/pi/f_cut;
for k=1:N
   gaussina_filter(k) = exp(-2*pi*pi*freq(k)*freq(k)*sigma*sigma);
end


for i=1:N
     original_function(i) = original_function(i)*gaussina_filter(i);
     causal_function(i) = abs(original_function(i))*exp(-1j*(phase_causal(i)));
end

original_function_conj = add_conj(original_function);
causal_function_conj = add_conj(causal_function);

pulse_response = getpulse(dt,data_rate,2*N-1,rise_time_per);

for i=1:length(pulse_response)
    pulse_response_causal_freq(i) = pulse_response(i)*causal_function_conj(i);
    pulse_response_orig_freq(i) = pulse_response(i)*original_function_conj(i);
end

pulse_response_causal_time = ifft(pulse_response_causal_freq)/2;
pulse_response_orig_time = ifft(pulse_response_orig_freq)/2;

% tic
% [~,~,shift_ind] = alignsignals(pulse_response_causal_time,pulse_response_orig_time);
% shift_ind
% toc
% tic
shift_ind = -1*alignsignals2(pulse_response_causal_time,pulse_response_orig_time);
% toc
delay = shift_ind*dt;