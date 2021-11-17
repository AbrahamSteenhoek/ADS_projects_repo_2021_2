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

function [causal_model, new_freq, delay] = getCausalModel(freq,sij,data_rate,rise_time_per)

df = freq(2)-freq(1);
%DC extrapolation
if(freq(1) ~= 0)
    disp('extrapolating to dc ...');
    [freq,sij] = dcextrapolation(freq,sij);   
end
%Interpolate data
New_N = floor(freq(end)/df);
new_freq = df*(ceil(freq(1)/df):New_N)';
sij = interpolation(freq,sij,new_freq);
freq = new_freq;
N = length(freq);

%Extend to negative frequencies
sij_conj = add_conj(sij);
%Extract magnitude
sij_magn_conj = real(log(abs(sij_conj)));

%Convert magnitude into time domain
sij_magn_time = ifft(sij_magn_conj);
%Multiply by sign(t)
for i=N+1:2*N-1
   sij_magn_time(i) = (-1)*sij_magn_time(i);
end
sij_magn_time = 1j * sij_magn_time;
%Calculate Phase
sij_phase_enforced = real(fft(sij_magn_time));
sij_phase_origin = -unwrap(angle(sij));

%Calculate Delay
%delay = get_delay(freq(1:N),sij_phase_origin(1:N));
delay = get_delay_time(freq(1:N),sij(1:N),sij_phase_enforced(1:N),data_rate,rise_time_per);

df = freq(2) - freq(1);
dt = 1/(2*freq(end)+df);
delay = (round(delay/dt))*dt;

for i=1:N
     w = 2*pi*freq(i);
     sij_phase_origin_nonlinear(i) = sij_phase_origin(i) - w*delay;
     causal_model(i) = exp(sij_magn_conj(i))*exp(-1j*(sij_phase_enforced(i)))*exp(-1j*delay*w);
end

delay = round(delay/dt);

