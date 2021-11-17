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

clc;
clear all;
close all;

datarate = 10e9;
rt = 0.4;
num_ui = 32;
dt = 1/datarate/num_ui
num_samples = 1000;
t=[-num_samples:num_samples]*dt; 
b=0;

sigma=rt/datarate/(sqrt(-log(0.8)) - sqrt(-log(0.2)));
for i=1:length(t)
 G(i) = exp(-t(i)^2/sigma^2);
end
middle = (length(t) - 1)/2;
start_point = 1.5*num_ui;
for i=1:length(t)
 GG(i) = G(1 + mod(i+middle-start_point,length(t)));
end

figure(1)
plot(dt*(1:length(GG))*1e12,GG, 'r','lineWidth', 4); hold on;

 
df = 1/length(t)/dt/2;
for i=1:(length(t)-1)/2
 om = 2*pi*df*(i-1);
 G3(i) = exp(-sigma^2*om^2)*sigma*sqrt(2*pi)*df;
end
 
G2 = fft(GG);

figure(22)
plot(imag(G2), 'r','lineWidth', 4); hold on;
G2(1)/G3(1)

figure(2)
plot((1:length(G2))*df/1e9,db(G2), 'r','lineWidth', 4); hold on;
plot((1:length(G3))*df/1e9,db(G3), '--b','lineWidth', 4); hold on;

G3 = add_conj(G3);
G4 = ifft(G3);
figure(4)
plot(G4, 'r','lineWidth', 4); hold on;
