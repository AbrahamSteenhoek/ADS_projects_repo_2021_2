function [data_side1,data_side2] = IEEEP3702xThru_Octave(data_2xthru,freq_2xthru)
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

%
% Function: [data_side1,data_side2] = IEEEP3702xThru(data_2xthru,freq_2xthru)
%
% Author: Jason J. Ellison
%
% Revision: Octave v0
% 
% IEEEP3702xThru.m creates error boxes from a test fixture 2x thru. 
% 
% Input: 
% data_2xthru--a two by two by n dimentional array. Where, n is the number of frequency points. The array is the 2x-thru S-parameters.
% freq_2xthru--a n by one dimensional array that represents the frequency vector. It cannot start with DC and must have uniform frequency steps.
% 
% Outputs: 
% data_side1--a two by two by n dimentional array. Where, n is the number of frequency points. The array is the side 1 fixture model S-parameters.
% data_side2--a two by two by n dimentional array. Where, n is the number of frequency points. The array is the side 2 fixture model S-parameters.

f = freq_2xthru;
n = length(f);
n_orig = length(f);
npad = n*2+1;
p2x = data_2xthru;
p112x = squeeze(p2x(1,1,:));
p212x = squeeze(p2x(2,1,:));

% get the DC point of p112x and p212x and interpolate
p112xDC = DC(p112x,f);
p112x = [p112xDC; p112x];

p212xDC = DC(p212x,f);
p212x = [p212xDC; p212x];
p112x = makeSymmetric(p112x);
p212x = makeSymmetric(p212x);

% convert p112x and p212x to time
t112x = real(ifft(p112x));
t212x = real(ifft(p212x));

% convert t112x and t212x to step response
t112xStep = makeStep(fftshift(t112x));
t212xStep = makeStep(fftshift(t212x));

% find the mid point of the trace in z
[~,mid_z] = min(abs(t212xStep-0.5));

% make the t112xStep t111xStep
t111xStep = zeros(n*2 + 1,1);
t111xStep((n+1):mid_z) = t112xStep((n+1):mid_z);
t111xStep = ifftshift(t111xStep);
% t111xStep = t112xStep;
% t111xStep(mid_z:end-mid_z) = 0;

% convert t111xStep to t111x
t111x = t111xStep - [0 ; t111xStep(1:end-1)];

% convert to p111x
p111x = fft(t111x);

% calculate the p221x and p211x
p221x = (p112x - p111x)./p212x;

% the choice of sign in the sqrt is important. Use the sign that captures the proper angle.
p211x = zeros(npad,1);
p211x(1) = sqrt(p212x(1).*(1-p221x(1).^2));
test = sqrt(p212x.*(1-p221x.^2));
k = 1;
for i = 2:npad
    if angle(test(i)) - angle(test(i-1)) > 0
        k = k*-1;
    end
    p211x(i) = k*sqrt(p212x(i).*(1-p221x(i).^2));
end

% convert errorbox elements to original frequency vector
p111x = p111x(2:length(f)+1);
p211x = p211x(2:length(f)+1);
p221x = p221x(2:length(f)+1);

% create the error box and make the s-parameter block
errorbox = zeros(2,2,n_orig);
errorbox(1,1,:) = p111x;
errorbox(2,1,:) = p211x;
errorbox(1,2,:) = p211x;
errorbox(2,2,:) = p221x;

data_side1 = errorbox;

p112x = squeeze(p2x(2,2,:));
p212x = squeeze(p2x(1,2,:));

% get the DC point of p112x and p212x and interpolate
p112xDC = DC(p112x,f);
p112x = [p112xDC; p112x];

p212xDC = DC(p212x,f);
p212x = [p212xDC; p212x];
p112x = makeSymmetric(p112x);
p212x = makeSymmetric(p212x);

% convert p112x and p212x to time
t112x = real(ifft(p112x));
t212x = real(ifft(p212x));

% convert t112x and t212x to step response
t112xStep = makeStep(fftshift(t112x));
t212xStep = makeStep(fftshift(t212x));

% find the mid point of the trace in z
[~,mid_z] = min(abs(t212xStep-0.5));

% make the t112xStep t111xStep
t111xStep = zeros(n*2 + 1,1);
t111xStep((n+1):mid_z) = t112xStep((n+1):mid_z);
t111xStep = ifftshift(t111xStep);
% t111xStep = t112xStep;
% t111xStep(mid_z:end-mid_z) = 0;

% convert t111xStep to t111x
t111x = t111xStep - [0 ; t111xStep(1:end-1)];

% convert to p111x
p111x = fft(t111x);

% calculate the p221x and p211x
p221x = (p112x - p111x)./p212x;

% the choice of sign in the sqrt is important. Use the sign that captures the proper angle.
p211x = zeros(npad,1);
p211x(1) = sqrt(p212x(1).*(1-p221x(1).^2));
test = sqrt(p212x.*(1-p221x.^2));
k = 1;
for i = 2:npad
    if angle(test(i)) - angle(test(i-1)) > 0
        k = k*-1;
    end
    p211x(i) = k*sqrt(p212x(i).*(1-p221x(i).^2));
end

% convert errorbox elements to original frequency vector
p111x = p111x(2:length(f)+1);
p211x = p211x(2:length(f)+1);
p221x = p221x(2:length(f)+1);

% create the error box and make the s-parameter block
errorbox = zeros(2,2,n_orig);
errorbox(1,1,:) = p111x;
errorbox(2,1,:) = p211x;
errorbox(1,2,:) = p211x;
errorbox(2,2,:) = data_side1(2,2,:);

data_side2 = errorbox([2 1],[2 1],:);

errorbox = data_side1;
errorbox(2,2,:) = p221x;
data_side1 = errorbox;



function [symmetric] = makeSymmetric(nonsymmetric)
    % [symmetric] = makeSymmetric(nonsymmetric)
    % this takes the nonsymmetric frequency domain input and makes it
    % symmetric.
    %
    % The function assumes the DC point is in the nonsymmetric data

    symmetric_abs = [abs(nonsymmetric); flip(abs(nonsymmetric(2:end)))];
    symmetric_ang = [angle(nonsymmetric); -flip(angle(nonsymmetric(2:end)))];
    symmetric = symmetric_abs.*exp(1i.*symmetric_ang);

function [DCpoint] = DC(s,f)       
    % find the real dc point. The equation is y = a*x^2+b.
    N = 2; % N can be changed to improve DC point accuracy. Larger numbers for short structures. Small numbers for long structures
    x = f(1:N);
    y = real(s(1:N));

    X = [ones(N,1) x.^2];
    t = (X'*X)\X'*y;
    DCpoint = t(1);

function [step] = makeStep(impulse)
    ustep = ones(length(impulse),1);    
    step = conv(ustep,impulse);
    step = step(1:length(impulse));