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

function [extrapolated_frequancy, extrapolated_component] = dcextrapolation(f,s)
% calculate delay
delay = get_delay(f,-unwrap(angle(s)));

% extract delay to smooth origianl function 
s = s.*exp(1j*2*pi*f*delay);

% extract real and imaginary parts from the original function
re = real(s);
im = imag(s);

% create a*x^2+b parabola using (f(1),re(1)) and (f(2),re(2)) points
a = (re(2) - re(1))/(f(2)^2 - f(1)^2);
b = re(1) - a*f(1)^2;


% extend real part to DC
df = f(2) - f(1);
k = 1;
while ((k-1)*df < f(1))   
    f2(k) = (k-1)*df;
    re2(k) = a*((k-1)*df)^2 + b;
    k = k +1;
end
re = [re2.'; re];


% create a*x^3+b*x cubic parabola using (f(1),im(1)) and (f(2),im(2)) points
a = (im(2)/f(2) - im(1)/f(1))/(f(2)^2 - f(1)^2);
b = im(1)/f(1) - a*f(1)^2;


% extend imaginary part to DC
k = 1;
while ((k-1)*df < f(1))   
    im2(k) = a*((k-1)*df)^3 + b*((k-1)*df);
    k = k +1;
end
im = [im2.'; im];

extrapolated_frequancy = [f2.'; f];

% create complex function from real and imaginary parts
extrapolated_component = re + 1j*im;

% return delay 
extrapolated_component = extrapolated_component.*exp(-1j*2*pi*extrapolated_frequancy*delay);