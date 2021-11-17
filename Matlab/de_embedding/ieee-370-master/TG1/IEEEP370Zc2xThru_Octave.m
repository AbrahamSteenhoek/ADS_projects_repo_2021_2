function [data_side1,data_side2] = IEEEP370Zc2xThru_Octave(data_2xthru,data_fix_dut_fix,freq_2xthru,varargin)
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
% FUNCTION: [data_side1,data_side2] = IEEEP3702xThru_Octave(data_2xthru,data_fix_dut_fix,z0,view)
%
% Author: Jason J. Ellison, Published February 22th, 2019
% Revision: v0 - Octave
% 
% IEEEP370Zc2xThru_Octave.m creates error boxes from a test fixture 2x thru and
% the fixture-dut-fixture S-parameters.
% 
% Input: 
% data_2xthru--a two by two by n dimensional array. Where, n is the number of frequency points. The array is the 2x-thru S-parameters.
% data_fix_dut_fix--a two by two by n dimensional array. Where, n is the number of frequency points. The array is the fixture-dut-fixture S-parameters.
% freq_2xthru--a n by one dimensional array that represents the frequency vector. It cannot start with DC and must have uniform frequency steps.
% z0 (optional): Port reference impedance. Default: 50.
% view(optional): set to 1 to view the impedance correction. Default: 0.
% 
% Outputs: 
% data_side1--a two by two by n dimentional array. Where, n is the number of frequency points. The array is the side 1 fixture model S-parameters.
% data_side2--a two by two by n dimentional array. Where, n is the number of frequency points. The array is the side 2 fixture model S-parameters.

if nargin >= 4
    z0 = varargin{1};
else
    z0 = 50;
end

if nargin >= 5
    view = varargin{2};
else
    view = 0;
end

if nargin > 5
    warning('Too many inputs. All input arguements after the third are ignored.');
end

p = data_2xthru;
f = freq_2xthru;

% grabbing S21
s212x = squeeze(p(2,1,:));

% get the attenuation and phase constant per length
beta_per_length = -unwrap(angle(s212x)); 
alpha_per_length = db(s212x)/-8.686;

if bandwidth_limit == 0
    % divide by 2*n + 1 to get prop constant per descrete unit length
    gamma = alpha_per_length + 1i.*beta_per_length; % gamma without DC
else
    % fit the attenuation up to the limited bandwidth
    [~,bwl_x] = min(abs(f - bandwidth_limit));
    X = [sqrt(f(1:bwl_x)) f(1:bwl_x) f(1:bwl_x).^2];
    b = (X'*X)\X'*alpha_per_length(1:bwl_x);
    alpha_per_length_fit = b(1).*sqrt(f) + b(2).*f + b(3).*f.^2;
    % divide by 2*n + 1 to get prop constant per descrete unit length
    gamma = alpha_per_length_fit + 1i.*beta_per_length; % gamma without DC
end



% make the first error box
data_side1 = makeErrorbox(data_fix_dut_fix,data_2xthru,freq_2xthru,gamma,z0,view);

% reverse the port order of fixture-dut-fixture and 2x thru
data_fix_dut_fix_reversed = data_fix_dut_fix([2 1],[2 1],:);
data_2xthru_reversed = data_2xthru([2 1],[2 1],:);

% make the second error box and correct the port order
data_side2 = makeErrorbox(data_fix_dut_fix_reversed,data_2xthru_reversed,freq_2xthru,gamma,z0,view);
data_side2 = data_side2([2 1],[2 1],:);

%% Supporting Functions

function errorbox = makeErrorbox(data_dut,data_2xthru,freq_2xthru,gamma,z0,view)
  % extract all relevant variables
  f = freq_2xthru;
  n = length(f);

  % extract the mid point from the 2x thru
  s212x = squeeze(data_2xthru(2,1,:));
  dc2x = 1;
  t212x = real(ifft(makeSymmetric([dc2x;s212x])));
  [~,x] = max(t212x);

  % define the reletive length
  l = 1/(2*x);

  % peel away the fixture and create the errorbox

  % create the errorbox seed (a perfect transmission line with no delay)
  errorbox = zeros(2,2,n);
  errorbox(2,1,:) = 1;
  errorbox(1,2,:) = 1;
  abcd_errorbox = stoabcd(errorbox,z0);

  if view == 1
    h = figure('color','white');
    df = f(2) - f(1);
    t_ns = linspace(-1/(2*df),1/(2*df),2*n + 1);
    fdf11 = squeeze(data_dut(1,1,:));
    dc11 = DC2(fdf11,f.');
    tfdf11 = real(ifft(makeSymmetric([dc11;fdf11])));
    tfdf11Step = makeStep(fftshift(tfdf11));
    zfdf11Step = -z0.*(tfdf11Step + 1)./(tfdf11Step - 1);
    drawnow;
  end

  for i = 1:x 
    % INPUTS: data_dut, f, abcd_errorbox, n
  
    % define the fixture-dut-fixture S-parameters
    s_dut = squeeze(data_dut(1,1,:));  
  
    % define the point for extraction
    dc11 = DC2(s_dut,f.');
    t11dut = real(ifft(makeSymmetric([dc11;s_dut])));
    t11dutStep = makeStep(fftshift(t11dut));
    z11dutStep = -z0.*(t11dutStep + 1)./(t11dutStep - 1);
    zLine = ifftshift(z11dutStep);  
  
    % create the TL
    TL = makeTL(gamma,l,zLine(1),z0);
  
    % peel away the TL
    abcd_TL = stoabcd(TL,z0);
    abcd_dut = stoabcd(data_dut,z0);
    for i = 1:n
      abcd_dut(:,:,i) = abcd_TL(:,:,i)\abcd_dut(:,:,i);
    end
    data_dut = abcdtos(abcd_dut,z0); % update data_dut
    
    % add to the errorbox
    for i = 1:n
      abcd_errorbox(:,:,i) = abcd_errorbox(:,:,i)*abcd_TL(:,:,i);
    end
  
    if view == 1
      errorbox = abcdtos(abcd_errorbox,z0);
      eb11 = squeeze(errorbox(1,1,:));
      dc11 = DC2(eb11,f.');
      teb11 = real(ifft(makeSymmetric([dc11;eb11])));
      teb11Step = makeStep(fftshift(teb11));
      zeb11Step = -z0.*(teb11Step + 1)./(teb11Step - 1);       
    
      figure(h); 
      hold off;
      plot(t_ns,zfdf11Step,t_ns,zeb11Step);    
      axis([0 t_ns(x*2 + n) -1 1], 'auto y');
      drawnow;    
   end
  endfor

  errorbox = abcdtos(abcd_errorbox,z0);
  % output
  errorbox = hybrid(errorbox,data_2xthru,f);
endfunction

function errorbox = hybrid(errorbox,data_2xthru,freq_2xthru)
    % taking the errorbox created by peeling and using it only for e00 and e11
    
    % grab s11 and s22 of errorbox model
    s11_errorbox = errorbox; clear errorbox;
    s111x = squeeze(s11_errorbox(1,1,:));
    s221x = squeeze(s11_errorbox(2,2,:));
    
    % grab s21 of the 2x thru measurement
    s212x = squeeze(data_2xthru(2,1,:));
    f = freq_2xthru;
    
    % the choice of sign in the sqrt is important. Use the sign that captures the proper angle.
    s211x = zeros(length(f),1);
    s211x(1) = sqrt(s212x(1).*(1-s221x(1).^2));
    test = sqrt(s212x.*(1-s221x.^2));
    k = 1;
    for i = 2:length(f)
        if angle(test(i)) - angle(test(i-1)) > 0
            k = k*-1;
        end
        s211x(i) = k*sqrt(s212x(i).*(1-s221x(i).^2));
    end
    
    % create the error box and make the s-parameter block
    errorbox_ = zeros(2,2,length(f));
    errorbox_(1,1,:) = s111x;
    errorbox_(2,1,:) = s211x;
    errorbox_(1,2,:) = s211x;
    errorbox_(2,2,:) = s221x;  
endfunction

function [DCpoint] = DC(s,f)       
% P370 DC extrapolation technique. See IEEE P370 Annex R
    smn = s;
    t = unwrap(angle(s))./(2.*pi.*f);
    w = 2*pi*f;
    delay = exp(-1i.*w.*t);

    f1 = f(10) ; f2 = f(20);
    real1 = real(smn(10).*delay(10)); real2 = real(smn(20).*delay(20));
    realb = (real1 - real2)/(f1^2 - f2^2);
    reala = (real1 - realb*f1^2);
    
    DCpoint = reala; 
endfunction

function [DCpoint] = DC2(s,f)       
        DCpoint = 0.002; % seed for the algorithm
        err = 1; % error seed
        allowedError = 1e-10; % allowable error
        cnt = 0;
        df = f(2) - f(1);
        n = length(f);
        t = linspace(-1/df,1/df,n*2+1);
        [~,ts] = min(abs(t - (-3e-9)));
        Hr = simple_filter(f,f(end)/2);
        while (err > allowedError)
            h1 = makeStep(fftshift(real(ifft(makeSymmetric([DCpoint;s.*Hr.'])))));
            h2 = makeStep(fftshift(real(ifft(makeSymmetric([DCpoint+0.001;s.*Hr.'])))));
            m = (h2(ts)-h1(ts))/0.001;
            b = h1(ts) - m*DCpoint;
            DCpoint = (0 - b)/m;
            err = abs(h1(ts) - 0);
            cnt = cnt+1;
        end 
endfunction

function [step] = makeStep(impulse)
    ustep = ones(length(impulse),1);    
    step = conv(ustep,impulse);
    step = step(1:length(impulse));
endfunction

function [symmetric] = makeSymmetric(nonsymmetric)
    % [symmetric] = makeSymmetric(nonsymmetric)
    % this takes the nonsymmetric frequency domain input and makes it
    % symmetric.
    %
    % The function assumes the DC point is in the nonsymmetric data

    symmetric_abs = [abs(nonsymmetric); flip(abs(nonsymmetric(2:end)))];
    symmetric_ang = [angle(nonsymmetric); -flip(angle(nonsymmetric(2:end)))];
    symmetric = symmetric_abs.*exp(1i.*symmetric_ang);  
endfunction

function TL = makeTL(gamma,len,zLine,z0)
  
        TL(1,1,:) = squeeze(((zLine.^2 - z0.^2).*sinh(gamma.*len))./((zLine.^2 + z0.^2).*sinh(gamma.*len) + 2.*z0.*zLine.*cosh(gamma.*len)));
        TL(2,1,:) = (2.*z0.*zLine)./((zLine.^2 + z0.^2).*sinh(gamma.*len)+2.*z0.*zLine.*cosh(gamma.*len));
        TL(1,2,:) = (2.*z0.*zLine)./((zLine.^2 + z0.^2).*sinh(gamma.*len)+2.*z0.*zLine.*cosh(gamma.*len));   
        TL(2,2,:) = ((zLine.^2 - z0.^2).*sinh(gamma.*len))./((zLine.^2 + z0.^2).*sinh(gamma.*len) + 2.*z0.*zLine.*cosh(gamma.*len)); 
endfunction

function out = simple_filter(f,f0)
    out = 1./(1 + 1i.*(f./f0).^4); 
endfunction

function abcd = stoabcd(s,z)
    s11 = squeeze(s(1,1,:)); s12 = squeeze(s(1,2,:));
    s21 = squeeze(s(2,1,:)); s22 = squeeze(s(2,2,:));
    
    abcd = zeros(2,2,length(s));
    abcd(1,1,:) = ((1 + s11).*(1 - s22) + s12.*s21)./(2*s21);      % A
    abcd(1,2,:) = z.*((1 + s11).*(1 + s22) - s12.*s21)./(2*s21);   % B
    abcd(2,1,:) = ((1 - s11).*(1 - s22) - s12.*s21)./((2*s21).*z); % C
    abcd(2,2,:) = ((1 - s11).*(1 + s22) + s12.*s21)./(2*s21);      % D
endfunction

function s = abcdtos(abcd,z)
    a = squeeze(abcd(1,1,:)); b = squeeze(abcd(1,2,:));
    c = squeeze(abcd(2,1,:)); d = squeeze(abcd(2,2,:));

    s = zeros(2,2,length(abcd));
    s(1,1,:) = (a + b./z - c.*z - d)./(a + b./z + c*z + d);
    s(1,2,:) = 2.*(a.*d - b.*c)./(a + b./z + c*z + d);
    s(2,1,:) = 2.*(a.*d - b.*c)./(a + b./z + c*z + d);
    s(2,2,:) = (-a + b./z - c.*z + d)./(a + b./z + c*z + d);   
endfunction

function out = db(in)
  out = 20.*log10(abs(squeeze(in)));
endfunction