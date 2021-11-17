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

function [index] = alignsignals2(x,y)

 y=y';
 x=x';
 
 n= length(x);
 m = round(length(x)*0.1);
 mm = round(length(x)*0.01);
 xx = x(1:m);
 xx(m+1:m+mm) = x(n-mm+1:n);
 yy = y(1:m);
 yy(m+1:m+mm) = y(n-mm+1:n);
 x = xx;
 y=yy;
 
 yy = y(1:m);
 
[Mx,Ix] = max(x);
[My,Iy] = max(y);


% figure(4)
% plot(x); hold on;
% plot(y); hold on;


index = Ix-Iy;
yy = circshift(y,index);

% figure(3)
% plot(x); hold on;
% plot(yy); hold on;

n = min(1000,round(length(x)*0.1));
error = length(x);
error_ind = 0;
for k= -n+index:1:n+index
    yy = circshift(y,k);
    curr_error = norm(yy-x);
    if (error > curr_error)
        error_ind = k;
        error = curr_error;
    end
end
y = circshift(y,error_ind);

index = error_ind;
% figure(5)
% plot(x); hold on;
% plot(y); hold on;