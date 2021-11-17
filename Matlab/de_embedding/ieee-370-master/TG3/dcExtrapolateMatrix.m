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

function [new_freq, Sdata_interpolated] = dcExtrapolateMatrix(freq,Sdata,port_num)

min_freq = freq(1);
min_freq_norm = norm(Sdata(:,:,1));

for i=1:port_num
    for j=1:port_num
        sij = squeeze(Sdata(i,j,:));
        df = freq(2)-freq(1);
        new_freq = freq;
        
        %DC extrapolation
        if(freq(1) ~= 0)
            %disp('extrapolating to dc ...');
            [new_freq,sij] = dcextrapolation(freq,sij);   
        else
            sij(1) = real(sij(1));
        end
        
        %Interpolate data
        New_N = floor(new_freq(end)/df);
        new_freq_interp = df*(ceil(new_freq(1)/df):New_N)';
        sij = interpolation(new_freq,sij,new_freq_interp);
        new_freq = new_freq_interp;
        Sdata_interpolated(i,j,:) = sij;
    end
end

    for i = 1:min(length(new_freq), length(Sdata(1,1,:)))
        if (new_freq(i) < min_freq);
            [U, D, V] = svd(Sdata_interpolated(:,:,i));
            for k=1:port_num
                if (D(k,k) > max(1,min_freq_norm))
                    D(k,k) = max(1,min_freq_norm);
                end
            end
            Sdata_interpolated(:,:,i) = U*D*V';
        end
    end

