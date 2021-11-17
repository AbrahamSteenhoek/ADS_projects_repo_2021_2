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

function [causality_metric_freq, reciprocity_metric_freq, passivity_metric_freq] = qualityCheckFrequencyDomain(Sdata,Nf,port_num)

%Passivity and Reciprocity
A = 1.00001;
B=0.1;
C = 10^(-6);
for i=1:Nf
    %passivity
    PM(i) = norm(Sdata(:,:,i));
    if(PM(i)>A)
        PW(i) = (PM(i) - A)/B;
    else
        PW(i) = 0;
    end
    
    %reciprocity
    RM(i) = 0;
    for k=1:port_num
        for m=1:port_num
            RM(i) = RM(i) + abs(Sdata(k,m,i) - Sdata(m,k,i));
        end
    end
    Ns = port_num*(port_num-1);
    RM(i) = RM(i)/Ns;
    if(RM(i)>C)
        RW(i) = (RM(i) - C)/B;
    else
        RW(i) = 0;
    end
end
passivity_metric_freq = max((Nf - sum(PW)),0)/Nf*100;
reciprocity_metric_freq = max((Nf - sum(RW)),0)/Nf*100;

%Causality
for i=1:port_num
    for j=1:port_num
        TotalR = 0;
        PositiveR = 0;
        for k=1:Nf-2
            Vn = Sdata(i,j,k+1) - Sdata(i,j,k);
            Vn1 = Sdata(i,j,k+2) - Sdata(i,j,k+1);
            R(k) = real(Vn1)*imag(Vn) - imag(Vn1)*real(Vn);
            if(R(k)>0)
                PositiveR = PositiveR + R(k);
            end
            TotalR = TotalR + abs(R(k));
        end
        CQM(i,j) = max(PositiveR/TotalR,0)*100;
    end
end
causality_metric_freq = min(min(CQM));



