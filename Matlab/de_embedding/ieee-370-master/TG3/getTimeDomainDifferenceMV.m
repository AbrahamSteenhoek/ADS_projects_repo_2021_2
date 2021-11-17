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

function [time_domain_difference_mv] = getTimeDomainDifferenceMV(time_domain1,time_domain2,port_num,datarate,time_,causality,delay_matrix)

N = length(time_);
dt = time_(2) - time_(1);
UI = 1/datarate/1e9/dt;
max_bits = 31;

for i=1:port_num
    for j=1:port_num
        for k=1:round(UI)
            diff(k) = 0;
            if (causality)
                if(i==j)
                    delay_num = 0;
                else
                    delay_num = delay_matrix(i,j);
                end
                for m = 0:(max_bits-1)
                    ind = delay_num - k - floor(m*UI);
                    if (ind <= 0)
                        ind = N + ind;
                    end
                    diff(k) = diff(k) + abs(time_domain2(i,j,ind)-time_domain1(i,j,ind));
                end 
            else
                [~, max_index] = max(time_domain1(i,j,:));
                last_index = max_index + max_bits*UI;
                lower_index = max_index - max_bits*UI;
                for m=0:floor(N/UI)-2
                    ind = k + floor(m*UI);
                    if(lower_index > 0)
                        condition = ((ind < last_index) && (ind > lower_index));
                    else
                        condition = ((ind < last_index) || (ind > N - lower_index));
                    end 
                    if condition
                        diff(k) = diff(k) + abs(time_domain1(i,j,ind) - time_domain2(i,j,ind));
                    end
                end 
            end
        end
        time_domain_difference_mv(i,j) = max(diff);
    end
end