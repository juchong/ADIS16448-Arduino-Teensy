%%
%  December 2017
%  Author: Juan Jose Chong <juan.chong@analog.com>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  crc16adis.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   This function calculates a CRC16 check on data generated from an
%   ADIS16448 IMU when using burst mode (MSC_CTRL[4] = 1). This implementation 
%   is a variant of the CCITT X.25 spec. 
%
%   Permission is hereby granted, free of charge, to any person obtaining
%   a copy of this software and associated documentation files (the
%   "Software"), to deal in the Software without restriction, including
%   without limitation the rights to use, copy, modify, merge, publish,
%   distribute, sublicense, and/or sell copies of the Software, and to
%   permit persons to whom the Software is furnished to do so, subject to
%   the following conditions:
% 
%   The above copyright notice and this permission notice shall be
%   included in all copies or substantial portions of the Software.
% 
%   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
%   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
%   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
%   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
%   LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
%   OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
%   WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function crc = crc16adis(data)

%Example Data (LSB-first) - Result should be 0x9AF4 (39668):
%data = [15 0 248 255 10 0 65 254 204 255 86 4 39 2 163 253 18 244 60 193 108 255];

poly = uint16(4129); %0x1021
crc = uint16(65535); %0xFFFF

for j = 1:size(data,2)
    dat = uint16(data(j));
    for k = 1:8
        if bitxor((bitand(crc,1)),(bitand(dat,1)))
            crc = bitxor(bitsrl(crc,1),poly);
        else
            crc = bitsrl(crc,1);
        end
        dat = bitsrl(dat,1);
    end
end
crc = bitcmp(crc);
tmp = crc;
crc = bitor((bitsll(bitand(crc,255),8)),(bitand(bitsrl(tmp,8),255)),'uint16');
 
