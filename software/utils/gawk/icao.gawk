#
# icao.gawk
#
# Copyright (C) 2019-2020 Linar Yusupov
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

function ltrim(s) { sub(/^[ \t\r\n]+/, "", s); return s }
function rtrim(s) { sub(/[ \t\r\n]+$/, "", s); return s }
function trim(s) { return rtrim(ltrim(s)); }

BEGIN {FS=","; OFS=","}
{
  for (i = 1; i <= 4; i++)
  { 
    str = trim($i);

    if (index(str, "\"") == 1)
    {
      str = substr(str, 2);
    }
    if (index(str, "\"") == 1)
    {
      str = substr(str, 2);
    }

    rev_str = "";

    for (j = length(str); j > 0; j--) rev_str=(rev_str substr(str, j, 1));

    if (index(rev_str, "\"") == 1)
    {
      rev_str = substr(rev_str, 2);
    }

    ret_str = "";

    for (j = length(rev_str); j > 0; j--) ret_str=(ret_str substr(rev_str, j, 1));

    if (i == 1) {

      ret_str = strtonum("0x" ret_str);

    }

    $i = ret_str;
  }
  print $0;
}
