#
# (C) Copyright 2007
# Stefan Roese, DENX Software Engineering, sr@denx.de.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#
#
# This config file is used for compilation of armadeus apf27 board
#

#
# TEXT_BASE for SPL:
#
#
# This config file is used for compilation of armadeus sources
#
# You might change location of U-Boot in memory by setting right TEXT_BASE.
# This allows for example having one copy located at the end of ram and stored
# in flash device and later on while developing use other location to test
# the code in RAM device only.
#

TEXT_BASE = 0xa0000000

# PAD_TO used to generate a 2kByte binary needed for the combined image
# -> PAD_TO = TEXT_BASE + 0x0800
PAD_TO	= 0xa0000800
