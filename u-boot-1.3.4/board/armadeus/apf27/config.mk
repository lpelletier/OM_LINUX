#
# This config file is used for compilation of armadeus sources
#
# You might change location of U-Boot in memory by setting right TEXT_BASE.
# This allows for example having one copy located at the end of ram and stored
# in flash device and later on while developing use other location to test
# the code in RAM device only.
#

ifndef TEXT_BASE
TEXT_BASE = 0xAFF00000
endif
CONFIG_NAND_U_BOOT = y
