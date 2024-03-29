# Copyright (C) 1989, 1990, 1991, 1992, 1993, 1994, 1995, 1996, 1997, 1998,
# 1999, 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007
# Free Software Foundation, Inc.

# This file is part of GDB.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

prefix = /usr
exec_prefix = /usr

host_alias = arm-unknown-linux-uclibcgnueabi
target_alias = arm-unknown-linux-uclibcgnueabi
program_transform_name = s,^,arm-unknown-linux-uclibcgnueabi-,
bindir = /usr/bin
libdir = ${exec_prefix}/lib
tooldir = $(libdir)/$(target_alias)

datadir = /usr/share
mandir = /usr/man
man1dir = $(mandir)/man1
man2dir = $(mandir)/man2
man3dir = $(mandir)/man3
man4dir = $(mandir)/man4
man5dir = $(mandir)/man5
man6dir = $(mandir)/man6
man7dir = $(mandir)/man7
man8dir = $(mandir)/man8
man9dir = $(mandir)/man9
infodir = /usr/info
htmldir = $(prefix)/html
includedir = /home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/include

SHELL = /bin/sh
EXEEXT = 

INSTALL = /usr/bin/install -c
INSTALL_PROGRAM = ${INSTALL}
INSTALL_DATA = ${INSTALL} -m 644

CC = /home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/bin/arm-unknown-linux-uclibcgnueabi-gcc --sysroot=/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir

# Directory containing source files.  Don't clean up the spacing,
# this exact string is matched for by the "configure" script.
srcdir = /home/leop/Code/armadeus-4.0/buildroot/output/toolchain/gdb-6.7.1/gdb/gdbserver
VPATH = /home/leop/Code/armadeus-4.0/buildroot/output/toolchain/gdb-6.7.1/gdb/gdbserver

# It is also possible that you will need to add -I/usr/include/sys to the
# CFLAGS section if your system doesn't have fcntl.h in /usr/include (which 
# is where it should be according to Posix).

# Set this up with gcc if you have gnu ld and the loader will print out
# line numbers for undefinded refs.
#CC-LD=gcc -static
CC-LD=${CC}

# Where is the "include" directory?  Traditionally ../include or ./include
INCLUDE_DIR =  ${srcdir}/../../include
INCLUDE_DEP = $$(INCLUDE_DIR)

# Where are the BFD library?
BFD_DIR = ../../bfd
BFD = $(BFD_DIR)/libbfd.a
BFD_SRC = $(srcdir)/$(BFD_DIR)
BFD_CFLAGS = -I$(BFD_DIR) -I$(BFD_SRC)

# Where is the source dir for the READLINE library?  Traditionally in .. or .
# (For the binary library built from it, we use ${READLINE_DIR}${subdir}.)
READLINE_DIR = ${srcdir}/../readline
READLINE_DEP = $$(READLINE_DIR)

# All the includes used for CFLAGS and for lint.
# -I. for config files.
# -I${srcdir} for our headers.
# -I$(srcdir)/../regformats for regdef.h.
INCLUDE_CFLAGS = -I. -I${srcdir} -I$(srcdir)/../regformats -I$(INCLUDE_DIR)

# M{H,T}_CFLAGS, if defined, has host- and target-dependent CFLAGS
# from the config/ directory.
GLOBAL_CFLAGS = ${MT_CFLAGS} ${MH_CFLAGS}
#PROFILE_CFLAGS = -pg

WARN_CFLAGS = -Wall

# CFLAGS is specifically reserved for setting from the command line
# when running make.  I.E.  "make CFLAGS=-Wmissing-prototypes".
CFLAGS = -Os -pipe -Os  -mtune=arm926ej-s -march=armv5te -mabi=aapcs-linux -msoft-float -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -I/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/include -I/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/include

# INTERNAL_CFLAGS is the aggregate of all other *CFLAGS macros.
INTERNAL_CFLAGS =  $(WARN_CFLAGS) ${CFLAGS} ${GLOBAL_CFLAGS} \
	${PROFILE_CFLAGS} ${INCLUDE_CFLAGS} ${BFD_CFLAGS}

# LDFLAGS is specifically reserved for setting from the command line
# when running make.
LDFLAGS = -L/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/lib -L/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/lib
INTERNAL_LDFLAGS = $(LDFLAGS) -rdynamic

# Perhaps should come from parent Makefile
VERSION = gdbserver-4.12.3
DIST=gdb

LINT=/usr/5bin/lint
LINTFLAGS= $(BFD_CFLAGS)

# All source files that go into linking GDB remote server.

SFILES=	$(srcdir)/gdbreplay.c $(srcdir)/inferiors.c \
	$(srcdir)/mem-break.c $(srcdir)/proc-service.c $(srcdir)/regcache.c \
	$(srcdir)/remote-utils.c $(srcdir)/server.c $(srcdir)/target.c \
	$(srcdir)/thread-db.c $(srcdir)/utils.c \
	$(srcdir)/linux-arm-low.c $(srcdir)/linux-cris-low.c \
	$(srcdir)/linux-crisv32-low.c $(srcdir)/linux-i386-low.c \
	$(srcdir)/i387-fp.c \
	$(srcdir)/linux-ia64-low.c $(srcdir)/linux-low.c \
	$(srcdir)/linux-m32r-low.c \
	$(srcdir)/linux-m68k-low.c $(srcdir)/linux-mips-low.c \
	$(srcdir)/linux-ppc-low.c $(srcdir)/linux-ppc64-low.c \
	$(srcdir)/linux-s390-low.c \
	$(srcdir)/linux-sh-low.c $(srcdir)/linux-x86-64-low.c \
	$(srcdir)/win32-arm-low.c $(srcdir)/win32-i386-low.c \
	$(srcdir)/win32-low.c $(srcdir)/wincecompat.c

DEPFILES = reg-arm.o linux-low.o linux-arm-low.o 

SOURCES = $(SFILES)
TAGFILES = $(SOURCES) ${HFILES} ${ALLPARAM} ${POSSLIBS} 

OBS = inferiors.o regcache.o remote-utils.o server.o signals.o target.o \
	utils.o version.o \
	mem-break.o \
	$(XML_BUILTIN) \
	$(DEPFILES)
GDBSERVER_LIBS = 
XM_CLIBS = 

# XML files to compile in to gdbserver, if any.
XML_DIR = $(srcdir)/../features
XML_TARGET = 
XML_FILES = 
XML_BUILTIN = 

# Prevent Sun make from putting in the machine type.  Setting
# TARGET_ARCH to nothing works for SunOS 3, 4.0, but not for 4.1.
.c.o:
	${CC} -c ${INTERNAL_CFLAGS} $<

all: gdbserver$(EXEEXT) gdbreplay$(EXEEXT)

# Traditionally "install" depends on "all".  But it may be useful
# not to; for example, if the user has made some trivial change to a 
# source file and doesn't care about rebuilding or just wants to save the
# time it takes for make to check that all is up to date.
# install-only is intended to address that need.
install: all install-only
install-only: 
	n=`echo gdbserver | sed '$(program_transform_name)'`; \
	if [ x$$n = x ]; then n=gdbserver; else true; fi; \
	$(SHELL) $(srcdir)/../../mkinstalldirs $(DESTDIR)$(bindir); \
	$(INSTALL_PROGRAM) gdbserver$(EXEEXT) $(DESTDIR)$(bindir)/$$n$(EXEEXT); \
	$(SHELL) $(srcdir)/../../mkinstalldirs $(DESTDIR)$(man1dir); \
	$(INSTALL_DATA) $(srcdir)/gdbserver.1 $(DESTDIR)$(man1dir)/$$n.1

uninstall: force
	n=`echo gdbserver | sed '$(program_transform_name)'`; \
	if [ x$$n = x ]; then n=gdbserver; else true; fi; \
	rm -f $(bindir)/$$n $(DESTDIR)$(man1dir)/$$n.1

installcheck:
check:
info dvi pdf:
install-info:
install-pdf:
html:
install-html:
clean-info:

gdbserver$(EXEEXT): $(OBS) ${ADD_DEPS} ${CDEPS}
	rm -f gdbserver$(EXEEXT)
	${CC-LD} $(INTERNAL_CFLAGS) $(INTERNAL_LDFLAGS) -o gdbserver$(EXEEXT) $(OBS) \
	  $(GDBSERVER_LIBS) $(XM_CLIBS)

gdbreplay$(EXEEXT): gdbreplay.o
	rm -f gdbreplay$(EXEEXT)
	${CC-LD} $(INTERNAL_CFLAGS) $(INTERNAL_LDFLAGS) -o gdbreplay$(EXEEXT) gdbreplay.o \
	  $(XM_CLIBS)

# Put the proper machine-specific files first, so M-. on a machine
# specific routine gets the one for the correct machine.
# The xyzzy stuff below deals with empty DEPFILES
TAGS:	${TAGFILES}
	etags `find ${srcdir}/../config -name $(DEPRECATED_TM_FILE) -print` \
	  `find ${srcdir}/../config -name ${XM_FILE} -print` \
	  `find ${srcdir}/../config -name ${NAT_FILE} -print` \
	  `for i in yzzy ${DEPFILES}; do \
	     if [ x$$i != xyzzy ]; then \
	       echo ${srcdir}/$$i | sed -e 's/\.o$$/\.c/' ; \
	     fi; \
	   done` \
	  ${TAGFILES}
tags: TAGS

clean:
	rm -f *.o ${ADD_FILES} *~
	rm -f version.c
	rm -f gdbserver$(EXEEXT) gdbreplay$(EXEEXT) core make.log
	rm -f reg-arm.c reg-i386.c reg-ia64.c reg-m32r.c reg-m68k.c
	rm -f reg-ppc.c reg-sh.c reg-spu.c reg-x86-64.c reg-i386-linux.c
	rm -f reg-cris.c reg-crisv32.c reg-x86-64-linux.c
	rm -f arm-with-iwmmxt.c mips-linux.c mips64-linux.c
	rm -f xml-builtin.c stamp-xml target.xml

maintainer-clean realclean distclean: clean
	rm -f nm.h tm.h xm.h config.status config.h stamp-h config.log
	rm -f Makefile

config.h: stamp-h ; @true
stamp-h: config.in config.status
	CONFIG_FILES="" CONFIG_HEADERS=config.h:config.in $(SHELL) ./config.status

Makefile: Makefile.in config.status
	CONFIG_HEADERS="" $(SHELL) ./config.status

config.status: configure configure.srv
	$(SHELL) ./config.status --recheck

force:

version.c: Makefile $(srcdir)/../version.in
	rm -f version.c-tmp version.c
	echo '#include "server.h"' >> version.c-tmp
	echo 'const char version[] = "'"`sed q ${srcdir}/../version.in`"'";' >> version.c-tmp
	echo 'const char host_name[] = "$(host_alias)";' >> version.c-tmp
	mv version.c-tmp version.c
version.o: version.c $(server_h)

target.xml: $(XML_TARGET)
	rm -f target.xml
	cp $(XML_TARGET) target.xml

xml-builtin.c: stamp-xml; @true
stamp-xml: $(XML_DIR)/feature_to_c.sh Makefile $(XML_FILES)
	rm -f xml-builtin.tmp
	$(SHELL) $(XML_DIR)/feature_to_c.sh xml-builtin.tmp $(XML_FILES)
	$(SHELL) $(srcdir)/../../move-if-change xml-builtin.tmp xml-builtin.c
	echo stamp > stamp-xml

.PRECIOUS: target.xml xml-builtin.c

# GNU Make has an annoying habit of putting *all* the Makefile variables
# into the environment, unless you include this target as a circumvention.
# Rumor is that this will be fixed (and this target can be removed)
# in GNU Make 4.0.
.NOEXPORT:

# GNU Make 3.63 has a different problem: it keeps tacking command line
# overrides onto the definition of $(MAKE).  This variable setting
# will remove them.
MAKEOVERRIDES=

gdb_proc_service_h = $(srcdir)/gdb_proc_service.h
regdat_sh = $(srcdir)/../regformats/regdat.sh
regdef_h = $(srcdir)/../regformats/regdef.h
regcache_h = $(srcdir)/regcache.h
server_h = $(srcdir)/server.h $(regcache_h) config.h $(srcdir)/target.h \
		$(srcdir)/mem-break.h

inferiors.o: inferiors.c $(server_h)
mem-break.o: mem-break.c $(server_h)
proc-service.o: proc-service.c $(server_h) $(gdb_proc_service_h)
regcache.o: regcache.c $(server_h) $(regdef_h)
remote-utils.o: remote-utils.c terminal.h $(server_h)
server.o: server.c $(server_h)
target.o: target.c $(server_h)
thread-db.o: thread-db.c $(server_h) $(gdb_proc_service_h)
utils.o: utils.c $(server_h)

signals.o: ../signals/signals.c $(server_h)
	$(CC) -c $(CPPFLAGS) $(INTERNAL_CFLAGS) $< -DGDBSERVER

i387-fp.o: i387-fp.c $(server_h)

linux_low_h = $(srcdir)/linux-low.h

linux-low.o: linux-low.c $(linux_low_h) $(server_h)
	$(CC) -c $(CPPFLAGS) $(INTERNAL_CFLAGS) $< 

linux-arm-low.o: linux-arm-low.c $(linux_low_h) $(server_h) \
	$(gdb_proc_service_h)
linux-cris-low.o: linux-cris-low.c $(linux_low_h) $(server_h)
linux-crisv32-low.o: linux-crisv32-low.c $(linux_low_h) $(server_h)
linux-i386-low.o: linux-i386-low.c $(linux_low_h) $(server_h) \
	$(gdb_proc_service_h)
linux-ia64-low.o: linux-ia64-low.c $(linux_low_h) $(server_h)
linux-m32r-low.o: linux-m32r-low.c $(linux_low_h) $(server_h)
linux-mips-low.o: linux-mips-low.c $(linux_low_h) $(server_h) \
	$(gdb_proc_service_h)
linux-ppc-low.o: linux-ppc-low.c $(linux_low_h) $(server_h)
linux-ppc64-low.o: linux-ppc64-low.c $(linux_low_h) $(server_h)
linux-s390-low.o: linux-s390-low.c $(linux_low_h) $(server_h)
linux-sh-low.o: linux-sh-low.c $(linux_low_h) $(server_h)
linux-x86-64-low.o: linux-x86-64-low.c $(linux_low_h) $(server_h) \
	$(gdb_proc_service_h)

win32-low.o: win32-low.c $(server_h) $(regdef_h) $(regcache_h)

spu-low.o: spu-low.c $(server_h)

reg-arm.o : reg-arm.c $(regdef_h)
reg-arm.c : $(srcdir)/../regformats/reg-arm.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-arm.dat reg-arm.c
arm-with-iwmmxt.o : arm-with-iwmmxt.c $(regdef_h)
arm-with-iwmmxt.c : $(srcdir)/../regformats/arm-with-iwmmxt.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/arm-with-iwmmxt.dat arm-with-iwmmxt.c
reg-cris.o : reg-cris.c $(regdef_h)
reg-cris.c : $(srcdir)/../regformats/reg-cris.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-cris.dat reg-cris.c
reg-crisv32.o : reg-crisv32.c $(regdef_h)
reg-crisv32.c : $(srcdir)/../regformats/reg-crisv32.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-crisv32.dat reg-crisv32.c
reg-i386.o : reg-i386.c $(regdef_h)
reg-i386.c : $(srcdir)/../regformats/reg-i386.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-i386.dat reg-i386.c
reg-i386-linux.o : reg-i386-linux.c $(regdef_h)
reg-i386-linux.c : $(srcdir)/../regformats/reg-i386-linux.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-i386-linux.dat reg-i386-linux.c
reg-ia64.o : reg-ia64.c $(regdef_h)
reg-ia64.c : $(srcdir)/../regformats/reg-ia64.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-ia64.dat reg-ia64.c
reg-m32r.o : reg-m32r.c $(regdef_h)
reg-m32r.c : $(srcdir)/../regformats/reg-m32r.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-m32r.dat reg-m32r.c
reg-m68k.o : reg-m68k.c $(regdef_h)
reg-m68k.c : $(srcdir)/../regformats/reg-m68k.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-m68k.dat reg-m68k.c
mips-linux.o : mips-linux.c $(regdef_h)
mips-linux.c : $(srcdir)/../regformats/mips-linux.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/mips-linux.dat mips-linux.c
mips64-linux.o : mips64-linux.c $(regdef_h)
mips64-linux.c : $(srcdir)/../regformats/mips64-linux.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/mips64-linux.dat mips64-linux.c
reg-ppc.o : reg-ppc.c $(regdef_h)
reg-ppc.c : $(srcdir)/../regformats/reg-ppc.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-ppc.dat reg-ppc.c
reg-ppc64.o : reg-ppc64.c $(regdef_h)
reg-ppc64.c : $(srcdir)/../regformats/reg-ppc64.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-ppc64.dat reg-ppc64.c
reg-s390.o : reg-s390.c $(regdef_h)
reg-s390.c : $(srcdir)/../regformats/reg-s390.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-s390.dat reg-s390.c
reg-s390x.o : reg-s390x.c $(regdef_h)
reg-s390x.c : $(srcdir)/../regformats/reg-s390x.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-s390x.dat reg-s390x.c
reg-sh.o : reg-sh.c $(regdef_h)
reg-sh.c : $(srcdir)/../regformats/reg-sh.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-sh.dat reg-sh.c
reg-spu.o : reg-spu.c $(regdef_h)
reg-spu.c : $(srcdir)/../regformats/reg-spu.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-spu.dat reg-spu.c
reg-x86-64.o : reg-x86-64.c $(regdef_h)
reg-x86-64.c : $(srcdir)/../regformats/reg-x86-64.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-x86-64.dat reg-x86-64.c
reg-x86-64-linux.o : reg-x86-64-linux.c $(regdef_h)
reg-x86-64-linux.c : $(srcdir)/../regformats/reg-x86-64-linux.dat $(regdat_sh)
	sh $(regdat_sh) $(srcdir)/../regformats/reg-x86-64-linux.dat reg-x86-64-linux.c

# This is the end of "Makefile.in".
