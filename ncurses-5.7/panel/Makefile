# $Id: Makefile.in,v 1.52 2007/04/28 14:56:11 tom Exp $
##############################################################################
# Copyright (c) 1998-2006,2007 Free Software Foundation, Inc.                #
#                                                                            #
# Permission is hereby granted, free of charge, to any person obtaining a    #
# copy of this software and associated documentation files (the "Software"), #
# to deal in the Software without restriction, including without limitation  #
# the rights to use, copy, modify, merge, publish, distribute, distribute    #
# with modifications, sublicense, and/or sell copies of the Software, and to #
# permit persons to whom the Software is furnished to do so, subject to the  #
# following conditions:                                                      #
#                                                                            #
# The above copyright notice and this permission notice shall be included in #
# all copies or substantial portions of the Software.                        #
#                                                                            #
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR #
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   #
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    #
# THE ABOVE COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      #
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    #
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        #
# DEALINGS IN THE SOFTWARE.                                                  #
#                                                                            #
# Except as contained in this notice, the name(s) of the above copyright     #
# holders shall not be used in advertising or otherwise to promote the sale, #
# use or other dealings in this Software without prior written               #
# authorization.                                                             #
##############################################################################
#
# Author: Thomas E. Dickey 1996-on
#
# Makefile for panels source code.
#
# This makes the following:
#	libraries (normal/debug/profile/shared)
#
# The variable 'srcdir' refers to the source-distribution, and can be set with
# the configure script by "--srcdir=DIR".
#
# The rules are organized to produce the libraries for the configured models,

# turn off _all_ suffix rules; we'll generate our own
.SUFFIXES:

SHELL		= /bin/sh
THIS		= Makefile

x		= 
o		= .o

MODEL		= shared
DESTDIR		= 
srcdir		= .
prefix		= /usr
exec_prefix	= /usr
bindir		= ${exec_prefix}/bin
libdir		= ${exec_prefix}/lib
includedir	= ${prefix}/include
datadir		= ${prefix}/share

LIBTOOL		= 
LIBTOOL_CLEAN	= 
LIBTOOL_COMPILE	= 
LIBTOOL_LINK	= ${CC}
LIBTOOL_INSTALL	= 
LIBTOOL_UNINSTALL = 

INSTALL		= /usr/bin/install -c
INSTALL_LIB	= /usr/bin/install -c -m 644
INSTALL_PROG	= ${INSTALL}
INSTALL_DATA	= ${INSTALL} -m 644

AR		= /home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/bin/arm-unknown-linux-uclibcgnueabi-ar
AR_OPTS		= rv
AWK		= mawk
LD		= /home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/bin/arm-unknown-linux-uclibcgnueabi-ld --sysroot=/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir
LN_S		= ln -s

CC		= /home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/bin/arm-unknown-linux-uclibcgnueabi-gcc --sysroot=/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir
CPP		= /home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/bin/arm-unknown-linux-uclibcgnueabi-cpp --sysroot=/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir
CFLAGS		= -Os -pipe -Os  -mtune=arm926ej-s -march=armv5te -mabi=aapcs-linux -msoft-float -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -I/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/include -I/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/include  --param max-inline-insns-single=1200

CPPFLAGS	= -I../ncurses -DHAVE_CONFIG_H -I. -I../include  -D_GNU_SOURCE -DNDEBUG

CCFLAGS		= $(CPPFLAGS) $(CFLAGS)

CFLAGS_LIBTOOL	= $(CCFLAGS)
CFLAGS_NORMAL	= $(CCFLAGS)
CFLAGS_DEBUG	= $(CCFLAGS) -g -DTRACE
CFLAGS_PROFILE	= $(CCFLAGS) -pg
CFLAGS_SHARED	= $(CCFLAGS) -fPIC

CFLAGS_DEFAULT	= $(CFLAGS_SHARED)

LINK		= $(LIBTOOL_LINK)
LDFLAGS		= -L/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/lib -L/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/lib  

SHLIB_DIRS	= -L../lib
SHLIB_LIST	= $(SHLIB_DIRS) -lncurses 

MK_SHARED_LIB	= ${CC} ${CFLAGS} -shared -Wl,-soname,`basename $@ .${REL_VERSION}`.${ABI_VERSION},-stats,-lc -o $@

NCURSES_MAJOR	= 5
NCURSES_MINOR	= 7
REL_VERSION	= 5.7
ABI_VERSION	= 5

RANLIB		= /home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/bin/arm-unknown-linux-uclibcgnueabi-ranlib

LIBRARIES	=  ../lib/libpanel.so.${REL_VERSION} ../lib/libpanel.a

LINT		= 
LINT_OPTS	= 
LINT_LIBS	= -lpanel -lncurses 

AUTO_SRC	= \
		../include/panel.h

################################################################################
all \
libs \
install ::	$(AUTO_SRC) $(LIBRARIES)

sources:	$(AUTO_SRC)

$(DESTDIR)$(bindir) \
$(DESTDIR)$(libdir) :
	sh $(srcdir)/../mkdirs.sh $@

# make a copy to simplify include-paths while still keeping panel's include
# file in this directory.
../include/panel.h : $(srcdir)/panel.h
	-rm -f $@
	cp $(srcdir)/panel.h $@

PANEL_PRIV_H = \
	$(srcdir)/panel.priv.h \
	$(srcdir)/panel.h

tags:
	ctags *.[ch]

#TAGS:
#	etags *.[ch]

mostlyclean ::
	-rm -f core tags TAGS *~ *.bak *.i *.ln *.atac trace

clean :: mostlyclean
	-rm -f $(AUTO_SRC)

distclean :: clean
	-rm -f Makefile

realclean :: distclean

###############################################################################
# The remainder of this file is automatically generated during configuration
###############################################################################

# generated by mk-0th.awk
#   libname:    panel
#   subsets:    termlib+ext_tinfo+base+ext_funcs

.SUFFIXES: .c .cc .h .i .ii
.c.i :
	$(CPP) $(CPPFLAGS) $< >$@
.cc.ii :
	$(CPP) $(CPPFLAGS) $< >$@
.h.i :
	$(CPP) $(CPPFLAGS) $< >$@

C_SRC = \
	$(srcdir)/panel.c \
	$(srcdir)/p_above.c \
	$(srcdir)/p_below.c \
	$(srcdir)/p_bottom.c \
	$(srcdir)/p_delete.c \
	$(srcdir)/p_hide.c \
	$(srcdir)/p_hidden.c \
	$(srcdir)/p_move.c \
	$(srcdir)/p_new.c \
	$(srcdir)/p_replace.c \
	$(srcdir)/p_show.c \
	$(srcdir)/p_top.c \
	$(srcdir)/p_update.c \
	$(srcdir)/p_user.c \
	$(srcdir)/p_win.c

# Producing llib-lpanel is time-consuming, so there's no direct-dependency for
# it in the lintlib rule.  We'll only remove in the cleanest setup.
clean ::
	rm -f llib-lpanel.*

realclean ::
	rm -f llib-lpanel

llib-lpanel : $(C_SRC)
	cproto -a -l -DNCURSES_ENABLE_STDBOOL_H=0 -DLINT $(CPPFLAGS) $(C_SRC) >$@

lintlib :
	sh $(srcdir)/../misc/makellib panel $(CPPFLAGS)
lint :
	$(LINT) $(LINT_OPTS) $(CPPFLAGS) $(C_SRC) $(LINT_LIBS)

# generated by mk-1st.awk (subset=termlib+ext_tinfo+base+ext_funcs)
#  name:          panel
#  traces:        DEBUG
#  MODEL:         SHARED
#  model:         obj_s
#  prefix:        lib
#  suffix:        .so
#  subset:        termlib+ext_tinfo+base+ext_funcs
#  ShlibVer:      rel
#  ShlibVerInfix: no
#  TermlibRoot:   ncurses
#  TermlibSuffix: .so
#  ReLink:        yes
#  DoLinks:       yes
#  rmSoLocs:      no
#  ldconfig:      :
#  overwrite:     yes
#  depend:        ../include/ncurses_cfg.h ./panel.priv.h
#  host:          arm-unknown-linux-gnu

SHARED_OBJS = \
	../obj_s/panel$o \
	../obj_s/p_above$o \
	../obj_s/p_below$o \
	../obj_s/p_bottom$o \
	../obj_s/p_delete$o \
	../obj_s/p_hide$o \
	../obj_s/p_hidden$o \
	../obj_s/p_move$o \
	../obj_s/p_new$o \
	../obj_s/p_replace$o \
	../obj_s/p_show$o \
	../obj_s/p_top$o \
	../obj_s/p_update$o \
	../obj_s/p_user$o \
	../obj_s/p_win$o

$(SHARED_OBJS) : ../include/ncurses_cfg.h ./panel.priv.h

../lib/libpanel.so.$(REL_VERSION) : \
		../lib \
		$(SHARED_OBJS)
	@echo linking $@
	$(MK_SHARED_LIB) $(SHARED_OBJS) $(SHLIB_LIST) $(LDFLAGS)
	cd ../lib && (rm -f libpanel.so.$(ABI_VERSION); $(LN_S) libpanel.so.$(REL_VERSION) libpanel.so.$(ABI_VERSION); rm -f libpanel.so; $(LN_S) libpanel.so.$(ABI_VERSION) libpanel.so; )

install \
install.libs \
install.panel :: $(DESTDIR)$(libdir)/libpanel.so.$(REL_VERSION)

$(DESTDIR)$(libdir)/libpanel.so.$(REL_VERSION) : \
		$(DESTDIR)$(libdir) \
		$(SHARED_OBJS)
	@echo linking $@
	$(MK_SHARED_LIB) $(SHARED_OBJS) $(SHLIB_LIST) $(LDFLAGS)
	cd $(DESTDIR)$(libdir) && (rm -f libpanel.so.$(ABI_VERSION); $(LN_S) libpanel.so.$(REL_VERSION) libpanel.so.$(ABI_VERSION); rm -f libpanel.so; $(LN_S) libpanel.so.$(ABI_VERSION) libpanel.so; )

uninstall \
uninstall.libs \
uninstall.panel ::
	@echo uninstalling $(DESTDIR)$(libdir)/libpanel.so.$(REL_VERSION)
	-rm -f $(DESTDIR)$(libdir)/libpanel.so.$(REL_VERSION)
	-rm -f $(DESTDIR)$(libdir)/libpanel.so.$(ABI_VERSION)
	-rm -f $(DESTDIR)$(libdir)/libpanel.so

clean ::
	-rm -f ../lib/libpanel.so.$(REL_VERSION)
	-rm -f ../lib/libpanel.so.$(ABI_VERSION)
	-rm -f ../lib/libpanel.so

mostlyclean::
	-rm -f $(SHARED_OBJS)

# generated by mk-2nd.awk
#   model:      obj_s
#   MODEL:      SHARED
#   echo:       yes
#   subset:     termlib+ext_tinfo+base+ext_funcs
#   crenames:   yes
#   cxxrenames: 
#   traces:     DEBUG
#   srcdir:     .

../obj_s/panel$o :	$(srcdir)/panel.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/panel.c -o ../obj_s/panel$o

../obj_s/p_above$o :	$(srcdir)/p_above.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_above.c -o ../obj_s/p_above$o

../obj_s/p_below$o :	$(srcdir)/p_below.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_below.c -o ../obj_s/p_below$o

../obj_s/p_bottom$o :	$(srcdir)/p_bottom.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_bottom.c -o ../obj_s/p_bottom$o

../obj_s/p_delete$o :	$(srcdir)/p_delete.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_delete.c -o ../obj_s/p_delete$o

../obj_s/p_hide$o :	$(srcdir)/p_hide.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_hide.c -o ../obj_s/p_hide$o

../obj_s/p_hidden$o :	$(srcdir)/p_hidden.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_hidden.c -o ../obj_s/p_hidden$o

../obj_s/p_move$o :	$(srcdir)/p_move.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_move.c -o ../obj_s/p_move$o

../obj_s/p_new$o :	$(srcdir)/p_new.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_new.c -o ../obj_s/p_new$o

../obj_s/p_replace$o :	$(srcdir)/p_replace.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_replace.c -o ../obj_s/p_replace$o

../obj_s/p_show$o :	$(srcdir)/p_show.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_show.c -o ../obj_s/p_show$o

../obj_s/p_top$o :	$(srcdir)/p_top.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_top.c -o ../obj_s/p_top$o

../obj_s/p_update$o :	$(srcdir)/p_update.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_update.c -o ../obj_s/p_update$o

../obj_s/p_user$o :	$(srcdir)/p_user.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_user.c -o ../obj_s/p_user$o

../obj_s/p_win$o :	$(srcdir)/p_win.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_SHARED) -c ../panel/p_win.c -o ../obj_s/p_win$o


# generated by mk-1st.awk (subset=termlib+ext_tinfo+base+ext_funcs)
#  name:          panel
#  traces:        DEBUG
#  MODEL:         NORMAL
#  model:         objects
#  prefix:        lib
#  suffix:        .a
#  subset:        termlib+ext_tinfo+base+ext_funcs
#  ShlibVer:      rel
#  ShlibVerInfix: no
#  TermlibRoot:   ncurses
#  TermlibSuffix: .so
#  ReLink:        yes
#  DoLinks:       yes
#  rmSoLocs:      no
#  ldconfig:      :
#  overwrite:     yes
#  depend:        ../include/ncurses_cfg.h ./panel.priv.h
#  host:          arm-unknown-linux-gnu

NORMAL_OBJS = \
	../objects/panel$o \
	../objects/p_above$o \
	../objects/p_below$o \
	../objects/p_bottom$o \
	../objects/p_delete$o \
	../objects/p_hide$o \
	../objects/p_hidden$o \
	../objects/p_move$o \
	../objects/p_new$o \
	../objects/p_replace$o \
	../objects/p_show$o \
	../objects/p_top$o \
	../objects/p_update$o \
	../objects/p_user$o \
	../objects/p_win$o

$(NORMAL_OBJS) : ../include/ncurses_cfg.h ./panel.priv.h

../lib/libpanel.a : $(NORMAL_OBJS)
	$(AR) $(AR_OPTS) $@ $?
	$(RANLIB) $@

install \
install.libs \
install.panel :: $(DESTDIR)$(libdir) ../lib/libpanel.a
	@echo installing ../lib/libpanel.a as $(DESTDIR)$(libdir)/libpanel.a
	$(INSTALL_DATA) ../lib/libpanel.a $(DESTDIR)$(libdir)/libpanel.a
	$(RANLIB) $(DESTDIR)$(libdir)/libpanel.a

uninstall \
uninstall.libs \
uninstall.panel ::
	@echo uninstalling $(DESTDIR)$(libdir)/libpanel.a
	-@rm -f $(DESTDIR)$(libdir)/libpanel.a

clean ::
	-rm -f ../lib/libpanel.a
	-rm -f ../lib/
	-rm -f ../lib/libpanel.a

mostlyclean::
	-rm -f $(NORMAL_OBJS)

# generated by mk-2nd.awk
#   model:      objects
#   MODEL:      NORMAL
#   echo:       yes
#   subset:     termlib+ext_tinfo+base+ext_funcs
#   crenames:   yes
#   cxxrenames: 
#   traces:     DEBUG
#   srcdir:     .

../objects/panel$o :	$(srcdir)/panel.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/panel.c -o ../objects/panel$o

../objects/p_above$o :	$(srcdir)/p_above.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_above.c -o ../objects/p_above$o

../objects/p_below$o :	$(srcdir)/p_below.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_below.c -o ../objects/p_below$o

../objects/p_bottom$o :	$(srcdir)/p_bottom.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_bottom.c -o ../objects/p_bottom$o

../objects/p_delete$o :	$(srcdir)/p_delete.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_delete.c -o ../objects/p_delete$o

../objects/p_hide$o :	$(srcdir)/p_hide.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_hide.c -o ../objects/p_hide$o

../objects/p_hidden$o :	$(srcdir)/p_hidden.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_hidden.c -o ../objects/p_hidden$o

../objects/p_move$o :	$(srcdir)/p_move.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_move.c -o ../objects/p_move$o

../objects/p_new$o :	$(srcdir)/p_new.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_new.c -o ../objects/p_new$o

../objects/p_replace$o :	$(srcdir)/p_replace.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_replace.c -o ../objects/p_replace$o

../objects/p_show$o :	$(srcdir)/p_show.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_show.c -o ../objects/p_show$o

../objects/p_top$o :	$(srcdir)/p_top.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_top.c -o ../objects/p_top$o

../objects/p_update$o :	$(srcdir)/p_update.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_update.c -o ../objects/p_update$o

../objects/p_user$o :	$(srcdir)/p_user.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_user.c -o ../objects/p_user$o

../objects/p_win$o :	$(srcdir)/p_win.c \
			$(PANEL_PRIV_H)
	$(LIBTOOL_COMPILE) $(CC) $(CFLAGS_NORMAL) -c ../panel/p_win.c -o ../objects/p_win$o


# generated by mk-hdr.awk
#  subset:     termlib+ext_tinfo+base+ext_funcs
#  compat:     yes

${DESTDIR}${includedir} :
	sh ${srcdir}/../mkdirs.sh $@

install \
install.libs \
install.includes :: ${AUTO_SRC} ${DESTDIR}${includedir} \
		$(srcdir)/panel.h
	@ (cd ${DESTDIR}${includedir} && rm -f panel.h) ; ../headers.sh ${INSTALL_DATA} ${DESTDIR}${includedir} ${srcdir} $(srcdir)/panel.h

uninstall \
uninstall.libs \
uninstall.includes ::
	-@ (cd ${DESTDIR}${includedir} && rm -f panel.h)
depend : ${AUTO_SRC}
	makedepend -- ${CPPFLAGS} -- ${C_SRC}

# DO NOT DELETE THIS LINE -- make depend depends on it.
