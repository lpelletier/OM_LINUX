# Generated automatically from Makefile.in by configure.
# Makefile.in generated automatically by automake 1.5 from Makefile.am.

# Copyright 1994, 1995, 1996, 1997, 1998, 1999, 2000, 2001
# Free Software Foundation, Inc.
# This Makefile.in is free software; the Free Software Foundation
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY, to the extent permitted by law; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.



# This is the auto-Makefile for the lib subdirectory of Taylor UUCP.

SHELL = /bin/sh

srcdir = .
top_srcdir = ..
prefix = /usr
exec_prefix = /usr

bindir = ${exec_prefix}/bin
sbindir = ${exec_prefix}/sbin
libexecdir = ${exec_prefix}/libexec
datadir = ${prefix}/share
sysconfdir = /etc
sharedstatedir = ${prefix}/com
localstatedir = ${prefix}/var
libdir = ${exec_prefix}/lib
infodir = ${prefix}/info
mandir = ${prefix}/man
includedir = ${prefix}/include
oldincludedir = /usr/include
pkgdatadir = $(datadir)/uucp
pkglibdir = $(libdir)/uucp
pkgincludedir = $(includedir)/uucp
top_builddir = ..

ACLOCAL = ${SHELL} /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/missing --run aclocal
AUTOCONF = ${SHELL} /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/missing --run autoconf
AUTOMAKE = ${SHELL} /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/missing --run automake
AUTOHEADER = ${SHELL} /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/missing --run autoheader

INSTALL = /usr/bin/install -c
INSTALL_PROGRAM = ${INSTALL}
INSTALL_DATA = ${INSTALL} -m 644
INSTALL_SCRIPT = ${INSTALL_PROGRAM}
INSTALL_HEADER = $(INSTALL_DATA)
transform = s,x,x,
NORMAL_INSTALL = :
PRE_INSTALL = :
POST_INSTALL = :
NORMAL_UNINSTALL = :
PRE_UNINSTALL = :
POST_UNINSTALL = :
AMTAR = ${SHELL} /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/missing --run tar
AR = /home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/bin/arm-unknown-linux-uclibcgnueabi-ar
AWK = mawk
CC = /home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/bin/arm-unknown-linux-uclibcgnueabi-gcc --sysroot=/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir
DEPDIR = .deps
EXEEXT = 
INSTALL_STRIP_PROGRAM = ${SHELL} $(install_sh) -c -s
LIBOBJS = 
LN_S = ln -s
MAINT = #
NEWCONFIGDIR = ${prefix}/conf/uucp
OBJEXT = o
OLDCONFIGDIR = /usr/lib/uucp
OWNER = uucp
PACKAGE = uucp
POUNDBANG = yes
RANLIB = /home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/bin/arm-unknown-linux-uclibcgnueabi-ranlib
UNIXOBJS = 
VERSION = 1.07
WARN_CFLAGS = -W -Wall -Wstrict-prototypes -Wmissing-prototypes
am__include = include
am__quote = 
install_sh = /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/install-sh

noinst_LIBRARIES = libuucp.a

libuucp_a_SOURCES = buffer.c crc.c debug.c escape.c getopt.c getop1.c \
	parse.c quote.c quotes.c spool.c status.c xfree.c xmall.c xreall.c

libuucp_a_LIBADD = $(LIBOBJS)

AM_CFLAGS = -I.. -I$(srcdir)/.. $(WARN_CFLAGS)
subdir = lib
mkinstalldirs = $(SHELL) $(top_srcdir)/mkinstalldirs
CONFIG_HEADER = $(top_builddir)/config.h
CONFIG_CLEAN_FILES =
LIBRARIES = $(noinst_LIBRARIES)

libuucp_a_AR = $(AR) cru
libuucp_a_DEPENDENCIES = 
am_libuucp_a_OBJECTS = buffer.$(OBJEXT) crc.$(OBJEXT) debug.$(OBJEXT) \
	escape.$(OBJEXT) getopt.$(OBJEXT) getop1.$(OBJEXT) \
	parse.$(OBJEXT) quote.$(OBJEXT) quotes.$(OBJEXT) \
	spool.$(OBJEXT) status.$(OBJEXT) xfree.$(OBJEXT) \
	xmall.$(OBJEXT) xreall.$(OBJEXT)
libuucp_a_OBJECTS = $(am_libuucp_a_OBJECTS)

DEFS = -DHAVE_CONFIG_H
DEFAULT_INCLUDES =  -I. -I$(srcdir) -I$(top_builddir)
CPPFLAGS = 
LDFLAGS = -L/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/lib -L/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/lib
LIBS = -lnsl 
depcomp = $(SHELL) $(top_srcdir)/depcomp
DEP_FILES = $(DEPDIR)/bsrch.Po $(DEPDIR)/buffer.Po \
	$(DEPDIR)/bzero.Po $(DEPDIR)/crc.Po \
	$(DEPDIR)/debug.Po $(DEPDIR)/escape.Po \
	$(DEPDIR)/getlin.Po $(DEPDIR)/getop1.Po \
	$(DEPDIR)/getopt.Po $(DEPDIR)/memchr.Po \
	$(DEPDIR)/memcmp.Po $(DEPDIR)/memcpy.Po \
	$(DEPDIR)/parse.Po $(DEPDIR)/quote.Po \
	$(DEPDIR)/quotes.Po $(DEPDIR)/spool.Po \
	$(DEPDIR)/status.Po $(DEPDIR)/strcas.Po \
	$(DEPDIR)/strchr.Po $(DEPDIR)/strdup.Po \
	$(DEPDIR)/strncs.Po $(DEPDIR)/strrch.Po \
	$(DEPDIR)/strstr.Po $(DEPDIR)/strtol.Po \
	$(DEPDIR)/strtou.Po $(DEPDIR)/xfree.Po \
	$(DEPDIR)/xmall.Po $(DEPDIR)/xreall.Po
COMPILE = $(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) \
	$(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS)
CCLD = $(CC)
LINK = $(CCLD) $(AM_CFLAGS) $(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
CFLAGS = -Os -pipe -Os  -mtune=arm926ej-s -march=armv5te -mabi=aapcs-linux -msoft-float -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -I/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/include -I/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/include
DIST_SOURCES = $(libuucp_a_SOURCES)
DIST_COMMON = Makefile.am Makefile.in bsrch.c bzero.c getlin.c memchr.c \
	memcmp.c memcpy.c strcas.c strchr.c strdup.c strncs.c strrch.c \
	strstr.c strtol.c strtou.c
SOURCES = $(libuucp_a_SOURCES)

all: all-am

.SUFFIXES:
.SUFFIXES: .c .o .obj
$(srcdir)/Makefile.in: # Makefile.am  $(top_srcdir)/configure.in $(ACLOCAL_M4)
	cd $(top_srcdir) && \
	  $(AUTOMAKE) --gnu  lib/Makefile
Makefile: # $(srcdir)/Makefile.in  $(top_builddir)/config.status
	cd $(top_builddir) && \
	  CONFIG_HEADERS= CONFIG_LINKS= \
	  CONFIG_FILES=$(subdir)/$@ $(SHELL) ./config.status

AR = ar

clean-noinstLIBRARIES:
	-test -z "$(noinst_LIBRARIES)" || rm -f $(noinst_LIBRARIES)
libuucp.a: $(libuucp_a_OBJECTS) $(libuucp_a_DEPENDENCIES) 
	-rm -f libuucp.a
	$(libuucp_a_AR) libuucp.a $(libuucp_a_OBJECTS) $(libuucp_a_LIBADD)
	$(RANLIB) libuucp.a

mostlyclean-compile:
	-rm -f *.$(OBJEXT) core *.core

distclean-compile:
	-rm -f *.tab.c

include $(DEPDIR)/bsrch.Po
include $(DEPDIR)/buffer.Po
include $(DEPDIR)/bzero.Po
include $(DEPDIR)/crc.Po
include $(DEPDIR)/debug.Po
include $(DEPDIR)/escape.Po
include $(DEPDIR)/getlin.Po
include $(DEPDIR)/getop1.Po
include $(DEPDIR)/getopt.Po
include $(DEPDIR)/memchr.Po
include $(DEPDIR)/memcmp.Po
include $(DEPDIR)/memcpy.Po
include $(DEPDIR)/parse.Po
include $(DEPDIR)/quote.Po
include $(DEPDIR)/quotes.Po
include $(DEPDIR)/spool.Po
include $(DEPDIR)/status.Po
include $(DEPDIR)/strcas.Po
include $(DEPDIR)/strchr.Po
include $(DEPDIR)/strdup.Po
include $(DEPDIR)/strncs.Po
include $(DEPDIR)/strrch.Po
include $(DEPDIR)/strstr.Po
include $(DEPDIR)/strtol.Po
include $(DEPDIR)/strtou.Po
include $(DEPDIR)/xfree.Po
include $(DEPDIR)/xmall.Po
include $(DEPDIR)/xreall.Po

distclean-depend:
	-rm -rf $(DEPDIR)

.c.o:
	source='$<' object='$@' libtool=no \
	depfile='$(DEPDIR)/$*.Po' tmpdepfile='$(DEPDIR)/$*.TPo' \
	$(CCDEPMODE) $(depcomp) \
	$(COMPILE) -c `test -f $< || echo '$(srcdir)/'`$<

.c.obj:
	source='$<' object='$@' libtool=no \
	depfile='$(DEPDIR)/$*.Po' tmpdepfile='$(DEPDIR)/$*.TPo' \
	$(CCDEPMODE) $(depcomp) \
	$(COMPILE) -c `cygpath -w $<`
CCDEPMODE = depmode=gcc3
uninstall-info-am:

tags: TAGS

ID: $(HEADERS) $(SOURCES) $(LISP) $(TAGS_FILES)
	list='$(SOURCES) $(HEADERS) $(TAGS_FILES)'; \
	unique=`for i in $$list; do \
	    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
	  done | \
	  $(AWK) '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	mkid -fID $$unique $(LISP)

TAGS:  $(HEADERS) $(SOURCES)  $(TAGS_DEPENDENCIES) \
		$(TAGS_FILES) $(LISP)
	tags=; \
	here=`pwd`; \
	list='$(SOURCES) $(HEADERS) $(TAGS_FILES)'; \
	unique=`for i in $$list; do \
	    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
	  done | \
	  $(AWK) '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	test -z "$(ETAGS_ARGS)$$unique$(LISP)$$tags" \
	  || etags $(ETAGS_ARGS) $$tags  $$unique $(LISP)

GTAGS:
	here=`CDPATH=: && cd $(top_builddir) && pwd` \
	  && cd $(top_srcdir) \
	  && gtags -i $(GTAGS_ARGS) $$here

distclean-tags:
	-rm -f TAGS ID GTAGS GRTAGS GSYMS GPATH

DISTFILES = $(DIST_COMMON) $(DIST_SOURCES) $(TEXINFOS) $(EXTRA_DIST)

top_distdir = ..
distdir = $(top_distdir)/$(PACKAGE)-$(VERSION)

distdir: $(DISTFILES)
	@for file in $(DISTFILES); do \
	  if test -f $$file; then d=.; else d=$(srcdir); fi; \
	  dir=`echo "$$file" | sed -e 's,/[^/]*$$,,'`; \
	  if test "$$dir" != "$$file" && test "$$dir" != "."; then \
	    $(mkinstalldirs) "$(distdir)/$$dir"; \
	  fi; \
	  if test -d $$d/$$file; then \
	    cp -pR $$d/$$file $(distdir) \
	    || exit 1; \
	  else \
	    test -f $(distdir)/$$file \
	    || cp -p $$d/$$file $(distdir)/$$file \
	    || exit 1; \
	  fi; \
	done
check-am: all-am
check: check-am
all-am: Makefile $(LIBRARIES)

installdirs:

install: install-am
install-exec: install-exec-am
install-data: install-data-am
uninstall: uninstall-am

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am

installcheck: installcheck-am
install-strip:
	$(MAKE) $(AM_MAKEFLAGS) INSTALL_PROGRAM="$(INSTALL_STRIP_PROGRAM)" \
	  `test -z '$(STRIP)' || \
	    echo "INSTALL_PROGRAM_ENV=STRIPPROG='$(STRIP)'"` install
mostlyclean-generic:

clean-generic:

distclean-generic:
	-rm -f Makefile $(CONFIG_CLEAN_FILES) stamp-h stamp-h[0-9]*

maintainer-clean-generic:
	@echo "This command is intended for maintainers to use"
	@echo "it deletes files that may require special tools to rebuild."
clean: clean-am

clean-am: clean-generic clean-noinstLIBRARIES mostlyclean-am

distclean: distclean-am

distclean-am: clean-am distclean-compile distclean-depend \
	distclean-generic distclean-tags

dvi: dvi-am

dvi-am:

info: info-am

info-am:

install-data-am:

install-exec-am:

install-info: install-info-am

install-man:

installcheck-am:

maintainer-clean: maintainer-clean-am

maintainer-clean-am: distclean-am maintainer-clean-generic

mostlyclean: mostlyclean-am

mostlyclean-am: mostlyclean-compile mostlyclean-generic

uninstall-am: uninstall-info-am

.PHONY: GTAGS all all-am check check-am clean clean-generic \
	clean-noinstLIBRARIES distclean distclean-compile \
	distclean-depend distclean-generic distclean-tags distdir dvi \
	dvi-am info info-am install install-am install-data \
	install-data-am install-exec install-exec-am install-info \
	install-info-am install-man install-strip installcheck \
	installcheck-am installdirs maintainer-clean \
	maintainer-clean-generic mostlyclean mostlyclean-compile \
	mostlyclean-generic tags uninstall uninstall-am \
	uninstall-info-am

# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
