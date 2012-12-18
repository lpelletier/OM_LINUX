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



# This is the auto-Makefile for Taylor UUCP
# This is processed by automake to create Makefile.in.

SHELL = /bin/sh

srcdir = .
top_srcdir = .
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
top_builddir = .

ACLOCAL = ${SHELL} /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/missing --run aclocal
AUTOCONF = ${SHELL} /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/missing --run autoconf
AUTOMAKE = ${SHELL} /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/missing --run automake
AUTOHEADER = ${SHELL} /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/missing --run autoheader

INSTALL = /usr/bin/install -c
INSTALL_PROGRAM = ${INSTALL}
INSTALL_DATA = ${INSTALL} -m 644
INSTALL_SCRIPT = ${INSTALL}
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

SUBDIRS = lib uuconf unix

sbin_PROGRAMS = uucico uuxqt uuchk uuconv
sbin_SCRIPTS = uusched
bin_PROGRAMS = uux uucp uustat uuname uulog uupick cu
bin_SCRIPTS = uuto
noinst_PROGRAMS = tstuu
info_TEXINFOS = uucp.texi
man_MANS = uux.1 uucp.1 uustat.1 cu.1 uucico.8 uuxqt.8

# If you do not have the mkdir system call, this will install uudir as
# an suid root program.  This is necessary because invoking /bin/mkdir
# from an suid program will leave the directories owned by the wrong
# user.
uudirdir = 
#uudirdir = $(sbindir)/util
uudir_PROGRAMS = 
#uudir_PROGRAMS = uudir

AM_CFLAGS = $(WARN_CFLAGS) $(UUDIRFLAGS)

LDADD = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a

UUHEADERS = uucp.h uudefs.h uuconf.h policy.h system.h sysdep.h getopt.h

uucico_SOURCES = uucico.c trans.h trans.c send.c rec.c xcmd.c prot.h prot.c \
	protg.c protf.c prott.c prote.c proti.c protj.c proty.c protz.c \
	time.c log.c chat.c conn.h conn.c util.c copy.c $(UUHEADERS)

uuxqt_SOURCES = uuxqt.c util.c log.c copy.c $(UUHEADERS)
uux_SOURCES = uux.c util.c log.c copy.c $(UUHEADERS)
uucp_SOURCES = uucp.c util.c log.c copy.c $(UUHEADERS)
uustat_SOURCES = uustat.c util.c log.c copy.c $(UUHEADERS)
uuname_SOURCES = uuname.c log.c $(UUHEADERS)
uulog_SOURCES = uulog.c log.c $(UUHEADERS)
uupick_SOURCES = uupick.c log.c copy.c $(UUHEADERS)
cu_SOURCES = cu.h cu.c prot.c log.c chat.c conn.c copy.c $(UUHEADERS)
uuchk_SOURCES = uuchk.c $(UUHEADERS)
uuconv_SOURCES = uuconv.c $(UUHEADERS)
tstuu_SOURCES = tstuu.c
uudir_SOURCES = uudir.c

uuconv_CFLAGS = -I$(srcdir)/uuconf $(AM_CFLAGS)

CLEANFILES = uusched uuto

EXTRA_DIST = uusched.in uuto.in \
	cu.1 uucp.1 uustat.1 uux.1 uucico.8 uuxqt.8 \
	contrib sample

subdir = .
ACLOCAL_M4 = $(top_srcdir)/aclocal.m4
mkinstalldirs = $(SHELL) $(top_srcdir)/mkinstalldirs
CONFIG_HEADER = config.h
CONFIG_CLEAN_FILES =
bin_PROGRAMS = uux$(EXEEXT) uucp$(EXEEXT) uustat$(EXEEXT) \
	uuname$(EXEEXT) uulog$(EXEEXT) uupick$(EXEEXT) cu$(EXEEXT)
noinst_PROGRAMS = tstuu$(EXEEXT)
sbin_PROGRAMS = uucico$(EXEEXT) uuxqt$(EXEEXT) uuchk$(EXEEXT) \
	uuconv$(EXEEXT)
uudir_PROGRAMS =
#uudir_PROGRAMS = uudir$(EXEEXT)
PROGRAMS = $(bin_PROGRAMS) $(noinst_PROGRAMS) $(sbin_PROGRAMS) \
	$(uudir_PROGRAMS)

am_cu_OBJECTS = cu.$(OBJEXT) prot.$(OBJEXT) log.$(OBJEXT) chat.$(OBJEXT) \
	conn.$(OBJEXT) copy.$(OBJEXT)
cu_OBJECTS = $(am_cu_OBJECTS)
cu_LDADD = $(LDADD)
cu_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
cu_LDFLAGS =
am_tstuu_OBJECTS = tstuu.$(OBJEXT)
tstuu_OBJECTS = $(am_tstuu_OBJECTS)
tstuu_LDADD = $(LDADD)
tstuu_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
tstuu_LDFLAGS =
am_uuchk_OBJECTS = uuchk.$(OBJEXT)
uuchk_OBJECTS = $(am_uuchk_OBJECTS)
uuchk_LDADD = $(LDADD)
uuchk_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uuchk_LDFLAGS =
am_uucico_OBJECTS = uucico.$(OBJEXT) trans.$(OBJEXT) send.$(OBJEXT) \
	rec.$(OBJEXT) xcmd.$(OBJEXT) prot.$(OBJEXT) protg.$(OBJEXT) \
	protf.$(OBJEXT) prott.$(OBJEXT) prote.$(OBJEXT) proti.$(OBJEXT) \
	protj.$(OBJEXT) proty.$(OBJEXT) protz.$(OBJEXT) time.$(OBJEXT) \
	log.$(OBJEXT) chat.$(OBJEXT) conn.$(OBJEXT) util.$(OBJEXT) \
	copy.$(OBJEXT)
uucico_OBJECTS = $(am_uucico_OBJECTS)
uucico_LDADD = $(LDADD)
uucico_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uucico_LDFLAGS =
am_uuconv_OBJECTS = uuconv-uuconv.$(OBJEXT)
uuconv_OBJECTS = $(am_uuconv_OBJECTS)
uuconv_LDADD = $(LDADD)
uuconv_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uuconv_LDFLAGS =
am_uucp_OBJECTS = uucp.$(OBJEXT) util.$(OBJEXT) log.$(OBJEXT) \
	copy.$(OBJEXT)
uucp_OBJECTS = $(am_uucp_OBJECTS)
uucp_LDADD = $(LDADD)
uucp_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uucp_LDFLAGS =
am_uudir_OBJECTS = uudir.$(OBJEXT)
uudir_OBJECTS = $(am_uudir_OBJECTS)
uudir_LDADD = $(LDADD)
uudir_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uudir_LDFLAGS =
am_uulog_OBJECTS = uulog.$(OBJEXT) log.$(OBJEXT)
uulog_OBJECTS = $(am_uulog_OBJECTS)
uulog_LDADD = $(LDADD)
uulog_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uulog_LDFLAGS =
am_uuname_OBJECTS = uuname.$(OBJEXT) log.$(OBJEXT)
uuname_OBJECTS = $(am_uuname_OBJECTS)
uuname_LDADD = $(LDADD)
uuname_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uuname_LDFLAGS =
am_uupick_OBJECTS = uupick.$(OBJEXT) log.$(OBJEXT) copy.$(OBJEXT)
uupick_OBJECTS = $(am_uupick_OBJECTS)
uupick_LDADD = $(LDADD)
uupick_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uupick_LDFLAGS =
am_uustat_OBJECTS = uustat.$(OBJEXT) util.$(OBJEXT) log.$(OBJEXT) \
	copy.$(OBJEXT)
uustat_OBJECTS = $(am_uustat_OBJECTS)
uustat_LDADD = $(LDADD)
uustat_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uustat_LDFLAGS =
am_uux_OBJECTS = uux.$(OBJEXT) util.$(OBJEXT) log.$(OBJEXT) \
	copy.$(OBJEXT)
uux_OBJECTS = $(am_uux_OBJECTS)
uux_LDADD = $(LDADD)
uux_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uux_LDFLAGS =
am_uuxqt_OBJECTS = uuxqt.$(OBJEXT) util.$(OBJEXT) log.$(OBJEXT) \
	copy.$(OBJEXT)
uuxqt_OBJECTS = $(am_uuxqt_OBJECTS)
uuxqt_LDADD = $(LDADD)
uuxqt_DEPENDENCIES = unix/libunix.a uuconf/libuuconf.a lib/libuucp.a
uuxqt_LDFLAGS =
SCRIPTS = $(bin_SCRIPTS) $(sbin_SCRIPTS)


DEFS = -DHAVE_CONFIG_H
DEFAULT_INCLUDES =  -I. -I$(srcdir) -I.
CPPFLAGS = 
LDFLAGS = -L/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/lib -L/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/lib
LIBS = -lnsl 
depcomp = $(SHELL) $(top_srcdir)/depcomp
DEP_FILES = $(DEPDIR)/chat.Po $(DEPDIR)/conn.Po \
	$(DEPDIR)/copy.Po $(DEPDIR)/cu.Po $(DEPDIR)/log.Po \
	$(DEPDIR)/prot.Po $(DEPDIR)/prote.Po \
	$(DEPDIR)/protf.Po $(DEPDIR)/protg.Po \
	$(DEPDIR)/proti.Po $(DEPDIR)/protj.Po \
	$(DEPDIR)/prott.Po $(DEPDIR)/proty.Po \
	$(DEPDIR)/protz.Po $(DEPDIR)/rec.Po \
	$(DEPDIR)/send.Po $(DEPDIR)/time.Po \
	$(DEPDIR)/trans.Po $(DEPDIR)/tstuu.Po \
	$(DEPDIR)/util.Po $(DEPDIR)/uuchk.Po \
	$(DEPDIR)/uucico.Po $(DEPDIR)/uuconv-uuconv.Po \
	$(DEPDIR)/uucp.Po $(DEPDIR)/uudir.Po \
	$(DEPDIR)/uulog.Po $(DEPDIR)/uuname.Po \
	$(DEPDIR)/uupick.Po $(DEPDIR)/uustat.Po \
	$(DEPDIR)/uux.Po $(DEPDIR)/uuxqt.Po \
	$(DEPDIR)/xcmd.Po
COMPILE = $(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) \
	$(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS)
CCLD = $(CC)
LINK = $(CCLD) $(AM_CFLAGS) $(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
CFLAGS = -Os -pipe -Os  -mtune=arm926ej-s -march=armv5te -mabi=aapcs-linux -msoft-float -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -I/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/usr/include -I/home/leop/Code/armadeus-4.0/buildroot/output/build/staging_dir/include
DIST_SOURCES = $(cu_SOURCES) $(tstuu_SOURCES) $(uuchk_SOURCES) \
	$(uucico_SOURCES) $(uuconv_SOURCES) $(uucp_SOURCES) \
	$(uudir_SOURCES) $(uulog_SOURCES) $(uuname_SOURCES) \
	$(uupick_SOURCES) $(uustat_SOURCES) $(uux_SOURCES) \
	$(uuxqt_SOURCES)
INFO_DEPS = uucp.info
DVIS = uucp.dvi
TEXINFOS = uucp.texi

NROFF = nroff
MANS = $(man_MANS)

RECURSIVE_TARGETS = info-recursive dvi-recursive install-info-recursive \
	uninstall-info-recursive all-recursive install-data-recursive \
	install-exec-recursive installdirs-recursive install-recursive \
	uninstall-recursive check-recursive installcheck-recursive
DIST_COMMON = README ./stamp-h.in AUTHORS COPYING ChangeLog INSTALL \
	Makefile.am Makefile.in NEWS TODO aclocal.m4 compile \
	config.h.in configure configure.in depcomp install-sh missing \
	mkinstalldirs texinfo.tex
DIST_SUBDIRS = $(SUBDIRS)
SOURCES = $(cu_SOURCES) $(tstuu_SOURCES) $(uuchk_SOURCES) $(uucico_SOURCES) $(uuconv_SOURCES) $(uucp_SOURCES) $(uudir_SOURCES) $(uulog_SOURCES) $(uuname_SOURCES) $(uupick_SOURCES) $(uustat_SOURCES) $(uux_SOURCES) $(uuxqt_SOURCES)

all: config.h
	$(MAKE) $(AM_MAKEFLAGS) all-recursive

.SUFFIXES:
.SUFFIXES: .c .dvi .info .o .obj .ps .texi
$(srcdir)/Makefile.in: # Makefile.am  $(top_srcdir)/configure.in $(ACLOCAL_M4)
	cd $(top_srcdir) && \
	  $(AUTOMAKE) --gnu  Makefile
Makefile: # $(srcdir)/Makefile.in  $(top_builddir)/config.status
	cd $(top_builddir) && \
	  CONFIG_HEADERS= CONFIG_LINKS= \
	  CONFIG_FILES=$@ $(SHELL) ./config.status

$(top_builddir)/config.status: $(srcdir)/configure $(CONFIG_STATUS_DEPENDENCIES)
	$(SHELL) ./config.status --recheck
$(srcdir)/configure: # $(srcdir)/configure.in $(ACLOCAL_M4) $(CONFIGURE_DEPENDENCIES)
	cd $(srcdir) && $(AUTOCONF)

$(ACLOCAL_M4): # configure.in 
	cd $(srcdir) && $(ACLOCAL) $(ACLOCAL_AMFLAGS)
config.h: stamp-h
	@if test ! -f $@; then \
		rm -f stamp-h; \
		$(MAKE) stamp-h; \
	else :; fi
stamp-h: $(srcdir)/config.h.in $(top_builddir)/config.status
	@rm -f stamp-h stamp-hT
	@echo timestamp > stamp-hT 2> /dev/null
	cd $(top_builddir) \
	  && CONFIG_FILES= CONFIG_HEADERS=config.h \
	     $(SHELL) ./config.status
	@mv stamp-hT stamp-h
$(srcdir)/config.h.in: # $(srcdir)/./stamp-h.in
	@if test ! -f $@; then \
		rm -f $(srcdir)/./stamp-h.in; \
		$(MAKE) $(srcdir)/./stamp-h.in; \
	else :; fi
$(srcdir)/./stamp-h.in: $(top_srcdir)/configure.in $(ACLOCAL_M4) 
	@rm -f $(srcdir)/./stamp-h.in $(srcdir)/./stamp-h.inT
	@echo timestamp > $(srcdir)/./stamp-h.inT 2> /dev/null
	cd $(top_srcdir) && $(AUTOHEADER)
	@mv $(srcdir)/./stamp-h.inT $(srcdir)/./stamp-h.in

distclean-hdr:
	-rm -f config.h
install-binPROGRAMS: $(bin_PROGRAMS)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(bindir)
	@list='$(bin_PROGRAMS)'; for p in $$list; do \
	  p1=`echo $$p|sed 's/$(EXEEXT)$$//'`; \
	  if test -f $$p \
	  ; then \
	    f=`echo $$p1|sed '$(transform);s/$$/$(EXEEXT)/'`; \
	   echo " $(INSTALL_PROGRAM_ENV) $(INSTALL_PROGRAM) $$p $(DESTDIR)$(bindir)/$$f"; \
	   $(INSTALL_PROGRAM_ENV) $(INSTALL_PROGRAM) $$p $(DESTDIR)$(bindir)/$$f; \
	  else :; fi; \
	done

uninstall-binPROGRAMS:
	@$(NORMAL_UNINSTALL)
	@list='$(bin_PROGRAMS)'; for p in $$list; do \
	  f=`echo $$p|sed 's/$(EXEEXT)$$//;$(transform);s/$$/$(EXEEXT)/'`; \
	  echo " rm -f $(DESTDIR)$(bindir)/$$f"; \
	  rm -f $(DESTDIR)$(bindir)/$$f; \
	done

clean-binPROGRAMS:
	-test -z "$(bin_PROGRAMS)" || rm -f $(bin_PROGRAMS)

clean-noinstPROGRAMS:
	-test -z "$(noinst_PROGRAMS)" || rm -f $(noinst_PROGRAMS)
install-sbinPROGRAMS: $(sbin_PROGRAMS)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(sbindir)
	@list='$(sbin_PROGRAMS)'; for p in $$list; do \
	  p1=`echo $$p|sed 's/$(EXEEXT)$$//'`; \
	  if test -f $$p \
	  ; then \
	    f=`echo $$p1|sed '$(transform);s/$$/$(EXEEXT)/'`; \
	   echo " $(INSTALL_PROGRAM_ENV) $(INSTALL_PROGRAM) $$p $(DESTDIR)$(sbindir)/$$f"; \
	   $(INSTALL_PROGRAM_ENV) $(INSTALL_PROGRAM) $$p $(DESTDIR)$(sbindir)/$$f; \
	  else :; fi; \
	done

uninstall-sbinPROGRAMS:
	@$(NORMAL_UNINSTALL)
	@list='$(sbin_PROGRAMS)'; for p in $$list; do \
	  f=`echo $$p|sed 's/$(EXEEXT)$$//;$(transform);s/$$/$(EXEEXT)/'`; \
	  echo " rm -f $(DESTDIR)$(sbindir)/$$f"; \
	  rm -f $(DESTDIR)$(sbindir)/$$f; \
	done

clean-sbinPROGRAMS:
	-test -z "$(sbin_PROGRAMS)" || rm -f $(sbin_PROGRAMS)
install-uudirPROGRAMS: $(uudir_PROGRAMS)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(uudirdir)
	@list='$(uudir_PROGRAMS)'; for p in $$list; do \
	  p1=`echo $$p|sed 's/$(EXEEXT)$$//'`; \
	  if test -f $$p \
	  ; then \
	    f=`echo $$p1|sed '$(transform);s/$$/$(EXEEXT)/'`; \
	   echo " $(INSTALL_PROGRAM_ENV) $(INSTALL_PROGRAM) $$p $(DESTDIR)$(uudirdir)/$$f"; \
	   $(INSTALL_PROGRAM_ENV) $(INSTALL_PROGRAM) $$p $(DESTDIR)$(uudirdir)/$$f; \
	  else :; fi; \
	done

uninstall-uudirPROGRAMS:
	@$(NORMAL_UNINSTALL)
	@list='$(uudir_PROGRAMS)'; for p in $$list; do \
	  f=`echo $$p|sed 's/$(EXEEXT)$$//;$(transform);s/$$/$(EXEEXT)/'`; \
	  echo " rm -f $(DESTDIR)$(uudirdir)/$$f"; \
	  rm -f $(DESTDIR)$(uudirdir)/$$f; \
	done

clean-uudirPROGRAMS:
	-test -z "$(uudir_PROGRAMS)" || rm -f $(uudir_PROGRAMS)
cu$(EXEEXT): $(cu_OBJECTS) $(cu_DEPENDENCIES) 
	@rm -f cu$(EXEEXT)
	$(LINK) $(cu_LDFLAGS) $(cu_OBJECTS) $(cu_LDADD) $(LIBS)
tstuu$(EXEEXT): $(tstuu_OBJECTS) $(tstuu_DEPENDENCIES) 
	@rm -f tstuu$(EXEEXT)
	$(LINK) $(tstuu_LDFLAGS) $(tstuu_OBJECTS) $(tstuu_LDADD) $(LIBS)
uuchk$(EXEEXT): $(uuchk_OBJECTS) $(uuchk_DEPENDENCIES) 
	@rm -f uuchk$(EXEEXT)
	$(LINK) $(uuchk_LDFLAGS) $(uuchk_OBJECTS) $(uuchk_LDADD) $(LIBS)
uucico$(EXEEXT): $(uucico_OBJECTS) $(uucico_DEPENDENCIES) 
	@rm -f uucico$(EXEEXT)
	$(LINK) $(uucico_LDFLAGS) $(uucico_OBJECTS) $(uucico_LDADD) $(LIBS)
uuconv-uuconv.$(OBJEXT): uuconv.c
uuconv$(EXEEXT): $(uuconv_OBJECTS) $(uuconv_DEPENDENCIES) 
	@rm -f uuconv$(EXEEXT)
	$(LINK) $(uuconv_LDFLAGS) $(uuconv_OBJECTS) $(uuconv_LDADD) $(LIBS)
uucp$(EXEEXT): $(uucp_OBJECTS) $(uucp_DEPENDENCIES) 
	@rm -f uucp$(EXEEXT)
	$(LINK) $(uucp_LDFLAGS) $(uucp_OBJECTS) $(uucp_LDADD) $(LIBS)
uudir$(EXEEXT): $(uudir_OBJECTS) $(uudir_DEPENDENCIES) 
	@rm -f uudir$(EXEEXT)
	$(LINK) $(uudir_LDFLAGS) $(uudir_OBJECTS) $(uudir_LDADD) $(LIBS)
uulog$(EXEEXT): $(uulog_OBJECTS) $(uulog_DEPENDENCIES) 
	@rm -f uulog$(EXEEXT)
	$(LINK) $(uulog_LDFLAGS) $(uulog_OBJECTS) $(uulog_LDADD) $(LIBS)
uuname$(EXEEXT): $(uuname_OBJECTS) $(uuname_DEPENDENCIES) 
	@rm -f uuname$(EXEEXT)
	$(LINK) $(uuname_LDFLAGS) $(uuname_OBJECTS) $(uuname_LDADD) $(LIBS)
uupick$(EXEEXT): $(uupick_OBJECTS) $(uupick_DEPENDENCIES) 
	@rm -f uupick$(EXEEXT)
	$(LINK) $(uupick_LDFLAGS) $(uupick_OBJECTS) $(uupick_LDADD) $(LIBS)
uustat$(EXEEXT): $(uustat_OBJECTS) $(uustat_DEPENDENCIES) 
	@rm -f uustat$(EXEEXT)
	$(LINK) $(uustat_LDFLAGS) $(uustat_OBJECTS) $(uustat_LDADD) $(LIBS)
uux$(EXEEXT): $(uux_OBJECTS) $(uux_DEPENDENCIES) 
	@rm -f uux$(EXEEXT)
	$(LINK) $(uux_LDFLAGS) $(uux_OBJECTS) $(uux_LDADD) $(LIBS)
uuxqt$(EXEEXT): $(uuxqt_OBJECTS) $(uuxqt_DEPENDENCIES) 
	@rm -f uuxqt$(EXEEXT)
	$(LINK) $(uuxqt_LDFLAGS) $(uuxqt_OBJECTS) $(uuxqt_LDADD) $(LIBS)
install-binSCRIPTS: $(bin_SCRIPTS)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(bindir)
	@list='$(bin_SCRIPTS)'; for p in $$list; do \
	  f="`echo $$p|sed '$(transform)'`"; \
	  if test -f $$p; then \
	    echo " $(INSTALL_SCRIPT) $$p $(DESTDIR)$(bindir)/$$f"; \
	    $(INSTALL_SCRIPT) $$p $(DESTDIR)$(bindir)/$$f; \
	  elif test -f $(srcdir)/$$p; then \
	    echo " $(INSTALL_SCRIPT) $(srcdir)/$$p $(DESTDIR)$(bindir)/$$f"; \
	    $(INSTALL_SCRIPT) $(srcdir)/$$p $(DESTDIR)$(bindir)/$$f; \
	  else :; fi; \
	done

uninstall-binSCRIPTS:
	@$(NORMAL_UNINSTALL)
	@list='$(bin_SCRIPTS)'; for p in $$list; do \
	  f="`echo $$p|sed '$(transform)'`"; \
	  echo " rm -f $(DESTDIR)$(bindir)/$$f"; \
	  rm -f $(DESTDIR)$(bindir)/$$f; \
	done
install-sbinSCRIPTS: $(sbin_SCRIPTS)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(sbindir)
	@list='$(sbin_SCRIPTS)'; for p in $$list; do \
	  f="`echo $$p|sed '$(transform)'`"; \
	  if test -f $$p; then \
	    echo " $(INSTALL_SCRIPT) $$p $(DESTDIR)$(sbindir)/$$f"; \
	    $(INSTALL_SCRIPT) $$p $(DESTDIR)$(sbindir)/$$f; \
	  elif test -f $(srcdir)/$$p; then \
	    echo " $(INSTALL_SCRIPT) $(srcdir)/$$p $(DESTDIR)$(sbindir)/$$f"; \
	    $(INSTALL_SCRIPT) $(srcdir)/$$p $(DESTDIR)$(sbindir)/$$f; \
	  else :; fi; \
	done

uninstall-sbinSCRIPTS:
	@$(NORMAL_UNINSTALL)
	@list='$(sbin_SCRIPTS)'; for p in $$list; do \
	  f="`echo $$p|sed '$(transform)'`"; \
	  echo " rm -f $(DESTDIR)$(sbindir)/$$f"; \
	  rm -f $(DESTDIR)$(sbindir)/$$f; \
	done

mostlyclean-compile:
	-rm -f *.$(OBJEXT) core *.core

distclean-compile:
	-rm -f *.tab.c

include $(DEPDIR)/chat.Po
include $(DEPDIR)/conn.Po
include $(DEPDIR)/copy.Po
include $(DEPDIR)/cu.Po
include $(DEPDIR)/log.Po
include $(DEPDIR)/prot.Po
include $(DEPDIR)/prote.Po
include $(DEPDIR)/protf.Po
include $(DEPDIR)/protg.Po
include $(DEPDIR)/proti.Po
include $(DEPDIR)/protj.Po
include $(DEPDIR)/prott.Po
include $(DEPDIR)/proty.Po
include $(DEPDIR)/protz.Po
include $(DEPDIR)/rec.Po
include $(DEPDIR)/send.Po
include $(DEPDIR)/time.Po
include $(DEPDIR)/trans.Po
include $(DEPDIR)/tstuu.Po
include $(DEPDIR)/util.Po
include $(DEPDIR)/uuchk.Po
include $(DEPDIR)/uucico.Po
include $(DEPDIR)/uuconv-uuconv.Po
include $(DEPDIR)/uucp.Po
include $(DEPDIR)/uudir.Po
include $(DEPDIR)/uulog.Po
include $(DEPDIR)/uuname.Po
include $(DEPDIR)/uupick.Po
include $(DEPDIR)/uustat.Po
include $(DEPDIR)/uux.Po
include $(DEPDIR)/uuxqt.Po
include $(DEPDIR)/xcmd.Po

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

uuconv-uuconv.o: uuconv.c
	source='uuconv.c' object='uuconv-uuconv.o' libtool=no \
	depfile='$(DEPDIR)/uuconv-uuconv.Po' tmpdepfile='$(DEPDIR)/uuconv-uuconv.TPo' \
	$(CCDEPMODE) $(depcomp) \
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(uuconv_CFLAGS) $(CFLAGS) -c -o uuconv-uuconv.o `test -f uuconv.c || echo '$(srcdir)/'`uuconv.c

uuconv-uuconv.obj: uuconv.c
	source='uuconv.c' object='uuconv-uuconv.obj' libtool=no \
	depfile='$(DEPDIR)/uuconv-uuconv.Po' tmpdepfile='$(DEPDIR)/uuconv-uuconv.TPo' \
	$(CCDEPMODE) $(depcomp) \
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(uuconv_CFLAGS) $(CFLAGS) -c -o uuconv-uuconv.obj `cygpath -w uuconv.c`
CCDEPMODE = depmode=gcc3

uucp.info: uucp.texi
uucp.dvi: uucp.texi

.texi.info:
	@cd $(srcdir) && rm -f $@ $@-[0-9] $@-[0-9][0-9]
	cd $(srcdir) \
	  && $(MAKEINFO) $(AM_MAKEINFOFLAGS) $(MAKEINFOFLAGS) \
	       `echo $< | sed 's,.*/,,'`

.texi.dvi:
	TEXINPUTS=$(srcdir):$$TEXINPUTS \
	MAKEINFO='$(MAKEINFO) $(AM_MAKEINFOFLAGS) $(MAKEINFOFLAGS) -I $(srcdir)' \
	$(TEXI2DVI) $<

.texi:
	@cd $(srcdir) && rm -f $@ $@-[0-9] $@-[0-9][0-9]
	cd $(srcdir) \
	  && $(MAKEINFO) $(AM_MAKEINFOFLAGS) $(MAKEINFOFLAGS) \
	       `echo $< | sed 's,.*/,,'`

MAKEINFO = ${SHELL} /home/leop/Code/armadeus-4.0/buildroot/output/build/uucp-1.07/missing --run makeinfo
TEXI2DVI = texi2dvi
DVIPS = dvips
.dvi.ps:
	$(DVIPS) $< -o $@

uninstall-info-am:
	$(PRE_UNINSTALL)
	@if (install-info --version && \
	     install-info --version | fgrep -i -v debian) >/dev/null 2>&1; then \
	  list='$(INFO_DEPS)'; \
	  for file in $$list; do \
	    echo " install-info --info-dir=$(DESTDIR)$(infodir) --remove $(DESTDIR)$(infodir)/$$file"; \
	    install-info --info-dir=$(DESTDIR)$(infodir) --remove $(DESTDIR)$(infodir)/$$file; \
	  done; \
	else :; fi
	@$(NORMAL_UNINSTALL)
	@list='$(INFO_DEPS)'; \
	for file in $$list; do \
	  (if cd $(DESTDIR)$(infodir); then \
	     echo " rm -f $$file $$file-[0-9] $$file-[0-9][0-9])"; \
	     rm -f $$file $$file-[0-9] $$file-[0-9][0-9]; \
	   else :; fi); \
	done

dist-info: $(INFO_DEPS)
	list='$(INFO_DEPS)'; \
	for base in $$list; do \
	  d=$(srcdir); \
	  for file in `CDPATH=: && cd $$d && eval echo $$base*`; do \
	    test -f $(distdir)/$$file \
	    || cp -p $$d/$$file $(distdir)/$$file; \
	  done; \
	done

mostlyclean-aminfo:
	-rm -f uucp.aux uucp.cp uucp.cps uucp.dvi uucp.fn uucp.fns uucp.ky uucp.log \
	  uucp.pg uucp.ps uucp.toc uucp.tp uucp.vr

maintainer-clean-aminfo:
	cd $(srcdir) && \
	for i in $(INFO_DEPS); do \
	  rm -f $$i; \
	  if test "`echo $$i-[0-9]*`" != "$$i-[0-9]*"; then \
	    rm -f $$i-[0-9]*; \
	  fi; \
	done

man1dir = $(mandir)/man1
install-man1: $(man1_MANS) $(man_MANS)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(man1dir)
	@list='$(man1_MANS) $(dist_man1_MANS) $(nodist_man1_MANS)'; \
	l2='$(man_MANS) $(dist_man_MANS) $(nodist_man_MANS)'; \
	for i in $$l2; do \
	  case "$$i" in \
	    *.1*) list="$$list $$i" ;; \
	  esac; \
	done; \
	for i in $$list; do \
	  if test -f $(srcdir)/$$i; then file=$(srcdir)/$$i; \
	  else file=$$i; fi; \
	  ext=`echo $$i | sed -e 's/^.*\\.//'`; \
	  inst=`echo $$i | sed -e 's/\\.[0-9a-z]*$$//'`; \
	  inst=`echo $$inst | sed -e 's/^.*\///'`; \
	  inst=`echo $$inst | sed '$(transform)'`.$$ext; \
	  echo " $(INSTALL_DATA) $$file $(DESTDIR)$(man1dir)/$$inst"; \
	  $(INSTALL_DATA) $$file $(DESTDIR)$(man1dir)/$$inst; \
	done
uninstall-man1:
	@$(NORMAL_UNINSTALL)
	@list='$(man1_MANS) $(dist_man1_MANS) $(nodist_man1_MANS)'; \
	l2='$(man_MANS) $(dist_man_MANS) $(nodist_man_MANS)'; \
	for i in $$l2; do \
	  case "$$i" in \
	    *.1*) list="$$list $$i" ;; \
	  esac; \
	done; \
	for i in $$list; do \
	  ext=`echo $$i | sed -e 's/^.*\\.//'`; \
	  inst=`echo $$i | sed -e 's/\\.[0-9a-z]*$$//'`; \
	  inst=`echo $$inst | sed -e 's/^.*\///'`; \
	  inst=`echo $$inst | sed '$(transform)'`.$$ext; \
	  echo " rm -f $(DESTDIR)$(man1dir)/$$inst"; \
	  rm -f $(DESTDIR)$(man1dir)/$$inst; \
	done

man8dir = $(mandir)/man8
install-man8: $(man8_MANS) $(man_MANS)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(man8dir)
	@list='$(man8_MANS) $(dist_man8_MANS) $(nodist_man8_MANS)'; \
	l2='$(man_MANS) $(dist_man_MANS) $(nodist_man_MANS)'; \
	for i in $$l2; do \
	  case "$$i" in \
	    *.8*) list="$$list $$i" ;; \
	  esac; \
	done; \
	for i in $$list; do \
	  if test -f $(srcdir)/$$i; then file=$(srcdir)/$$i; \
	  else file=$$i; fi; \
	  ext=`echo $$i | sed -e 's/^.*\\.//'`; \
	  inst=`echo $$i | sed -e 's/\\.[0-9a-z]*$$//'`; \
	  inst=`echo $$inst | sed -e 's/^.*\///'`; \
	  inst=`echo $$inst | sed '$(transform)'`.$$ext; \
	  echo " $(INSTALL_DATA) $$file $(DESTDIR)$(man8dir)/$$inst"; \
	  $(INSTALL_DATA) $$file $(DESTDIR)$(man8dir)/$$inst; \
	done
uninstall-man8:
	@$(NORMAL_UNINSTALL)
	@list='$(man8_MANS) $(dist_man8_MANS) $(nodist_man8_MANS)'; \
	l2='$(man_MANS) $(dist_man_MANS) $(nodist_man_MANS)'; \
	for i in $$l2; do \
	  case "$$i" in \
	    *.8*) list="$$list $$i" ;; \
	  esac; \
	done; \
	for i in $$list; do \
	  ext=`echo $$i | sed -e 's/^.*\\.//'`; \
	  inst=`echo $$i | sed -e 's/\\.[0-9a-z]*$$//'`; \
	  inst=`echo $$inst | sed -e 's/^.*\///'`; \
	  inst=`echo $$inst | sed '$(transform)'`.$$ext; \
	  echo " rm -f $(DESTDIR)$(man8dir)/$$inst"; \
	  rm -f $(DESTDIR)$(man8dir)/$$inst; \
	done

# This directory's subdirectories are mostly independent; you can cd
# into them and run `make' without going through this Makefile.
# To change the values of `make' variables: instead of editing Makefiles,
# (1) if the variable is set in `config.status', edit `config.status'
#     (which will cause the Makefiles to be regenerated when you run `make');
# (2) otherwise, pass the desired values on the `make' command line.
$(RECURSIVE_TARGETS):
	@set fnord $(MAKEFLAGS); amf=$$2; \
	dot_seen=no; \
	target=`echo $@ | sed s/-recursive//`; \
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  echo "Making $$target in $$subdir"; \
	  if test "$$subdir" = "."; then \
	    dot_seen=yes; \
	    local_target="$$target-am"; \
	  else \
	    local_target="$$target"; \
	  fi; \
	  (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) $$local_target) \
	   || case "$$amf" in *=*) exit 1;; *k*) fail=yes;; *) exit 1;; esac; \
	done; \
	if test "$$dot_seen" = "no"; then \
	  $(MAKE) $(AM_MAKEFLAGS) "$$target-am" || exit 1; \
	fi; test -z "$$fail"

mostlyclean-recursive clean-recursive distclean-recursive \
maintainer-clean-recursive:
	@set fnord $(MAKEFLAGS); amf=$$2; \
	dot_seen=no; \
	case "$@" in \
	  distclean-* | maintainer-clean-*) list='$(DIST_SUBDIRS)' ;; \
	  *) list='$(SUBDIRS)' ;; \
	esac; \
	rev=''; for subdir in $$list; do \
	  if test "$$subdir" = "."; then :; else \
	    rev="$$subdir $$rev"; \
	  fi; \
	done; \
	rev="$$rev ."; \
	target=`echo $@ | sed s/-recursive//`; \
	for subdir in $$rev; do \
	  echo "Making $$target in $$subdir"; \
	  if test "$$subdir" = "."; then \
	    local_target="$$target-am"; \
	  else \
	    local_target="$$target"; \
	  fi; \
	  (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) $$local_target) \
	   || case "$$amf" in *=*) exit 1;; *k*) fail=yes;; *) exit 1;; esac; \
	done && test -z "$$fail"
tags-recursive:
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  test "$$subdir" = . || (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) tags); \
	done

tags: TAGS

ID: $(HEADERS) $(SOURCES) $(LISP) $(TAGS_FILES)
	list='$(SOURCES) $(HEADERS) $(TAGS_FILES)'; \
	unique=`for i in $$list; do \
	    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
	  done | \
	  $(AWK) '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	mkid -fID $$unique $(LISP)

TAGS: tags-recursive $(HEADERS) $(SOURCES) config.h.in $(TAGS_DEPENDENCIES) \
		$(TAGS_FILES) $(LISP)
	tags=; \
	here=`pwd`; \
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  if test "$$subdir" = .; then :; else \
	    test -f $$subdir/TAGS && tags="$$tags -i $$here/$$subdir/TAGS"; \
	  fi; \
	done; \
	list='$(SOURCES) $(HEADERS) $(TAGS_FILES)'; \
	unique=`for i in $$list; do \
	    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
	  done | \
	  $(AWK) '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	test -z "$(ETAGS_ARGS)config.h.in$$unique$(LISP)$$tags" \
	  || etags $(ETAGS_ARGS) $$tags config.h.in $$unique $(LISP)

GTAGS:
	here=`CDPATH=: && cd $(top_builddir) && pwd` \
	  && cd $(top_srcdir) \
	  && gtags -i $(GTAGS_ARGS) $$here

distclean-tags:
	-rm -f TAGS ID GTAGS GRTAGS GSYMS GPATH

DISTFILES = $(DIST_COMMON) $(DIST_SOURCES) $(TEXINFOS) $(EXTRA_DIST)

top_distdir = .
# Avoid unsightly `./'.
distdir = $(PACKAGE)-$(VERSION)

GZIP_ENV = --best

distdir: $(DISTFILES)
	-chmod -R a+w $(distdir) >/dev/null 2>&1; rm -rf $(distdir)
	mkdir $(distdir)
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
	for subdir in $(SUBDIRS); do \
	  if test "$$subdir" = .; then :; else \
	    test -d $(distdir)/$$subdir \
	    || mkdir $(distdir)/$$subdir \
	    || exit 1; \
	    (cd $$subdir && \
	      $(MAKE) $(AM_MAKEFLAGS) \
	        top_distdir="$(top_distdir)" \
	        distdir=../$(distdir)/$$subdir \
	        distdir) \
	      || exit 1; \
	  fi; \
	done
	$(MAKE) $(AM_MAKEFLAGS) \
	  top_distdir="${top_distdir}" distdir="$(distdir)" \
	  dist-info dist-hook
	-find $(distdir) -type d ! -perm -777 -exec chmod a+rwx {} \; -o \
	  ! -type d ! -perm -444 -links 1 -exec chmod a+r {} \; -o \
	  ! -type d ! -perm -400 -exec chmod a+r {} \; -o \
	  ! -type d ! -perm -444 -exec $(SHELL) $(install_sh) -c -m a+r {} {} \; \
	|| chmod -R a+r $(distdir)
dist: distdir
	$(AMTAR) chof - $(distdir) | GZIP=$(GZIP_ENV) gzip -c >$(distdir).tar.gz
	-chmod -R a+w $(distdir) >/dev/null 2>&1; rm -rf $(distdir)

# This target untars the dist file and tries a VPATH configuration.  Then
# it guarantees that the distribution is self-contained by making another
# tarfile.
distcheck: dist
	-chmod -R a+w $(distdir) > /dev/null 2>&1; rm -rf $(distdir)
	GZIP=$(GZIP_ENV) gunzip -c $(distdir).tar.gz | $(AMTAR) xf -
	chmod -R a-w $(distdir); chmod a+w $(distdir)
	mkdir $(distdir)/=build
	mkdir $(distdir)/=inst
	chmod a-w $(distdir)
	dc_install_base=`CDPATH=: && cd $(distdir)/=inst && pwd` \
	  && cd $(distdir)/=build \
	  && ../configure --srcdir=.. --prefix=$$dc_install_base \
	  && $(MAKE) $(AM_MAKEFLAGS) \
	  && $(MAKE) $(AM_MAKEFLAGS) dvi \
	  && $(MAKE) $(AM_MAKEFLAGS) check \
	  && $(MAKE) $(AM_MAKEFLAGS) install \
	  && $(MAKE) $(AM_MAKEFLAGS) installcheck \
	  && $(MAKE) $(AM_MAKEFLAGS) uninstall \
	  && (test `find $$dc_install_base -type f -print | wc -l` -le 1 \
	     || (echo "Error: files left after uninstall" 1>&2; \
	         exit 1) ) \
	  && $(MAKE) $(AM_MAKEFLAGS) dist \
	  && $(MAKE) $(AM_MAKEFLAGS) distclean \
	  && rm -f $(distdir).tar.gz \
	  && (test `find . -type f -print | wc -l` -eq 0 \
	     || (echo "Error: files left after distclean" 1>&2; \
	         exit 1) )
	-chmod -R a+w $(distdir) > /dev/null 2>&1; rm -rf $(distdir)
	@echo "$(distdir).tar.gz is ready for distribution" | \
	  sed 'h;s/./=/g;p;x;p;x'
check-am: all-am
check: check-recursive
all-am: Makefile $(INFO_DEPS) $(PROGRAMS) $(SCRIPTS) $(MANS) config.h
installdirs: installdirs-recursive
installdirs-am:
	$(mkinstalldirs) $(DESTDIR)$(bindir) $(DESTDIR)$(sbindir) $(DESTDIR)$(uudirdir) $(DESTDIR)$(bindir) $(DESTDIR)$(sbindir) $(DESTDIR)$(infodir) $(DESTDIR)$(man1dir) $(DESTDIR)$(man8dir)

install: install-recursive
install-exec: install-exec-recursive
install-data: install-data-recursive
uninstall: uninstall-recursive

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am

installcheck: installcheck-recursive
install-strip:
	$(MAKE) $(AM_MAKEFLAGS) INSTALL_PROGRAM="$(INSTALL_STRIP_PROGRAM)" \
	  `test -z '$(STRIP)' || \
	    echo "INSTALL_PROGRAM_ENV=STRIPPROG='$(STRIP)'"` install
mostlyclean-generic:

clean-generic:
	-test -z "$(CLEANFILES)" || rm -f $(CLEANFILES)

distclean-generic:
	-rm -f Makefile $(CONFIG_CLEAN_FILES) stamp-h stamp-h[0-9]*

maintainer-clean-generic:
	@echo "This command is intended for maintainers to use"
	@echo "it deletes files that may require special tools to rebuild."
clean: clean-recursive

clean-am: clean-binPROGRAMS clean-generic clean-noinstPROGRAMS \
	clean-sbinPROGRAMS clean-uudirPROGRAMS mostlyclean-am

dist-all: distdir
	$(AMTAR) chof - $(distdir) | GZIP=$(GZIP_ENV) gzip -c >$(distdir).tar.gz
	-chmod -R a+w $(distdir) >/dev/null 2>&1; rm -rf $(distdir)
distclean: distclean-recursive
	-rm -f config.status config.cache config.log
distclean-am: clean-am distclean-compile distclean-depend \
	distclean-generic distclean-hdr distclean-tags

dvi: dvi-recursive

dvi-am: $(DVIS)

info: info-recursive

info-am: $(INFO_DEPS)

install-data-am: install-info-am install-man install-uudirPROGRAMS

install-exec-am: install-binPROGRAMS install-binSCRIPTS \
	install-sbinPROGRAMS install-sbinSCRIPTS
	@$(NORMAL_INSTALL)
	$(MAKE) $(AM_MAKEFLAGS) install-exec-hook

install-info: install-info-recursive

install-info-am: $(INFO_DEPS)
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(infodir)
	@list='$(INFO_DEPS)'; \
	for file in $$list; do \
	  d=$(srcdir); \
	  for ifile in `CDPATH=: && cd $$d && echo $$file $$file-[0-9] $$file-[0-9][0-9]`; do \
	    if test -f $$d/$$ifile; then \
	      echo " $(INSTALL_DATA) $$d/$$ifile $(DESTDIR)$(infodir)/$$ifile"; \
	      $(INSTALL_DATA) $$d/$$ifile $(DESTDIR)$(infodir)/$$ifile; \
	    else : ; fi; \
	  done; \
	done
	@$(POST_INSTALL)
	@if (install-info --version && \
	     install-info --version | fgrep -i -v debian) >/dev/null 2>&1; then \
	  list='$(INFO_DEPS)'; \
	  for file in $$list; do \
	    echo " install-info --info-dir=$(DESTDIR)$(infodir) $(DESTDIR)$(infodir)/$$file";\
	    install-info --info-dir=$(DESTDIR)$(infodir) $(DESTDIR)$(infodir)/$$file || :;\
	  done; \
	else : ; fi
install-man: install-man1 install-man8

installcheck-am:

maintainer-clean: maintainer-clean-recursive

maintainer-clean-am: distclean-am maintainer-clean-aminfo \
	maintainer-clean-generic

mostlyclean: mostlyclean-recursive

mostlyclean-am: mostlyclean-aminfo mostlyclean-compile \
	mostlyclean-generic

uninstall-am: uninstall-binPROGRAMS uninstall-binSCRIPTS \
	uninstall-info-am uninstall-man uninstall-sbinPROGRAMS \
	uninstall-sbinSCRIPTS uninstall-uudirPROGRAMS

uninstall-info: uninstall-info-recursive

uninstall-man: uninstall-man1 uninstall-man8

.PHONY: $(RECURSIVE_TARGETS) GTAGS all all-am check check-am clean \
	clean-binPROGRAMS clean-generic clean-noinstPROGRAMS \
	clean-recursive clean-sbinPROGRAMS clean-uudirPROGRAMS dist \
	dist-all dist-info distcheck distclean distclean-compile \
	distclean-depend distclean-generic distclean-hdr \
	distclean-recursive distclean-tags distdir dvi dvi-am \
	dvi-recursive info info-am info-recursive install install-am \
	install-binPROGRAMS install-binSCRIPTS install-data \
	install-data-am install-data-recursive install-exec \
	install-exec-am install-exec-recursive install-info \
	install-info-am install-info-recursive install-man install-man1 \
	install-man8 install-recursive install-sbinPROGRAMS \
	install-sbinSCRIPTS install-strip install-uudirPROGRAMS \
	installcheck installcheck-am installdirs installdirs-am \
	installdirs-recursive maintainer-clean maintainer-clean-aminfo \
	maintainer-clean-generic maintainer-clean-recursive mostlyclean \
	mostlyclean-aminfo mostlyclean-compile mostlyclean-generic \
	mostlyclean-recursive tags tags-recursive uninstall \
	uninstall-am uninstall-binPROGRAMS uninstall-binSCRIPTS \
	uninstall-info-am uninstall-info-recursive uninstall-man \
	uninstall-man1 uninstall-man8 uninstall-recursive \
	uninstall-sbinPROGRAMS uninstall-sbinSCRIPTS \
	uninstall-uudirPROGRAMS


uusched: uusched.in Makefile
	-rm -f $@ uusched.t
	if [ "x$(POUNDBANG)" = "xno" ]; then \
	  sed -e 's,#!/bin/sh,:,' -e 's,@SBINDIR@,$(sbindir),' < $(srcdir)/uusched.in > uusched.t; \
	else \
	  sed -e 's,@SBINDIR@,$(sbindir),' < $(srcdir)/uusched.in > uusched.t; \
	fi
	chmod 0555 uusched.t
	mv -f uusched.t $@

uuto: uuto.in Makefile
	-rm -f $@ uuto.t
	if [ "x$(POUNDBANG)" = "xno" ]; then \
	  sed -e 's,#!/bin/sh,:,' -e 's,@BINDIR@,$(bindir),' -e 's,@VERS@,$(VERSION),' < $(srcdir)/uuto.in > uuto.t; \
	else \
	  sed -e 's,@BINDIR@,$(bindir),' -e 's,@VERS@,$(VERSION),' < $(srcdir)/uuto.in > uuto.t; \
	fi
	chmod 0555 uuto.t
	mv -f uuto.t $@

dist-hook:
	-rm -rf $(distdir)/contrib/CVS $(distdir)/sample/CVS

install-exec-hook:
	for f in uucico uuxqt; do \
	  chown $(OWNER) $(DESTDIR)$(sbindir)/$${f}; \
	  chmod 4555 $(DESTDIR)$(sbindir)/$${f}; \
	done
	for f in uux uucp uustat uuname cu; do \
	  chown $(OWNER) $(DESTDIR)$(bindir)/$${f}; \
	  chmod 4555 $(DESTDIR)$(bindir)/$${f}; \
	done
#	chown $(OWNER) $(DESTDIR)$(uudirdir)
#	chmod 100 $(DESTDIR)$(uudirdir)
#	chown root $(DESTDIR)$(uudirdir)/uudir
#	chmod 4555 $(DESTDIR)$(uudirdir)/uudir
# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
