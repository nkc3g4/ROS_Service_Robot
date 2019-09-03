#
# Common Makefile code for building D examples.
#
# We actually need to configure this to gain access to the default D version to
# use when D_VERSION is not set. Using Examples/Makefile.in is not enough, as
# the location of the source files (d1/ or d2/) depends on it. The alternative
# would be to add the functionality specific to Examples/d (as opposed to the
# test suite) directly to Examples/Makefile.in.
#
# This file is supposed to be included from a Makefile in the subdirectory
# corresponding to a specific example.
#

ifeq (,$(D_VERSION))
	D_VERSION = 
endif

ifeq (1,$(D_VERSION))
	VERSION_DIR = d1/
else
	VERSION_DIR = d2/
endif

EXAMPLES_TOP   = ../../..
SWIG_TOP       = ../../../..
SWIGEXE        = $(SWIG_TOP)/swig
SWIG_LIB_DIR   = $(SWIG_TOP)/$(TOP_BUILDDIR_TO_TOP_SRCDIR)Lib
EXTRA_CFLAGS   =
EXTRA_CXXFLAGS =
EXTRA_LDFLAGS  =
TARGET         = example_wrap
SWIGOPT        = -outcurrentdir
DFLAGS         = -ofrunme

ifeq (,$(SRCDIR))
DSRCS          = *.d
else
DSRCS          = *.d $(addprefix ../$(SRCDIR)$(VERSION_DIR),runme.d)
endif


check: build
	$(MAKE) -C $(VERSION_DIR) -f $(EXAMPLES_TOP)/Makefile SRCDIR='../$(SRCDIR)' d_run

build:
	mkdir -p $(VERSION_DIR)
	if [ -f $(SRCDIR)example.cxx ]; then \
		$(MAKE) -C $(VERSION_DIR) -f $(EXAMPLES_TOP)/Makefile SRCDIR='../$(SRCDIR)' EXTRA_CXXFLAGS='$(EXTRA_CXXFLAGS)' EXTRA_LDFLAGS='$(EXTRA_LDFLAGS)' \
		SWIG_LIB_DIR='$(SWIG_LIB_DIR)' SWIGEXE='$(SWIGEXE)' \
		SWIGOPT='$(SWIGOPT)' TARGET='$(TARGET)' INTERFACE='example.i' CXXSRCS='example.cxx' d_cpp; \
	elif [ -f $(SRCDIR)example.c ]; then \
		$(MAKE) -C $(VERSION_DIR) -f $(EXAMPLES_TOP)/Makefile SRCDIR='../$(SRCDIR)' EXTRA_CFLAGS='$(EXTRA_CFLAGS)' EXTRA_LDFLAGS='$(EXTRA_LDFLAGS)' \
		SWIG_LIB_DIR='$(SWIG_LIB_DIR)' SWIGEXE='$(SWIGEXE)' \
		SWIGOPT='$(SWIGOPT)' TARGET='$(TARGET)' INTERFACE='example.i' SRCS='example.c' d; \
	else \
		$(MAKE) -C $(VERSION_DIR) -f $(EXAMPLES_TOP)/Makefile SRCDIR='../$(SRCDIR)' EXTRA_CFLAGS='$(EXTRA_CFLAGS)' EXTRA_LDFLAGS='$(EXTRA_LDFLAGS)' \
		SWIG_LIB_DIR='$(SWIG_LIB_DIR)' SWIGEXE='$(SWIGEXE)' \
		SWIGOPT='$(SWIGOPT)' TARGET='$(TARGET)' INTERFACE='example.i' SRCS='' d; \
	fi
	$(MAKE) -C $(VERSION_DIR) -f $(EXAMPLES_TOP)/Makefile SRCDIR='../$(SRCDIR)' DSRCS='$(DSRCS)' DFLAGS='$(DFLAGS)' d_compile

clean:
	if [ -d $(VERSION_DIR) ]; then \
		$(MAKE) -C $(VERSION_DIR) -f $(EXAMPLES_TOP)/Makefile SRCDIR='../$(SRCDIR)' d_clean; \
	fi
	test -f $(VERSION_DIR)runme.d || rm -rf $(VERSION_DIR) # Only delete dir if out of source
