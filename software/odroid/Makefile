SUBDIRS := $(wildcard */.)

all: $(SUBDIRS)
$(SUBDIRS):
	cd $@; make;

.PHONY: all $(SUBDIRS)
