SUBDIRS := $(wildcard */.)

all: $(SUBDIRS)
.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

.PHONY: all $(SUBDIRS)
