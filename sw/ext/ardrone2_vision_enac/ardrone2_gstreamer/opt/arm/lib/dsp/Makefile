version := $(shell ./get-version)
dspdir := /lib/dsp

D = $(DESTDIR)

install: $(targets)
	mkdir -p $(D)$(dspdir)
	install -m 644 baseimage.dof *.dll64P LICENSE $(D)$(dspdir)

dist: base := tidsp-binaries-$(version)
dist:
	@tar -czf /tmp/$(base).tar.gz --transform='s,^,$(base)/,' * .version
