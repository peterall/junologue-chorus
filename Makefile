TOPTARGETS := all

PLATFORMS := prologue minilogue-xd nutekt-digital

VERSION=1.2-0
PACKAGE = junologue-chorus-$(VERSION).zip

$(TOPTARGETS): $(PLATFORMS) package

clean:
	@echo Cleaning ./build and packages
	@rm -fR .dep ./build
	@rm -f junologue-chorus.*unit
	@rm -f junologue-chorus-*.zip

$(PLATFORMS):
	@PLATFORM=$@ VERSION=$(VERSION) $(MAKE) -f junologue-chorus.mk $(MAKECMDGOALS)

package:
	@echo Packaging to ./${PACKAGE}
	@zip -q9 - *.*unit > ${PACKAGE}
	@echo All done