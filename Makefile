.DEFAULT_GOAL := libs

# Build hardware library submodules
libs:
	$(MAKE) -C lib/libDaisy -j4
	$(MAKE) -C lib/DaisySP -j4

clean-libs:
	$(MAKE) -C lib/libDaisy clean
	$(MAKE) -C lib/DaisySP clean

clean:
	rm -rf build/

.PHONY: libs clean-libs clean
