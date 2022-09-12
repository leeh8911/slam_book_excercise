.PHONY: all
all: format test build debug static-test clean

.PHONY: format
format:
	find . -regex '.*\.\(cpp\|h\)' -exec clang-format -style=file -i {} \;

.PHONY: build
build:
	mkdir -p build
	cd build && \
	cmake .. && \
	make

.PHONY: test
test:
	mkdir -p build
	cd build && \
	cmake .. && \
	make &&\
	test/SLAMBOOK_TEST

.PHONY: debug
debug:
	mkdir -p build
	cd build && \
	cmake -DCMAKE_BUILD_TYPE=debug -DCMAKE_CXX_CLANG_TIDY="clang-tidy;-checks=-*,google-readability-casting;-fix;-fix-errors;" .. &&\
	make

.PHONY: static-test
static-test:
	mkdir -p build/static-test
	cppcheck --cppcheck-build-dir=build/static-tesst src

.PHONY: clean
clean:
	rm -rf build