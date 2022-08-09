.PHONY: all
all: format test build

.PHONY: format
format:
	clang-format src/*/*.hpp src/*/*.cpp src/*.hpp src/*.cpp test/**.cpp -i

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
	test/EXCERCISE_TEST

.PHONY: debug
debug:
	mkdir -p build
	cd build && \
	cmake -DCMAKE_BUILD_TYPE=debug .. && \
	make

.PHONY: clean
clean:
	rm -rf build
