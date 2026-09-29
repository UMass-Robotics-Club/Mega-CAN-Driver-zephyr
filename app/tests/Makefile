# PC unit tests for pure-C firmware modules (no Zephyr, no board).
#   make -C app/tests
CC ?= cc
CFLAGS += -std=c11 -Wall -Wextra -Werror -O1 -I../src
LDLIBS += -lm

test: test_robstride test_proto
	./test_robstride
	python3 crosscheck_proto.py

test_robstride: test_robstride.c ../src/robstride.c ../src/robstride.h
	$(CC) $(CFLAGS) -o $@ test_robstride.c ../src/robstride.c $(LDLIBS)

test_proto: test_proto.c ../src/proto.h
	$(CC) $(CFLAGS) -o $@ test_proto.c

clean:
	rm -f test_robstride test_proto

.PHONY: test clean
