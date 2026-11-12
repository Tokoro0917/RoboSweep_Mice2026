# PC 上で motion.c をテストする
CFLAGS = -std=gnu11 -Wall -Wextra -O0 -g -Istub -I../Core/Inc
test_motion: test_motion.c ../Core/Src/motion.c
	$(CC) $(CFLAGS) $^ -lm -o $@
	./$@
clean:
	rm -f test_motion
.PHONY: clean
