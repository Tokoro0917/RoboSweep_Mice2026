# PC 上で ctl_motion.c / ctl_wall.c をテストする
CFLAGS = -std=gnu11 -Wall -Wextra -O0 -g -Istub -I. -I../Core/Inc
SRCS = test_motion.c fake_hw.c ../Core/Src/ctl_motion.c ../Core/Src/ctl_wall.c
test_motion: $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -lm -o $@
	./$@
clean:
	rm -f test_motion
.PHONY: clean
