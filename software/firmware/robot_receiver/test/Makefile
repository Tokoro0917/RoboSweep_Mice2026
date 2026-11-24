# PC 上で ball_detect.h をテストする
CFLAGS = -std=gnu11 -Wall -Wextra -O0 -g

test_ball_detect: test_ball_detect.c ../ball_detect.h
	$(CC) $(CFLAGS) test_ball_detect.c -lm -o $@
	./$@

clean:
	rm -f test_ball_detect
.PHONY: clean
