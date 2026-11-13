# PC 上でテストする
#   test_motion: ctl_motion.c / ctl_wall.c
#   test_maze:   maze.c
#   test_auto:   app_auto.c（maze.c と ctl_motion.c も本物を使う）
CFLAGS = -std=gnu11 -Wall -Wextra -O0 -g -Istub -I. -I../Core/Inc
MOTION_SRCS = test_motion.c fake_hw.c ../Core/Src/ctl_motion.c ../Core/Src/ctl_wall.c
MAZE_SRCS = test_maze.c sim_maze.c ../Core/Src/maze.c
AUTO_SRCS = test_auto.c sim_maze.c fake_hw.c ../Core/Src/app_auto.c ../Core/Src/maze.c ../Core/Src/ctl_motion.c ../Core/Src/ctl_wall.c

all: test_motion test_maze test_auto test_auto_mid test_auto_2nd
	./test_motion
	./test_maze
	./test_auto
	./test_auto_mid
	./test_auto_2nd

test_motion: $(MOTION_SRCS)
	$(CC) $(CFLAGS) $(MOTION_SRCS) -lm -o $@

test_maze: $(MAZE_SRCS) sim_maze.h ../Core/Inc/maze.h
	$(CC) $(CFLAGS) $(MAZE_SRCS) -o $@

test_auto: $(AUTO_SRCS) ../Core/Inc/maze.h ../Core/Inc/app_auto.h ../Core/Inc/config.h
	$(CC) $(CFLAGS) $(AUTO_SRCS) -lm -o $@

# スタート区画の後ろが内側（壁がない）の場合
test_auto_mid: $(AUTO_SRCS) ../Core/Inc/maze.h ../Core/Inc/app_auto.h ../Core/Inc/config.h
	$(CC) $(CFLAGS) -DAUTO_START_X=2 -DAUTO_START_Y=2 -DAUTO_START_DIR=1 $(AUTO_SRCS) -lm -o $@

# 自律機 2 台のうちの 2 台目（左上のスタート、ボール 1 個）
test_auto_2nd: $(AUTO_SRCS) ../Core/Inc/maze.h ../Core/Inc/app_auto.h ../Core/Inc/config.h
	$(CC) $(CFLAGS) -DAUTO_ROBOT_COUNT=2 -DAUTO_ROBOT_ID=1 $(AUTO_SRCS) -lm -o $@

clean:
	rm -f test_motion test_maze test_auto test_auto_mid test_auto_2nd
.PHONY: all clean
