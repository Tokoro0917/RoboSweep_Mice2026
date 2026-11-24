/*
 * ball_detect.h
 *
 * カメラの画像（RGB565）から黄色いボールを見つける（Issue #8）。
 * robot_receiver.ino（XIAO ESP32S3 Sense）から使う。ハードに触らないので PC でもテストできる
 * （software/firmware/robot_receiver/test）。
 *
 * やり方
 *   1. 画素ごとに「黄色か」を決める（色相・彩度・明るさのしきい値。下の BALL_* で調整）
 *   2. 黄色い画素のつながった塊のうち、いちばん大きいものを選ぶ
 *      （ボールは最大 4 個あり、2 個以上見えることがある。いちばん大きい = いちばん近い）
 *   3. 塊の中心・幅・高さ・画素数と、画像の端で切れているかを返す
 *
 * フィールドの色（規程 2-1）: 壁の側面は白、壁の上面は赤、床は黒。ボールは黄色（規程 3-4）。
 * 白（彩度が低い）・赤（色相が 0 付近）・黒（暗い）は、どれも黄色と区別できる。
 */
#ifndef BALL_DETECT_H_
#define BALL_DETECT_H_

#include <stdint.h>
#include <string.h>
#include <math.h>

/* ---- 黄色のしきい値（実機の画像を見て調整する） ---- */
#define BALL_HUE_MIN        35    /* 色相 [deg]。赤 0、黄 60、緑 120 */
#define BALL_HUE_MAX        75
#define BALL_SAT_MIN        110   /* 彩度 0〜255。白い壁はこれより小さい */
#define BALL_VAL_MIN        70    /* 明るさ 0〜255。黒い床はこれより小さい */
#define BALL_MIN_PIXELS     12    /* これより小さい塊はゴミとして無視する */

/* ---- カメラ（要確認：実機で 2 か所の距離にボールを置いて合わせる） ---- */
#define BALL_CAM_HFOV_DEG   66.0f   /* 横の画角 [deg]。レンズで違う。要確認 */
#define BALL_DIAMETER_MM    70.0f

typedef struct {
	int found;      /* 見つけたら 1 */
	int cx, cy;     /* 塊の中心 [画素]（左上が 0, 0） */
	int x0, y0;     /* 塊を囲む四角の左上 */
	int w, h;       /* 塊を囲む四角の幅・高さ [画素] */
	int pixels;     /* 塊の画素数 */
	int clipped;    /* 塊が画像の左右の端に触れていれば 1（幅が本当より小さい） */
	int blobs;      /* 見つかった塊の数（BALL_MIN_PIXELS 以上） */
} ball_blob_t;

/* RGB565 の 1 画素が黄色か。swap = 1 なら上位と下位のバイトを入れ替えてから読む */
static inline int ball_is_yellow(uint16_t px, int swap) {
	if (swap) {
		px = (uint16_t)((px >> 8) | (px << 8));
	}
	int r = ((px >> 11) & 0x1F) * 255 / 31;
	int g = ((px >> 5) & 0x3F) * 255 / 63;
	int b = (px & 0x1F) * 255 / 31;
	int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
	int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
	if (mx < BALL_VAL_MIN || mx == 0) {
		return 0;
	}
	int d = mx - mn;
	if (d * 255 < BALL_SAT_MIN * mx) {
		return 0;
	}
	int hue;
	if (mx == r) {
		hue = 60 * (g - b) / d;          /* -60〜60 */
		if (hue < 0) {
			hue += 360;
		}
	} else if (mx == g) {
		hue = 120 + 60 * (b - r) / d;
	} else {
		hue = 240 + 60 * (r - g) / d;
	}
	return hue >= BALL_HUE_MIN && hue <= BALL_HUE_MAX;
}

/*
 * 画像からボールを探す。
 *   img:   RGB565 の画素（width * height 個）
 *   mask:  作業用（width * height バイト）
 *   stack: 作業用（width * height 個の int32_t）
 */
static inline void ball_detect(const uint16_t *img, int width, int height, int swap,
		uint8_t *mask, int32_t *stack, ball_blob_t *out) {
	const int n = width * height;
	memset(out, 0, sizeof(*out));
	for (int i = 0; i < n; i++) {
		mask[i] = (uint8_t)ball_is_yellow(img[i], swap);
	}
	int best = 0;
	for (int start = 0; start < n; start++) {
		if (mask[start] != 1) {
			continue;
		}
		/* つながった黄色い画素を数える（4 近傍）。数えた画素は 2 にする */
		int sp = 0, count = 0;
		long sx = 0, sy = 0;
		int minx = width, maxx = -1, miny = height, maxy = -1;
		stack[sp++] = start;
		mask[start] = 2;
		while (sp > 0) {
			int p = stack[--sp];
			int x = p % width, y = p / width;
			count++;
			sx += x;
			sy += y;
			if (x < minx) minx = x;
			if (x > maxx) maxx = x;
			if (y < miny) miny = y;
			if (y > maxy) maxy = y;
			if (x > 0 && mask[p - 1] == 1) { mask[p - 1] = 2; stack[sp++] = p - 1; }
			if (x < width - 1 && mask[p + 1] == 1) { mask[p + 1] = 2; stack[sp++] = p + 1; }
			if (y > 0 && mask[p - width] == 1) { mask[p - width] = 2; stack[sp++] = p - width; }
			if (y < height - 1 && mask[p + width] == 1) { mask[p + width] = 2; stack[sp++] = p + width; }
		}
		if (count < BALL_MIN_PIXELS) {
			continue;
		}
		out->blobs++;
		if (count > best) {
			best = count;
			out->found = 1;
			out->pixels = count;
			out->cx = (int)(sx / count);
			out->cy = (int)(sy / count);
			out->x0 = minx;
			out->y0 = miny;
			out->w = maxx - minx + 1;
			out->h = maxy - miny + 1;
			out->clipped = (minx == 0 || maxx == width - 1);
		}
	}
}

/* 焦点距離 [画素]（横の画角から） */
static inline float ball_focal_px(int width) {
	return (width / 2.0f) / tanf(BALL_CAM_HFOV_DEG * 0.5f * 3.14159265f / 180.0f);
}

/* ボールの向き [deg]。機体の正面が 0、左が +（ctl_motion.h と同じ向き） */
static inline float ball_bearing_deg(const ball_blob_t *b, int width) {
	float dx = (float)b->cx - (width - 1) / 2.0f;
	return -atanf(dx / ball_focal_px(width)) * 180.0f / 3.14159265f;
}

/*
 * ボールまでのだいたいの距離 [mm]（カメラから）。
 * ボールは丸いので、幅と高さの大きいほうを直径とみなす
 * （壁で下が隠れたり、画像の上下で切れたりしても、幅が残っていれば使える）。
 * 左右の端で切れているときは本当より遠く出る。
 */
static inline float ball_distance_mm(const ball_blob_t *b, int width) {
	int size = b->w > b->h ? b->w : b->h;
	if (size <= 0) {
		return 0.0f;
	}
	return ball_focal_px(width) * BALL_DIAMETER_MM / (float)size;
}

#endif /* BALL_DETECT_H_ */
