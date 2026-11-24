/*
 * ball_detect.h を PC 上で動かすテスト。
 *   make -C software/firmware/robot_receiver/test
 *
 * フィールドの色（壁の側面は白、壁の上面は赤、床は黒）で、低い所から見た画像をにせで作り、
 * 黄色いボールを正しく見つけるか（ほかの色を間違えないか）を確かめる。
 * 本物のカメラの画像で、しきい値は必ず合わせ直すこと。
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../ball_detect.h"

#define W 160
#define H 120

static uint16_t img[W * H];
static uint8_t mask[W * H];
static int32_t stack[W * H];
static int failures = 0;

static uint16_t rgb(int r, int g, int b) {
	return (uint16_t)(((r * 31 / 255) << 11) | ((g * 63 / 255) << 5) | (b * 31 / 255));
}

/* 低い所から通路を見た景色: 上は奥の壁（白、上面は赤い線）、下は黒い床 */
static void scene(void) {
	for (int y = 0; y < H; y++)
		for (int x = 0; x < W; x++) {
			uint16_t c;
			if (y < 20) c = rgb(40, 40, 50);              /* 壁の上の暗いところ */
			else if (y < 24) c = rgb(210, 30, 30);         /* 壁の上面（赤） */
			else if (y < 60) c = rgb(235, 235, 225);       /* 壁の側面（白） */
			else c = rgb(20, 20, 22);                      /* 床（黒） */
			/* 少しノイズ */
			int n = (x * 7 + y * 13) % 5;
			c = (uint16_t)(c + (n == 0 ? 1 : 0));
			img[y * W + x] = c;
		}
	/* 横の壁（白）を左右に */
	for (int y = 24; y < H; y++)
		for (int x = 0; x < W; x++)
			if (x < 8 + (H - y) / 6 || x > W - 9 - (H - y) / 6) img[y * W + x] = rgb(225, 225, 215);
}

/* 黄色いボール（陰を少しつける） */
static void ball(int cx, int cy, int r) {
	for (int y = cy - r; y <= cy + r; y++)
		for (int x = cx - r; x <= cx + r; x++) {
			if (x < 0 || x >= W || y < 0 || y >= H) continue;
			int dx = x - cx, dy = y - cy;
			if (dx * dx + dy * dy > r * r) continue;
			int shade = 255 - 60 * (dx + dy + 2 * r) / (4 * r);   /* 右下が暗い */
			img[y * W + x] = rgb(shade, shade * 210 / 255, 20);
		}
}

/* 四角で隠す（手前の壁など） */
static void cover(int x0, int y0, int x1, int y1, uint16_t c) {
	for (int y = y0; y < y1; y++)
		for (int x = x0; x < x1; x++)
			if (x >= 0 && x < W && y >= 0 && y < H) img[y * W + x] = c;
}

static void expect(const char *name, int cond) {
	printf("%-56s %s\n", name, cond ? "OK" : "NG");
	if (!cond) failures++;
}

static ball_blob_t run(int swap) {
	ball_blob_t b;
	if (swap) {
		for (int i = 0; i < W * H; i++) img[i] = (uint16_t)((img[i] >> 8) | (img[i] << 8));
	}
	ball_detect(img, W, H, swap, mask, stack, &b);
	return b;
}

int main(void) {
	ball_blob_t b;

	/* 色の判定 */
	expect("黄色は黄色", ball_is_yellow(rgb(240, 200, 20), 0));
	expect("暗めの黄色（陰）も黄色", ball_is_yellow(rgb(180, 150, 20), 0));
	expect("赤（壁の上面）は黄色でない", !ball_is_yellow(rgb(210, 30, 30), 0));
	expect("白（壁の側面）は黄色でない", !ball_is_yellow(rgb(235, 235, 225), 0));
	expect("黒（床）は黄色でない", !ball_is_yellow(rgb(20, 20, 22), 0));
	expect("オレンジ寄りの赤は黄色でない", !ball_is_yellow(rgb(230, 80, 20), 0));
	expect("緑は黄色でない", !ball_is_yellow(rgb(60, 200, 40), 0));

	/* ボールなし */
	scene();
	b = run(0);
	expect("ボールなし: 見つけない", !b.found);

	/* 1 区画先の正面（大きく写る） */
	scene();
	ball(80, 75, 22);
	b = run(0);
	expect("正面のボール: 見つける", b.found);
	expect("正面のボール: 中心の横位置", abs(b.cx - 80) <= 2);
	expect("正面のボール: 幅 = 直径", abs(b.w - 45) <= 2);
	expect("正面のボール: 向きはほぼ 0°", fabsf(ball_bearing_deg(&b, W)) < 2.0f);

	/* バイトの順番が逆のカメラ */
	scene();
	ball(80, 75, 22);
	b = run(1);
	expect("バイトの順番が逆でも、swap = 1 で見つける", b.found && abs(b.cx - 80) <= 2);

	/* 左にある（向きは +） */
	scene();
	ball(40, 70, 12);
	b = run(0);
	expect("左のボール: 向きが +（左）", b.found && ball_bearing_deg(&b, W) > 5.0f);

	/* 遠いほど小さい → 距離が大きい */
	scene();
	ball(80, 65, 22);
	b = run(0);
	float near_mm = ball_distance_mm(&b, W);
	scene();
	ball(80, 62, 11);
	b = run(0);
	float far_mm = ball_distance_mm(&b, W);
	printf("    距離の目安: 大きく写ったボール %.0fmm、半分の大きさ %.0fmm\n", near_mm, far_mm);
	expect("半分の大きさなら、距離はほぼ 2 倍", fabsf(far_mm / near_mm - 2.0f) < 0.2f);

	/* 2 個見えたら、大きい（近い）ほう */
	scene();
	ball(40, 72, 9);
	ball(110, 78, 20);
	b = run(0);
	expect("2 個: 大きいほうを選ぶ", b.found && abs(b.cx - 110) <= 2);
	expect("2 個: 塊の数は 2", b.blobs == 2);

	/* 画像の左の端で切れている */
	scene();
	ball(5, 75, 20);
	b = run(0);
	expect("端で切れたボール: 見つけて、切れていると分かる", b.found && b.clipped);

	/* 下半分が手前の壁で隠れている → 幅が残っていれば距離は同じくらい */
	scene();
	ball(80, 70, 18);
	b = run(0);
	float full_mm = ball_distance_mm(&b, W);
	scene();
	ball(80, 70, 18);
	cover(0, 72, W, H, rgb(225, 225, 215));
	b = run(0);
	expect("下半分が隠れたボール: 見つける", b.found);
	expect("下半分が隠れたボール: 距離はほぼ同じ", fabsf(ball_distance_mm(&b, W) - full_mm) < full_mm * 0.1f);

	/* 小さすぎる黄色はゴミとして無視 */
	scene();
	ball(80, 75, 1);
	b = run(0);
	expect("数画素の黄色は無視する", !b.found);

	printf(failures ? "\n%d 件失敗\n" : "\nすべて OK\n", failures);
	return failures ? 1 : 0;
}
