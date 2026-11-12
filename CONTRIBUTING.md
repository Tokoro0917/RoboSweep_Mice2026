# 作業の進め方

コラボレーターが多いので、全員が同じルールで作業できるようにまとめています。

## 基本の流れ

1. `main` を最新にする
   ```bash
   git switch main
   git pull origin main
   ```
2. 作業用ブランチを作る（名前は下のルールに従う）
   ```bash
   git switch -c feature/sweep-arm-design
   ```
3. 作業してコミット
4. push して Pull Request (PR) を作る
   ```bash
   git push -u origin feature/sweep-arm-design
   ```
5. 他のメンバーに確認してもらってから `main` にマージ

> `main` へ直接 push しないでください。必ず PR を経由します。

## ブランチ名のルール

**`種類/内容`** の形にします。内容は英小文字とハイフンで、見ただけで何の作業か分かる名前にしてください。

| 種類 | 使う場面 | 例 |
| --- | --- | --- |
| `feature/` | 新しい機能・機構の追加 | `feature/line-tracing`, `feature/sweep-arm-design` |
| `fix/` | 不具合の修正 | `fix/motor-direction`, `fix/sensor-calibration` |
| `hardware/` | CAD・回路・部品表の変更 | `hardware/chassis-v2`, `hardware/battery-wiring` |
| `docs/` | ルールメモ・議事録・資料 | `docs/competition-rules`, `docs/meeting-2026-11-20` |
| `tools/` | 解析・調整用ツール | `tools/log-viewer` |

悪い例: `test`, `tanaka`, `new`, `aaa`（何の作業か分からない）

関連する Issue があれば番号を付けても OK です: `fix/12-motor-direction`

## コミットメッセージ

何をしたかが一行で分かるように書いてください。日本語で構いません。

```
旋回時のモーター回転方向を修正
```

## Pull Request

- PR テンプレートの項目を埋めてください
- 関係するメンバーをレビュアーに指定してください
- 小さめの単位で出すと、レビューもしやすく衝突も起きにくくなります

## Issue

「やること」「困っていること」「決めたいこと」は Issue に書いて共有してください。
テンプレートを用意しています。

## ファイルの扱い

- CAD や画像など大きなバイナリは、同じファイルを複数人で同時に編集すると衝突を解消できません。編集前に Issue やチャットで声をかけてください
- パスワードや API キーなどの秘密情報はコミットしないでください
