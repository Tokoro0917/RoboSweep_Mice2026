# RoboSweep Mice2026

Mice で開催されるロボスイープ競技に出場するためのチームリポジトリです。
設計データ・ソースコード・資料をここに集約し、メンバー全員で編集します。

## ディレクトリ構成

| パス | 内容 |
| --- | --- |
| `docs/` | 資料全般 |
| `docs/rules/` | 競技ルール・レギュレーションのメモ |
| `docs/meetings/` | 打ち合わせの議事録（`YYYY-MM-DD_議題.md`） |
| `hardware/cad/` | 機構設計（CAD データ、図面） |
| `hardware/electrical/` | 回路図・配線図・部品表 |
| `software/firmware/` | ロボット本体のプログラム |
| `software/tools/` | 調整・解析用のツールやスクリプト |

## リポジトリの使い方

コマンドを打つ方法（ターミナル）と、画面で操作する方法（GitHub Desktop）があります。Git に慣れていない人は GitHub Desktop がおすすめです。

### 1. 最初に 1 回だけやること

1. **GitHub のアカウントを作る**：<https://github.com/> で作ります。
2. **招待を受け取る**：リポジトリの管理者に GitHub のユーザー名を伝えます。届いた招待メールの「Accept invitation」を押してください。これで編集できるようになります。
3. **道具を入れる**（どちらか片方）
   - 画面で操作する：[GitHub Desktop](https://desktop.github.com/) を入れて、GitHub のアカウントでログインする
   - コマンドで操作する：[Git](https://git-scm.com/downloads) を入れて、名前とメールアドレスを設定する
     ```bash
     git config --global user.name "あなたの名前"
     git config --global user.email "GitHub に登録したメールアドレス"
     ```
4. **クローンする**（リポジトリを自分の PC にコピーする）
   - GitHub Desktop：「File → Clone repository」で `Tokoro0917/RoboSweep_Mice2026` を選ぶ
   - コマンド：
     ```bash
     git clone https://github.com/Tokoro0917/RoboSweep_Mice2026.git
     cd RoboSweep_Mice2026
     ```
     初めて push するときは、ブラウザで GitHub にログインする画面が出ます（Windows / Mac の Git では自動で出ます）。出ないときは、GitHub のパスワードではなく [Personal access token](https://github.com/settings/tokens) を使ってください。

### 2. 作業して GitHub に上げる手順

`main` ブランチには**直接 push できません**（保護しています）。必ず「ブランチを作る → push → Pull Request」の順で上げます。

| 手順 | GitHub Desktop | コマンド |
| --- | --- | --- |
| ① `main` を最新にする | 上の「Current branch」で `main` を選び、「Fetch origin」→「Pull origin」 | `git switch main`<br>`git pull` |
| ② 作業用のブランチを作る | 「Current branch」→「New branch」。名前は [ブランチ名のルール](CONTRIBUTING.md#ブランチ名のルール) に従う | `git switch -c feature/区画走行` |
| ③ ファイルを編集する | いつもどおり CAD・エディタなどで編集・保存 | 同じ |
| ④ コミットする（変更を記録する） | 左下の「Summary」に何をしたかを書いて「Commit to ブランチ名」 | `git add 変えたファイル`<br>`git commit -m "何をしたか"` |
| ⑤ GitHub に上げる（push） | 「Push origin」（初回は「Publish branch」） | `git push -u origin ブランチ名` |
| ⑥ Pull Request を作る | 「Create Pull Request」→ ブラウザで内容を書いて作成 | GitHub のページに出る「Compare & pull request」を押す |
| ⑦ 見てもらってマージする | ほかのメンバーに確認してもらい、GitHub の PR 画面で「Merge pull request」 | 同じ |
| ⑧ 片付ける | `main` に戻って ① をやる。終わったブランチは消してよい | `git switch main`<br>`git pull` |

コミットは小さく、こまめにするのがおすすめです。PR を出したあとに直したくなったら、同じブランチでコミットして push すると、その PR に追加されます。

### 3. 困ったとき

- **push が拒否された（rejected）**：ほかの人が先に同じブランチを更新しています。pull してから、もう一度 push してください（GitHub Desktop は「Pull origin」、コマンドは `git pull`）。
- **衝突した（conflict）**：同じファイルの同じ所を 2 人が変えると起きます。テキストなら `<<<<<<<` と `>>>>>>>` の間を見て、正しいほうを残します。**CAD（.f3z）や KiCad のファイルは衝突を直せない**ので、編集する前に Issue やチャットで声をかけてください。
- **間違えて `main` で作業していた**：コミットする前なら、そのまま ② でブランチを作れば変更はそのブランチに移ります。
- 分からなくなったら、何をしたかと画面のメッセージをチャットに貼って聞いてください。

### 4. 気をつけること

- パスワードや API キーはコミットしない
- このリポジトリは公開（Public）です。公式の参考機体「ムブアイ」の資料はコミットしない（[ルールまとめ](docs/rules/robosweep-rules.md) を参照）
- ビルドで作られるファイル（`Debug/` など）や、CAD の自動バックアップは入れない（`.gitignore` で除外しています）
- やること・困っていること・決めたいことは [Issue](https://github.com/Tokoro0917/RoboSweep_Mice2026/issues) に書く

ブランチ名・コミットメッセージ・PR の書き方の決まりは [CONTRIBUTING.md](CONTRIBUTING.md) にあります。

## スケジュール

| 日付 | 内容 |
| --- | --- |
| TBD | 大会当日 |

<!-- 決まったら追記してください -->
