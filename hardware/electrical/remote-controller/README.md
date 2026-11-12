# リモコン基板（rimocon）

機体を手で動かして試すためのリモコンの基板です。**KiCad** で設計しています。プログラムは [`software/firmware/remote_controller/`](../../../software/firmware/remote_controller/) にあります。

## 主な部品

| 部品 | 内容 |
| --- | --- |
| U1 | Seeed Studio XIAO ESP32C3（ESP-NOW で機体に送信） |
| DS1 | AQM0802A（8 文字 × 2 行の I2C 液晶） |
| SW1〜SW8 | タクトスイッチ 8 個（A / B / X / Y / L / R / F / Back） |
| D1, D2 | LED |

## ファイル

| ファイル | 内容 |
| --- | --- |
| `rimocon.kicad_pro` | プロジェクト（これを KiCad で開く） |
| `rimocon.kicad_sch` / `rimocon.kicad_pcb` | 回路図 / 基板 |
| `production/` | 基板発注用のデータ |
| `lib/xiao_esp32c3/` | XIAO ESP32C3 のシンボルとフットプリント（GPL-3.0、作者のライセンスファイル付き） |

## 足りないもの：AQM0802A のライブラリ

AQM0802A のシンボルとフットプリントは SamacSys（Component Search Engine）のものです。このライブラリは規約で配布が禁止されているので、リポジトリには入れていません。部品を変更するときは、各自で [Component Search Engine](https://componentsearchengine.com/) から「AQM0802A」をダウンロードしてください。回路図と基板を見るだけなら、ダウンロードしなくても大丈夫です。
