# AGENTS.md

## やりとり

- ユーザーへの返答・報告はすべて日本語で書く（コード、コミットメッセージ、README は英語のまま）。

## 作業上の決まり

- リポジトリのルートがそのまま mod フォルダ（`ModInfo.xml`、`Config/`、ビルドした `t3taAutopilot.dll`）。ソースは `src/`。
- 改行コードはファイルごとに違う（CRLF / LF 混在）。編集前の改行を必ず保つ。
  Windows の Python をテキストモードで書くと CRLF になり、`sed -i` は LF にするので注意。編集後は `git diff --stat` で差分行数を確かめる。
- ビルド: `dotnet build src/t3taAutopilot.csproj -c Release`（System.Net.Http の MSB3277 警告は既存のもの）。
- デプロイは `powershell.exe -NoProfile -File tools/deploy.ps1`。ゲーム起動中は DLL がロックされて失敗するので、ユーザーにゲームを閉じてもらう。
  成功したかは `cmp t3taAutopilot.dll "$APPDATA/7DaysToDie/Mods/t3taAutopilot/t3taAutopilot.dll"` で確認する。
- 配布用 zip は `powershell.exe -NoProfile -File tools/package.ps1`（`dist/t3taAutopilot-<version>.zip`。バージョンは `ModInfo.xml`）。

## 検証

- オフラインシミュレータ `tools/sim`（mod の Driver / RoutePlanner / Pilot をそのままコンパイルしている）。
  例: `AD_FENCES=1 dotnet run -c Release -- "$APPDATA/7DaysToDie/GeneratedWorlds/Lulica Mountains" 30 5`
  結果はシードごとのばらつきが大きいので、複数シード（5, 11, 23 など）の合計で判断する。
  ジャイロは `AD_GYRO=1`（旋回待機は `AD_GYRO=1 AD_LOITER=1`）。
- 走行記録（テレメトリ）は既定でオフ。開発機では `Mods/t3taAutopilot/t3taAutopilot.json` に `"telemetry": true` を入れる。
  記録先は `%APPDATA%/7DaysToDie/Mods/t3taAutopilot/telemetry`。
  集計は `tools/telemetry/summarize.py`、スタック地点の分析は `tools/telemetry/stuck_report.py`（Windows では `PYTHONUTF8=1`）。
- 地上車両の自動運転は既定でオフ（`"groundVehicles": true` で有効）。車の変更を確かめるときは開発機の設定で有効にする。
