# Deploy the mod into the user-level 7DTD Mods folder.
# The game loads mods from <UserGameData>/Mods (i.e. %APPDATA%\7DaysToDie\Mods)
# in addition to the install dir, so no admin rights are needed.
param(
    [string]$GameDataDir = "$env:APPDATA\7DaysToDie"
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot   # the repo root is the mod folder
$modsDir = Join-Path $GameDataDir 'Mods'
$dstMod = Join-Path $modsDir 't3taAutopilot'

# Before its release the mod was called AutoDrive: carry that folder over
# (telemetry, bookmarks, config). Left in place, the game would load both.
$oldMod = Join-Path $modsDir 'AutoDrive'
if (Test-Path $oldMod) {
    if (Test-Path $dstMod) { throw "Both $oldMod and $dstMod exist - merge them by hand." }
    Rename-Item $oldMod 't3taAutopilot'
    foreach ($f in 'AutoDrive.dll', 'AutoDrive.pdb') {
        $old = Join-Path $dstMod $f
        if (Test-Path $old) { Remove-Item -Force $old }
    }
    $oldCfg = Join-Path $dstMod 'AutoDrive.json'
    if (Test-Path $oldCfg) { Rename-Item $oldCfg 't3taAutopilot.json' }
    Write-Host "Moved $oldMod -> $dstMod" -ForegroundColor Yellow
}

$dll = Join-Path $repoRoot 't3taAutopilot.dll'
if (-not (Test-Path $dll)) {
    Write-Host "t3taAutopilot.dll not found - building..." -ForegroundColor Yellow
    dotnet build -c Release (Join-Path $repoRoot 'src\t3taAutopilot.csproj')
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

New-Item -ItemType Directory -Force -Path $dstMod | Out-Null
Copy-Item -Force (Join-Path $repoRoot 'ModInfo.xml') $dstMod
Copy-Item -Force $dll                                $dstMod
$pdb = Join-Path $repoRoot 't3taAutopilot.pdb'
if (Test-Path $pdb) { Copy-Item -Force $pdb $dstMod }

# XUi window definitions (destination list)
Copy-Item -Force -Recurse (Join-Path $repoRoot 'Config') $dstMod

Write-Host "Deployed t3taAutopilot -> $dstMod" -ForegroundColor Green
Get-ChildItem $dstMod | Format-Table Name, Length -AutoSize
