# Build the release zip: dist\t3taAutopilot-<version>.zip, holding a
# t3taAutopilot folder to extract into the game's Mods folder.
$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot

dotnet build -c Release (Join-Path $repoRoot 'src\t3taAutopilot.csproj')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$info = New-Object System.Xml.XmlDocument
$info.Load((Join-Path $repoRoot 'ModInfo.xml'))
$version = $info.DocumentElement.SelectSingleNode('Version').GetAttribute('value')

$dist = Join-Path $repoRoot 'dist'
$stage = Join-Path $dist 't3taAutopilot'
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Force -Path $stage | Out-Null
foreach ($f in 'ModInfo.xml', 't3taAutopilot.dll', 't3taAutopilot.pdb', 'README.md', 'LICENSE') {
    Copy-Item (Join-Path $repoRoot $f) $stage
}
Copy-Item -Recurse (Join-Path $repoRoot 'Config') $stage

$zip = Join-Path $dist "t3taAutopilot-$version.zip"
if (Test-Path $zip) { Remove-Item -Force $zip }
Compress-Archive -Path $stage -DestinationPath $zip

Write-Host "Packaged $zip" -ForegroundColor Green
