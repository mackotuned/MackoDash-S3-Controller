# Builds and runs the knob UI preview with the MSVC Build Tools, then writes
# preview PNGs (circular mask, as on the round panel) next to the raw frames.
# usage: build_preview.ps1 <out_dir>
param([Parameter(Mandatory = $true)][string]$OutDir)
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$main = Resolve-Path (Join-Path $here "..\..\main")
New-Item -ItemType Directory -Force $OutDir | Out-Null
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vs = & $vswhere -latest -products * -property installationPath
$vcvars = Join-Path $vs "VC\Auxiliary\Build\vcvars64.bat"
$exe = Join-Path $OutDir "knob_preview.exe"
$sources = "`"$here\preview.c`" `"$main\knob_ui.c`" `"$main\knob_fonts.c`""
cmd /c "`"$vcvars`" >nul 2>nul && cl /nologo /O2 /std:c11 /I`"$main`" $sources /Fe:`"$exe`" /Fo:`"$OutDir\\`" >`"$OutDir\cl.log`" 2>&1"
if ($LASTEXITCODE -ne 0) { Get-Content "$OutDir\cl.log"; exit 1 }
& $exe $OutDir
