# Builds and runs the match log analyzer.
#
#   .\tools\log-analyze\analyze.ps1 logs\*.wpilog
#
# Run from the repo root. Takes one or more .wpilog paths.
param([Parameter(ValueFromRemainingArguments = $true)] [string[]] $Logs)

if (-not $Logs) {
  Write-Host "usage: .\tools\log-analyze\analyze.ps1 <log.wpilog> [more.wpilog ...]"
  exit 1
}

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Split-Path -Parent (Split-Path -Parent $here)
$out = Join-Path $here "build"

Push-Location $root
try {
  Write-Host "resolving classpath..."
  $cp = (& .\gradlew.bat -q -I "$here\classpath.gradle" printCompileClasspath 2>$null | Select-Object -Last 1)
  if (-not $cp) { throw "could not resolve the compile classpath - does './gradlew build' work?" }

  New-Item -ItemType Directory -Force $out | Out-Null
  & javac -cp $cp -d $out "$here\MatchAnalyze.java"
  if ($LASTEXITCODE -ne 0) { throw "compile failed" }

  # Force an array: a single path would otherwise be a bare string, which splats wrong
  $resolved = @($Logs | ForEach-Object { (Resolve-Path $_).Path })
  & java -cp "$cp;$out" MatchAnalyze $resolved
} finally {
  Pop-Location
}
