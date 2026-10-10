#Requires -Version 5.1
[CmdletBinding()]
param(
    [ValidateSet('All', 'F401', 'F411')]
    [string] $Chip = 'All',
    [ValidateSet('All', 'uart', 'usb')]
    [string] $Port = 'All',
    [ValidateSet('Baseline', 'P3Steel', 'Features', 'All')]
    [string] $Suite = 'Baseline',
    [string] $Part = 'all',
    [switch] $List,
    [switch] $SelfTest,
    [string] $ToolsDirectory = (Join-Path $env:LOCALAPPDATA 'Teacup_Firmware\renode-tools'),
    [ValidateRange(1, 32)] [int] $Jobs = 4,
    [ValidateRange(30, 7200)] [int] $TimeoutSeconds = 1800,
    [switch] $PrepareOnly
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if (-not [Environment]::Is64BitOperatingSystem -or $env:OS -ne 'Windows_NT') {
    throw 'This launcher requires 64-bit Windows 10/11.'
}
$repoRoot = Split-Path $PSScriptRoot -Parent
$ToolsDirectory = [IO.Path]::GetFullPath($ToolsDirectory)
$resultsDir = Join-Path $PSScriptRoot 'results'
New-Item -ItemType Directory -Force $ToolsDirectory, $resultsDir | Out-Null
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
$savedPath = $env:PATH
$environmentNames = @('TEACUP_PORT', 'TEACUP_LOG_DIR', 'TEACUP_TEST_TIMEOUT_SEC')
$savedEnvironment = @{}
foreach ($name in $environmentNames) {
    $savedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}
$lock = $null

function Get-PortableTool {
    param([string] $Name, [string] $Url, [string] $Sha256 = '')
    $destination = Join-Path $ToolsDirectory $Name
    $marker = Join-Path $destination '.complete'
    if (Test-Path $marker -PathType Leaf) { return $destination }
    $archive = Join-Path $ToolsDirectory ($Name + '.zip')
    if (-not (Test-Path $archive -PathType Leaf)) {
        Write-Host "Downloading $Name ..."
        $partial = $archive + '.partial'
        $oldProgress = $ProgressPreference
        $ProgressPreference = 'SilentlyContinue'
        try {
            for ($attempt = 1; $attempt -le 3; $attempt++) {
                try {
                    Invoke-WebRequest -UseBasicParsing -Uri $Url -OutFile $partial -TimeoutSec 600
                    Move-Item $partial $archive -Force
                    break
                } catch {
                    Remove-Item $partial -Force -ErrorAction SilentlyContinue
                    if ($attempt -eq 3) { throw }
                    Start-Sleep -Seconds (2 * $attempt)
                }
            }
        } finally { $ProgressPreference = $oldProgress }
    }
    $actualHash = (Get-FileHash $archive -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($Sha256 -and $actualHash -ne $Sha256.ToLowerInvariant()) {
        Remove-Item $archive -Force
        throw "Checksum mismatch for $Name. The download was removed; retry or check your proxy."
    }
    if (Test-Path $destination) { Remove-Item $destination -Recurse -Force }
    New-Item -ItemType Directory -Force $destination | Out-Null
    Write-Host "Extracting $Name ..."
    # Windows' native tar handles large archives faster than Expand-Archive.
    $tar = Join-Path $env:SystemRoot 'System32\tar.exe'
    if (Test-Path $tar -PathType Leaf) {
        & $tar -xf $archive -C $destination
        if ($LASTEXITCODE -ne 0) { throw "Cannot extract $archive (exit $LASTEXITCODE)." }
    } else {
        Expand-Archive -LiteralPath $archive -DestinationPath $destination -Force
    }
    Set-Content -LiteralPath $marker -Value "URL=$Url`nSHA256=$actualHash" -Encoding ascii
    return $destination
}
function Find-Tool {
    param([string] $Root, [string] $Name)
    $matches = @(Get-ChildItem -LiteralPath $Root -Filter $Name -Recurse -File)
    if ($matches.Count -ne 1) {
        throw "Expected one $Name in $Root; found $($matches.Count). Remove that cached tool directory and retry."
    }
    return $matches[0].FullName
}

try {
    try {
        $lock = [IO.File]::Open((Join-Path $ToolsDirectory '.launcher.lock'),
            [IO.FileMode]::OpenOrCreate, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
    } catch { throw 'Another launcher is using this tool cache. Wait for it to finish.' }

    $renodeRoot = Get-PortableTool -Name 'renode-1.17.0' `
        -Url 'https://github.com/renode/renode/releases/download/v1.17.0/renode-1.17.0.windows-portable.zip' `
        -Sha256 '18bcf145422039e8702c3e5f9864c7787561bfd5deb7a3ee6e168ecf19e2b9db'
    $pythonRoot = Get-PortableTool -Name 'python-3.12.10' `
        -Url 'https://www.python.org/ftp/python/3.12.10/python-3.12.10-embed-amd64.zip' `
        -Sha256 '4acbed6dd1c744b0376e3b1cf57ce906f9dc9e95e68824584c8099a63025a3c3'
    $armRoot = Get-PortableTool -Name 'arm-14.3.rel1' `
        -Url 'https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-mingw-w64-i686-arm-none-eabi.zip'
    $makeRoot = Get-PortableTool -Name 'windows-build-tools-4.4.1-3' `
        -Url 'https://github.com/xpack-dev-tools/windows-build-tools-xpack/releases/download/v4.4.1-3/xpack-windows-build-tools-4.4.1-3-win32-x64.zip' `
        -Sha256 '113d4dfdbbc56dc9b865c9f75d38cd0da82f0d7094187e6f7a803fe6eef1d218'

    $python = Find-Tool $pythonRoot 'python.exe'
    $compiler = Find-Tool $armRoot 'arm-none-eabi-gcc.exe'
    $make = Find-Tool $makeRoot 'make.exe'
    $shell = Find-Tool $makeRoot 'sh.exe'
    $renode = Find-Tool $renodeRoot 'Renode.exe'
    $env:PATH = @((Split-Path $compiler), (Split-Path $make), (Split-Path $shell),
        (Split-Path $python), $savedPath) -join ';'
    foreach ($command in @(@($python, '--version'), @($compiler, '--version'), @($make, '--version'))) {
        $exe = $command[0]
        & $exe $command[1]
        if ($LASTEXITCODE -ne 0) { throw "Cannot run $exe (exit $LASTEXITCODE)." }
    }
    Write-Host "Renode: $renode"
    Write-Host "Tool cache: $ToolsDirectory"
    if ($PrepareOnly) {
        Write-Host 'All components are ready. Run again without -PrepareOnly to build and test.'
        exit 0
    }

    $runner = Join-Path $PSScriptRoot 'renode\launch_tests.py'
    $arguments = @('--suite', $Suite, '--chip', $Chip, '--port', $Port,
        '--part', $Part, '--renode', $renode, '--make', $make,
        '--jobs', [string] $Jobs, '--timeout', [string] $TimeoutSeconds)
    if ($List) { $arguments += '--list' }
    if ($SelfTest) {
        $runner = Join-Path $PSScriptRoot 'renode\test_launcher.py'
        $arguments = @()
    }
    # The embedded distribution isolates sys.path. Add this script's directory
    # explicitly, without installing packages or changing its cached ._pth file.
    $bootstrap = "import sys,runpy;sys.path.insert(0,sys.argv.pop(1));runpy.run_path(sys.argv.pop(1),run_name='__main__')"
    try {
        $ErrorActionPreference = 'Continue'
        & $python '-u' '-c' $bootstrap (Split-Path $runner) $runner @arguments 2>&1 |
            ForEach-Object { $_.ToString() } |
            Tee-Object -FilePath (Join-Path $resultsDir 'launcher.log')
        $testExit = $LASTEXITCODE
    } finally { $ErrorActionPreference = 'Stop' }
    exit $testExit
} catch {
    Write-Host "`nERROR: $($_.Exception.Message)"
    Write-Host "Tool cache: $ToolsDirectory"
    Write-Host "Logs: $resultsDir"
    exit 2
} finally {
    $env:PATH = $savedPath
    foreach ($name in $environmentNames) {
        [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name], 'Process')
    }
    if ($lock) { $lock.Dispose() }
}
