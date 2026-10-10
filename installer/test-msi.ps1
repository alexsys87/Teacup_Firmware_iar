#Requires -Version 7.0
#Requires -RunAsAdministrator
[CmdletBinding()]
param(
    [Parameter(Mandatory)] [string] $Msi,
    [Parameter(Mandatory)] [string] $BaselineMsi,
    [switch] $AllowMachineChanges
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if (-not $IsWindows -or -not $AllowMachineChanges) {
    throw 'Run on a disposable Windows machine with -AllowMachineChanges.'
}
$repoRoot = Split-Path $PSScriptRoot -Parent
$Msi = [IO.Path]::GetFullPath($Msi, $repoRoot)
$BaselineMsi = [IO.Path]::GetFullPath($BaselineMsi, $repoRoot)
$registryPath = 'HKLM:\SOFTWARE\alexsys87\TeacupHost'
if (Test-Path $registryPath) { throw 'TeacupHost is already installed; refusing to modify it.' }
$shortcut = Join-Path ([Environment]::GetFolderPath('CommonPrograms')) 'Teacup Host.lnk'
if (Test-Path $shortcut) { throw 'A TeacupHost shortcut already exists; refusing to modify it.' }
$installDir = Join-Path $env:ProgramFiles ("TeacupHost MSI Verification " + [guid]::NewGuid().ToString('N'))
$logs = Join-Path $repoRoot 'artifacts/msi-logs'
New-Item -ItemType Directory -Force $logs | Out-Null
$userDataDir = Join-Path ([Environment]::GetFolderPath('ApplicationData')) 'TeacupHost'
$hadUserDataDir = Test-Path $userDataDir
New-Item -ItemType Directory -Force $userDataDir | Out-Null
$sentinel = Join-Path $userDataDir ("msi-test-" + [guid]::NewGuid().ToString('N') + '.txt')
Set-Content $sentinel 'User data must survive MSI removal.'
$sentinelHash = (Get-FileHash $sentinel -Algorithm SHA256).Hash

# Verify the actual installed shortcut, not just the existence of a .lnk file.
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class TeacupHostShortcutIcons
{
    [DllImport("shell32.dll", CharSet = CharSet.Unicode, EntryPoint = "ExtractIconExW")]
    public static extern uint ExtractIconEx(string file, int index,
        [Out] IntPtr[] large, [Out] IntPtr[] small, uint count);
    [DllImport("user32.dll")]
    public static extern bool DestroyIcon(IntPtr icon);
}
'@

function Invoke-Msi {
    param([string] $Arguments, [string] $LogName, [int[]] $AllowedCodes = @(0, 3010))
    $log = Join-Path $logs "$LogName.log"
    $process = Start-Process msiexec.exe -ArgumentList "$Arguments /qn /norestart /L*v `"$log`"" -Wait -PassThru
    if ($process.ExitCode -notin $AllowedCodes) {
        throw "msiexec $LogName failed: $($process.ExitCode); see $log"
    }
    return $process.ExitCode
}
function Assert-Installed {
    param([bool] $CheckIcon = $true)
    if (-not (Test-Path $registryPath)) { throw 'Install-location registry value is missing.' }
    $actual = (Get-ItemProperty $registryPath).InstallDir.TrimEnd('\')
    if ($actual -ine $installDir.TrimEnd('\')) { throw "Installation path changed: $actual" }
    if (-not (Test-Path $shortcut -PathType Leaf)) { throw 'Start menu shortcut is missing.' }
    if ($CheckIcon) {
        $shell = New-Object -ComObject WScript.Shell
        $link = $shell.CreateShortcut($shortcut)
        $location = $link.IconLocation
        if ($location -notmatch '^(.*),\s*(-?\d+)$') { throw "Invalid shortcut icon location: $location" }
        $iconPath = [Environment]::ExpandEnvironmentVariables($Matches[1].Trim('"'))
        $iconIndex = [int] $Matches[2]
        if ([IO.Path]::GetExtension($iconPath) -ine '.exe' -or $iconIndex -ne 0) {
            throw "Shortcut must use the EXE icon resource at index 0: $location"
        }
        if (-not (Test-Path $iconPath -PathType Leaf)) { throw "Missing cached shortcut icon: $iconPath" }
        $expectedExe = Join-Path $repoRoot 'artifacts/publish/win-x64/TeacupHost.exe'
        if ((Get-FileHash $iconPath).Hash -ne (Get-FileHash $expectedExe).Hash) {
            throw 'Cached shortcut icon is not the published TeacupHost EXE resource.'
        }
        $large = [IntPtr[]]::new(1)
        $small = [IntPtr[]]::new(1)
        try {
            $count = [TeacupHostShortcutIcons]::ExtractIconEx($iconPath, $iconIndex, $large, $small, 1)
            if ($count -eq 0 -or $large[0] -eq [IntPtr]::Zero -or $small[0] -eq [IntPtr]::Zero) {
                throw "Windows cannot extract the shortcut icon: $location"
            }
        } finally {
            foreach ($handle in @($large[0], $small[0])) {
                if ($handle -ne [IntPtr]::Zero) { [void] [TeacupHostShortcutIcons]::DestroyIcon($handle) }
            }
            [void] [Runtime.InteropServices.Marshal]::FinalReleaseComObject($link)
            [void] [Runtime.InteropServices.Marshal]::FinalReleaseComObject($shell)
        }
    }
    $publishDir = Join-Path $repoRoot 'artifacts/publish/win-x64'
    foreach ($source in (Get-ChildItem $publishDir -Recurse -File)) {
        $relative = [IO.Path]::GetRelativePath($publishDir, $source.FullName)
        $installed = Join-Path $installDir $relative
        if (-not (Test-Path $installed -PathType Leaf)) { throw "Missing installed file: $relative" }
        if ((Get-FileHash $source.FullName).Hash -ne (Get-FileHash $installed).Hash) {
            throw "Installed file differs: $relative"
        }
    }
    Assert-UserData
}
function Assert-UserData {
    if (-not (Test-Path $sentinel -PathType Leaf)) { throw 'User data was removed.' }
    if ((Get-FileHash $sentinel -Algorithm SHA256).Hash -ne $sentinelHash) {
        throw 'User data was modified.'
    }
}
function Assert-Uninstalled {
    if (Test-Path $shortcut) { throw 'Shortcut was not removed.' }
    if (Test-Path $registryPath) { throw 'Install-location registration was not removed.' }
    $publishDir = Join-Path $repoRoot 'artifacts/publish/win-x64'
    foreach ($source in (Get-ChildItem $publishDir -Recurse -File)) {
        $relative = [IO.Path]::GetRelativePath($publishDir, $source.FullName)
        if (Test-Path (Join-Path $installDir $relative)) { throw "Installed file was not removed: $relative" }
    }
    Assert-UserData
}
try {
    [void] (Invoke-Msi "/i `"$BaselineMsi`" INSTALLFOLDER=`"$installDir`"" 'install-baseline')
    Assert-Installed
    # Deliberately omit INSTALLFOLDER: the upgrade must retain the custom path.
    [void] (Invoke-Msi "/i `"$Msi`"" 'upgrade')
    Assert-Installed
    [void] (Invoke-Msi "/i `"$BaselineMsi`"" 'blocked-downgrade' @(1603))
    Assert-Installed
    Remove-Item (Join-Path $installDir 'TeacupHost.dll') -Force
    [void] (Invoke-Msi "/fa `"$Msi`"" 'repair')
    Assert-Installed
    [void] (Invoke-Msi "/x `"$Msi`"" 'uninstall')
    Assert-Uninstalled
    # Fresh install, independently of the upgrade path.
    [void] (Invoke-Msi "/i `"$Msi`" INSTALLFOLDER=`"$installDir`"" 'fresh-install')
    Assert-Installed
    [void] (Invoke-Msi "/x `"$Msi`"" 'fresh-uninstall')
    Assert-Uninstalled
    Write-Host 'MSI installation, repair, upgrade, downgrade blocking and uninstall passed.'
} finally {
    # /x on an absent product returns 1605; do not mask the original test failure.
    foreach ($package in @($Msi, $BaselineMsi)) {
        try { [void] (Invoke-Msi "/x `"$package`"" ('cleanup-' + [IO.Path]::GetFileNameWithoutExtension($package)) @(0, 3010, 1605)) }
        catch { Write-Warning $_ }
    }
    Remove-Item $sentinel -Force -ErrorAction SilentlyContinue
    if (-not $hadUserDataDir -and (Test-Path $userDataDir) -and @(Get-ChildItem $userDataDir -Force).Count -eq 0) {
        Remove-Item $userDataDir -Force
    }
}
