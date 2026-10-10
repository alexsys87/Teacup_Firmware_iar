#Requires -Version 7.0
[CmdletBinding()]
param(
    [string] $Version = '1.0.0',
    [string] $OutputDirectory = 'artifacts/msi',
    [switch] $SkipPublish
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if (-not $IsWindows) { throw 'MSI packaging must run on Windows.' }
if ($Version -notmatch '^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$') {
    throw 'Version must have three numeric fields: major.minor.build (no leading zeros).'
}
$parts = $Version.Split('.') | ForEach-Object { [long] $_ }
if ($parts[0] -gt 255 -or $parts[1] -gt 255 -or $parts[2] -gt 65535) {
    throw 'MSI version limits are 255.255.65535.'
}
$repoRoot = Split-Path $PSScriptRoot -Parent
Push-Location $repoRoot
try {
    function Invoke-DotNet {
        param([string[]] $Arguments)
        & dotnet @Arguments
        if ($LASTEXITCODE -ne 0) { throw "dotnet failed with exit code $LASTEXITCODE" }
    }
    $publishDir = Join-Path $repoRoot 'artifacts/publish/win-x64'
    $generatedDir = Join-Path $repoRoot 'artifacts/installer-source'
    $outputDir = [IO.Path]::GetFullPath($OutputDirectory, $repoRoot)
    New-Item -ItemType Directory -Force $generatedDir, $outputDir | Out-Null
    if (-not $SkipPublish) {
        # Never package stale files left by a previous publish.
        if (Test-Path $publishDir) { Remove-Item $publishDir -Recurse -Force }
        Invoke-DotNet -Arguments @(
            'publish', 'host/TeacupHost/TeacupHost.csproj',
            '-c', 'Release', '-f', 'net8.0-windows', '-r', 'win-x64',
            '--self-contained', 'true',
            '-p:PublishSingleFile=false', '-p:PublishTrimmed=false',
            '-p:DebugType=none', '-p:DebugSymbols=false', "-p:Version=$Version",
            '-o', $publishDir
        )
        Copy-Item 'installer/THIRD-PARTY-NOTICES.md' $publishDir
        Copy-Item 'installer/licenses' (Join-Path $publishDir 'licenses') -Recurse
    }
    foreach ($required in @('TeacupHost.exe', 'TeacupHost.dll', 'coreclr.dll', 'PresentationFramework.dll', 'THIRD-PARTY-NOTICES.md', 'licenses/WPF-UI-LICENSE.txt', 'licenses/dotnet-runtime-LICENSE.txt')) {
        if (-not (Test-Path (Join-Path $publishDir $required) -PathType Leaf)) {
            throw "Self-contained publish is missing $required. Run without -SkipPublish."
        }
    }

    # One file per component, stable IDs based on relative paths. This also
    # includes native libraries and culture subdirectories, not only *.dll.
    $ns = 'http://wixtoolset.org/schemas/v4/wxs'
    $doc = [xml] "<Wix xmlns='$ns'><Fragment><ComponentGroup Id='PublishedFiles'/></Fragment></Wix>"
    $group = $doc.DocumentElement.FirstChild.FirstChild
    foreach ($file in (Get-ChildItem $publishDir -Recurse -File | Sort-Object FullName)) {
        $relative = [IO.Path]::GetRelativePath($publishDir, $file.FullName)
        if ($relative -ieq 'TeacupHost.exe') { continue }
        $hash = [Convert]::ToHexString([Security.Cryptography.SHA256]::HashData(
            [Text.Encoding]::UTF8.GetBytes($relative.ToLowerInvariant()))).Substring(0, 32)
        $component = $doc.CreateElement('Component', $ns)
        $component.SetAttribute('Id', "cmp_$hash")
        $component.SetAttribute('Guid', '*')
        $component.SetAttribute('Bitness', 'always64')
        $component.SetAttribute('Directory', 'INSTALLFOLDER')
        $parent = [IO.Path]::GetDirectoryName($relative)
        if ($parent) { $component.SetAttribute('Subdirectory', $parent) }
        $fileNode = $doc.CreateElement('File', $ns)
        $fileNode.SetAttribute('Id', "fil_$hash")
        $fileNode.SetAttribute('Source', $file.FullName)
        $fileNode.SetAttribute('KeyPath', 'yes')
        [void] $component.AppendChild($fileNode)
        [void] $group.AppendChild($component)
    }
    $fragment = Join-Path $generatedDir 'PublishedFiles.wxs'
    $doc.Save($fragment)
    Invoke-DotNet -Arguments @('tool', 'restore')
    Invoke-DotNet -Arguments @('tool', 'run', 'wix', 'extension', 'add', '-g', 'WixToolset.UI.wixext/5.0.2')
    $msi = Join-Path $outputDir "TeacupHost-$Version-win-x64.msi"
    Invoke-DotNet -Arguments @(
        'tool', 'run', 'wix', 'build', 'installer/TeacupHost.wxs', $fragment,
        '-arch', 'x64', '-ext', 'WixToolset.UI.wixext',
        '-d', "ProductVersion=$Version", '-d', "PublishDir=$publishDir",
        '-o', $msi
    )
    Get-FileHash $msi -Algorithm SHA256 |
        ForEach-Object { "$($_.Hash.ToLowerInvariant())  $([IO.Path]::GetFileName($msi))" } |
        Set-Content "$msi.sha256" -Encoding ascii
    Write-Host "MSI created: $msi"
} finally {
    Pop-Location
}
