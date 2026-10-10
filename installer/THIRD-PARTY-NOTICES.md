# Third-party components distributed with Teacup Host

This MSI distributes only the Windows host application, not BlackPill firmware.
It does not add or change the license of the project's own source code.

## WPF UI 3.0.5

- Project: https://github.com/lepoco/wpfui
- Copyright (c) 2021-2024 Leszek Pomianowski and WPF UI Contributors.
- License: MIT; see `licenses/WPF-UI-LICENSE.txt`.

## .NET 8 runtime, WPF and System.IO.Ports 8.0.0

- Projects: https://github.com/dotnet/runtime and https://github.com/dotnet/wpf
- Copyright (c) .NET Foundation and Contributors.
- License: MIT; see `licenses/dotnet-runtime-LICENSE.txt`.
- Runtime distributions may contain additional third-party notices. Any notice
  files supplied in the published runtime payload are retained by the installer.

The .NET SDK, WiX and GitHub Actions are build tools and are not installed by
this MSI. Full license texts listed above are shipped in the `licenses` folder.
