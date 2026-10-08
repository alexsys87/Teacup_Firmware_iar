using System.Windows;
using System.Windows.Threading;
using TeacupHost.Infrastructure;
using TeacupHost.Services;
using TeacupHost.ViewModels;

namespace TeacupHost;

public partial class App : Application
{
    protected override void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);
        DispatcherUnhandledException += OnUnhandled;
        NumberBoxCommit.Register();

        var settings = AppSettings.Load();
        // Language and theme before the window: its texts and colors come from them.
        Loc.Apply(settings.Language ?? Loc.SystemLanguage);
        ThemeService.Apply(settings.DarkTheme);
        var vm = new MainViewModel(settings);
        var window = new MainWindow(vm);
        MainWindow = window;
        window.Show();

        // A G-code file given on the command line (or "Open with").
        if (e.Args.Length > 0 && System.IO.File.Exists(e.Args[0]))
            _ = vm.LoadFileAsync(e.Args[0]);
    }

    private static void OnUnhandled(object sender, DispatcherUnhandledExceptionEventArgs e)
    {
        // Keep a copy for bug reports: %LOCALAPPDATA%\TeacupHost\error.log.
        try
        {
            string dir = System.IO.Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "TeacupHost");
            System.IO.Directory.CreateDirectory(dir);
            System.IO.File.AppendAllText(System.IO.Path.Combine(dir, "error.log"),
                $"{DateTime.Now:yyyy-MM-dd HH:mm:ss} {e.Exception}{Environment.NewLine}{Environment.NewLine}");
        }
        catch
        {
            // Nowhere to write, the message box still shows it.
        }
        MessageBox.Show(e.Exception.ToString(), Loc.T("S.ErrorTitle"), MessageBoxButton.OK, MessageBoxImage.Error);
        e.Handled = true;
    }
}
