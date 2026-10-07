using System.Windows;
using System.Windows.Threading;
using TeacupHost.Services;
using TeacupHost.ViewModels;

namespace TeacupHost;

public partial class App : Application
{
    protected override void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);
        DispatcherUnhandledException += OnUnhandled;

        var settings = AppSettings.Load();
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
        MessageBox.Show(e.Exception.ToString(), "Teacup Host: ошибка", MessageBoxButton.OK, MessageBoxImage.Error);
        e.Handled = true;
    }
}
