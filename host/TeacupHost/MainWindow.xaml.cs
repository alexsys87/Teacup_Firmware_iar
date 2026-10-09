using System.ComponentModel;
using System.IO;
using System.Windows;
using TeacupHost.Services;
using TeacupHost.ViewModels;

namespace TeacupHost;

public partial class MainWindow : Wpf.Ui.Controls.FluentWindow
{
    /// <summary>Position of the Axes tab in LeftTabs.</summary>
    private const int AxesTabIndex = 1;

    private readonly MainViewModel _vm;
    private readonly InputController _input;

    public MainWindow(MainViewModel vm)
    {
        _vm = vm;
        DataContext = vm;
        InitializeComponent();
        Drop += OnDrop;
        DragOver += OnDragOver;

        // Keyboard and gamepad control of the axes.
        _input = new InputController(vm, this, () => LeftTabs.SelectedIndex == AxesTabIndex);
        PreviewKeyDown += (_, e) => _input.OnPreviewKeyDown(e);
    }

    private void OnDragOver(object sender, DragEventArgs e)
    {
        e.Effects = e.Data.GetDataPresent(DataFormats.FileDrop) ? DragDropEffects.Copy : DragDropEffects.None;
        e.Handled = true;
    }

    private void OnDrop(object sender, DragEventArgs e)
    {
        if (e.Data.GetData(DataFormats.FileDrop) is string[] { Length: > 0 } files && File.Exists(files[0]))
            _ = _vm.LoadFileAsync(files[0]);
    }

    protected override void OnClosing(CancelEventArgs e)
    {
        if (_vm.IsPrinting && _vm.PrintSource == PrintSource.Host &&
            MessageBox.Show(Services.Loc.T("S.Ask.CloseWhilePrinting"),
                "Teacup Host", MessageBoxButton.YesNo, MessageBoxImage.Warning) != MessageBoxResult.Yes)
        {
            e.Cancel = true;
            return;
        }
        _input.Dispose();
        _vm.SaveSettings();
        _vm.Dispose();
        base.OnClosing(e);
    }
}
